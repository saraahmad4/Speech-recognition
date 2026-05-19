#define F_CPU 11059200UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
// #include "lcd_helpers.h"
// #include "uart_helpers.h"

// System config
#define UART_BAUD 9600UL
#define FRAME_SIZE 128
#define NUM_FRAMES 50
#define NUM_SEGMENTS 5
#define FRAMES_PER_SEGMENT 8
#define TOTAL_FRAMES (NUM_SEGMENTS * FRAMES_PER_SEGMENT)
#define PREROLL_FRAMES 5
#define CALIBRATE_FRAMES 5
#define DEADZONE 10
#define VAD_THRESHOLD_MULTIPLIER 8.0f
#define DEFAULT_NOISE_FLOOR 10.0f
#define REJECT_DISTANCE_THRESHOLD 1.0f
#define ENERGY_WEIGHT 1.0f
#define ZCR_WEIGHT 6.0f
#define NUM_WORDS 8
#define NUM_FEATURES 10

// Button pins
#define BUTTON_START_PIN PD2
#define BUTTON_CALIBRATE_PIN PD7

// LCD pins (8-bit mode)
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR DDRC
#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR DDRD
#define LCD_RS PD3
#define LCD_RW PD4
#define LCD_EN PD5

// Global states
volatile uint16_t adc_sample = 0;
volatile uint8_t frame_buffer[FRAME_SIZE];
volatile uint8_t frame_index = 0;
volatile bool frame_ready = false;

volatile bool recognition_requested = false;
volatile bool capture_complete = false;

// UART command interface
#define UART_CMD_BUFFER_SIZE 32
char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
static uint8_t uart_cmd_index = 0;

// Online segment accumulation (no frame storage)
float online_energies[NUM_SEGMENTS] = {0};
float online_zcrs[NUM_SEGMENTS] = {0};
static uint8_t current_segment = 0;
static uint16_t frames_in_segment = 0;
static uint16_t post_vad_frames = 0;

static bool vad_triggered = false;
static float noise_floor = DEFAULT_NOISE_FLOOR;
static uint16_t recording_frame_count = 0;

// Word labels and templates
static const char word_names[NUM_WORDS][7] = {
    "FREEZE",
    "GRAB",
    "JUMP",
    "LIFT",
    "ON",
    "PUSH",
    "RIGHT",
    "START"
};

static const float word_templates[NUM_WORDS][NUM_FEATURES] PROGMEM = {
    {0.6034, 1.0000, 0.5798, 0.0360, 0.0000, 0.1978, 0.1332, 0.1472, 0.0806, 0.0244}, // FREEZE
    {0.4892, 1.0000, 0.2620, 0.0016, 0.0000, 0.2002, 0.4810, 0.2072, 0.0074, 0.0000}, // GRAB
    {0.4506, 1.0000, 0.3728, 0.1152, 0.0642, 0.2160, 0.3328, 0.0942, 0.0256, 0.0056}, // JUMP
    {1.0000, 0.6102, 0.0322, 0.0420, 0.0100, 0.1406, 0.1464, 0.0722, 0.0746, 0.0212}, // LIFT
    {1.0000, 0.2338, 0.0000, 0.0000, 0.0000, 0.3518, 0.0892, 0.0296, 0.0000, 0.0000}, // ON
    {1.0000, 0.4208, 0.1698, 0.0528, 0.0000, 0.2300, 0.4052, 0.3654, 0.0906, 0.0006}, // PUSH
    {1.0000, 0.4828, 0.0030, 0.0006, 0.0000, 0.3126, 0.1536, 0.0524, 0.0128, 0.0002}, // RIGHT
    {0.1134, 1.0000, 0.5330, 0.0000, 0.0000, 0.2728, 0.4036, 0.2882, 0.0150, 0.0094}, // START
};

// ============================================================
// Forward declarations
//// ============================================================
static void reset_capture_state(void);
static void lcd_clear(void);
static void lcd_print(const char *str);
static void calibrate_noise_floor(void);
static void start_recognition(void);
static void handle_uart_command(const char *cmd);
static void receive_input();

// ============================================================
// Utility helpers
// ============================================================
static void uart_init(void) {
   uint16_t ubrr = (F_CPU / (16UL * UART_BAUD)) - 1;
   UBRRH = (uint8_t)(ubrr >> 8);
   UBRRL = (uint8_t)ubrr;
   UCSRB = (1 << TXEN) | (1 << RXEN);
   UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

static void uart_transmit(uint8_t data) {
   while (!(UCSRA & (1 << UDRE)));
   UDR = data;
}

static void uart_puts(const char *str) {
   while (*str) {
       uart_transmit((uint8_t)*str++);
   }
}

static void uart_print_uint16(uint16_t value) {
   char buf[6];
   uint8_t idx = 0;
   if (value == 0) {
       uart_transmit('0');
       return;
   }
   while (value > 0 && idx < sizeof(buf)) {
       buf[idx++] = '0' + (value % 10);
       value /= 10;
   }
   for (int8_t i = idx - 1; i >= 0; i--) {
       uart_transmit(buf[i]);
   }
}

static void uart_print_float(float value, uint8_t decimals) {
   if (value < 0.0f) {
       uart_transmit('-');
       value = -value;
   }
   uint16_t integer = (uint16_t)value;
   uart_print_uint16(integer);
   uart_transmit('.');
   float remainder = value - (float)integer;
   for (uint8_t i = 0; i < decimals; i++) {
       remainder *= 10.0f;
       uint8_t digit = (uint8_t)remainder;
       uart_transmit('0' + digit);
       remainder -= digit;
   }
}

// ============================================================
// UART command interface
// ============================================================
static uint8_t uart_receive(void) {
   while (!(UCSRA & (1 << RXC)));
   return UDR;
}

static bool uart_data_available(void) {
   return (UCSRA & (1 << RXC)) != 0;
}

static void uart_process_input(void) {
   if (uart_data_available()) {
       uint8_t ch = uart_receive();
       
       // Handle backspace
       if (ch == 8 || ch == 127) {
           if (uart_cmd_index > 0) {
               uart_cmd_index--;
               uart_puts("\b \b");
           }
           return;
       }
       
       // Handle newline/carriage return
       if (ch == '\r' || ch == '\n') {
           uart_transmit('\r');
           uart_transmit('\n');
           uart_cmd_buffer[uart_cmd_index] = '\0';
           
           // Process the command
           if (uart_cmd_index > 0) {
               handle_uart_command(uart_cmd_buffer);
           }
           
           uart_cmd_index = 0;
//            uart_puts("> ");
           return;
       }
       
       // Store printable characters
       if (ch >= 32 && ch < 127 && uart_cmd_index < UART_CMD_BUFFER_SIZE - 1) {
           uart_cmd_buffer[uart_cmd_index++] = ch;
       }
   }
}

static void handle_uart_command(const char *cmd) {
   // Skip leading spaces
   uint8_t idx = 0;
   while (cmd[idx] == ' ' && cmd[idx] != '\0') idx++;
   
   const char *start = &cmd[idx];
   uint8_t cmd_len = 0;
   
   // Find command length (until space or null terminator)
   while (start[cmd_len] != ' ' && start[cmd_len] != '\0') {
       cmd_len++;
   }
   
   // Convert to uppercase for comparison
   char cmd_upper[32];
   for (uint8_t i = 0; i < cmd_len && i < 31; i++) {
       if (start[i] >= 'a' && start[i] <= 'z') {
           cmd_upper[i] = start[i] - 'a' + 'A';
       } else {
           cmd_upper[i] = start[i];
       }
   }
   cmd_upper[cmd_len] = '\0';
   
   // Process commands
   if (cmd_len == 5 && cmd_upper[0] == 'R' && cmd_upper[1] == 'E' && 
       cmd_upper[2] == 'S' && cmd_upper[3] == 'E' && cmd_upper[4] == 'T') {
       // RESET command
       uart_puts("[UART] Executing RESET\r\n");
       reset_capture_state();
       vad_triggered = false;
       noise_floor = DEFAULT_NOISE_FLOOR;
       recording_frame_count = 0;
       current_segment = 0;
       frames_in_segment = 0;
       recognition_requested = false;
       capture_complete = false;
       lcd_clear();
       lcd_print("Ready");
       uart_puts("[UART] RESET complete\r\n");
   } 
   else if (cmd_len == 5 && cmd_upper[0] == 'N' && cmd_upper[1] == 'O' && 
            cmd_upper[2] == 'I' && cmd_upper[3] == 'S' && cmd_upper[4] == 'E' ) {
       // CAPTURE NOISE command
       uart_puts("[UART] Executing CAPTURE NOISE\r\n");
       calibrate_noise_floor();
   } 
   else if (cmd_len == 5 && cmd_upper[0] == 'S' && cmd_upper[1] == 'T' && 
            cmd_upper[2] == 'A' && cmd_upper[3] == 'R' && cmd_upper[4] == 'T') {
       // START command
       uart_puts("[UART] Executing START\r\n");
       start_recognition();
   } 
   else {
       // Invalid command
       uart_puts("[UART] Invalid command: ");
       uart_puts(start);
       receive_input();
   }
}

//============================================================
//LCD helpers
//============================================================
static void lcd_pulse_enable(void) {
   LCD_CTRL_PORT |= (1 << LCD_EN);
   _delay_us(1);
   LCD_CTRL_PORT &= ~(1 << LCD_EN);
   _delay_us(100);
}

static void lcd_write_byte(uint8_t value, bool command) {
   if (command) {
       LCD_CTRL_PORT &= ~(1 << LCD_RS);
   } else {
       LCD_CTRL_PORT |= (1 << LCD_RS);
   }
   LCD_CTRL_PORT &= ~(1 << LCD_RW);
   LCD_DATA_PORT = value;
   _delay_us(1);
   lcd_pulse_enable();
}

static void lcd_init(void) {
   LCD_DATA_DDR = 0xFF;
   LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN);
   _delay_ms(40);
   lcd_write_byte(0x38, true);
   _delay_ms(5);
   lcd_write_byte(0x38, true);
   _delay_us(150);
   lcd_write_byte(0x38, true);
   lcd_write_byte(0x0C, true);
   lcd_write_byte(0x06, true);
   lcd_write_byte(0x01, true);
   _delay_ms(2);
}

static void lcd_clear(void) {
   lcd_write_byte(0x01, true);
   _delay_ms(5);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
   uint8_t address = (row == 0) ? 0x00 : 0x40;
   address += col;
   lcd_write_byte(0x80 | address, true);
}

static void lcd_print(const char *str) {
   while (*str) {
       lcd_write_byte((uint8_t)*str++, false);
   }
}

static void lcd_print_float(float value, uint8_t decimals)
{
   if (value < 0.0f) {
       lcd_write_byte('-', false);
       value = -value;
   }

   uint16_t integer = (uint16_t)value;
   uint16_t temp = integer;

   // Print integer part
   char buf[6];
   uint8_t idx = 0;

   if (temp == 0) {
       lcd_write_byte('0', false);
   } else {
       while (temp > 0 && idx < sizeof(buf)) {
           buf[idx++] = '0' + (temp % 10);
           temp /= 10;
       }

       for (int8_t i = idx - 1; i >= 0; i--) {
           lcd_write_byte(buf[i], false);
       }
   }

   // Print decimal point
   lcd_write_byte('.', false);

   // Fractional part
   float remainder = value - (float)integer;

   for (uint8_t i = 0; i < decimals; i++) {
       remainder *= 10.0f;

       uint8_t digit = (uint8_t)remainder;

       lcd_write_byte('0' + digit, false);

       remainder -= digit;
   }
}

// ============================================================
// ADC + Timer setup
// ============================================================
static void adc_init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static void timer0_init(void) {
    TCCR0 = (1 << WGM01) | (1 << CS01);
    OCR0 = 172;
    TIMSK |= (1 << OCIE0);
}

ISR(TIMER0_COMP_vect) {
    ADCSRA |= (1 << ADSC);
}

ISR(ADC_vect) {
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    adc_sample = (high << 8) | low;
    frame_buffer[frame_index++] =(uint8_t) (adc_sample >> 2);
    if (frame_index >= FRAME_SIZE) {
        frame_index = 0;
        frame_ready = true;
    }
}

static void receive_input(void){
    uart_puts("\r\nSpeech recognizer ready\r\n");
    uart_puts("Commands: 'noise', 'start', 'reset'\r\n");
    uart_puts("> ");
}
// ============================================================
// Feature extraction
// ============================================================
static void compute_frame_features(const uint8_t frame[FRAME_SIZE], float *out_energy, float *out_zcr) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < FRAME_SIZE; i++) {
        sum += frame[i];
    }

    int16_t mean = (int16_t)(sum / FRAME_SIZE);
    uint32_t energy_acc = 0;
    uint8_t sign = 0;
    uint8_t prev_sign = 0;
    uint16_t zc_count = 0;

    for (uint8_t i = 0; i < FRAME_SIZE; i++) {
        int16_t sample = (int16_t)frame[i];
        int16_t x = sample - mean;
        energy_acc += (uint32_t)((int32_t)x * x);
        // uart_puts()
        // uart_print_uint16(x)
        if (x > DEADZONE) {
            sign = 1;
        } else if (x < -DEADZONE) {
            sign = 2;
        } else {
            sign = 0;
        }

        if (sign != 0 && prev_sign != 0 && sign != prev_sign) {
            zc_count++;
        }
        if (sign != 0) {
            prev_sign = sign;
        }
    }

    *out_energy = (float)energy_acc / FRAME_SIZE;
    *out_zcr = (float)zc_count / (FRAME_SIZE - 1);
}

static void reset_capture_state(void) {
    for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
        online_energies[i] = 0.0f;
        online_zcrs[i] = 0.0f;
    }
    current_segment = 0;
    frames_in_segment = 0;
    post_vad_frames = 0;
    vad_triggered = false;
    recording_frame_count = 0;
}

static void process_frame(void) {
    float energy;
    float zcr;
    compute_frame_features((const uint8_t *)frame_buffer, &energy, &zcr);

    if (!recognition_requested) {
        return;
    }

    // Simple VAD logic: wait for energy to cross threshold
    if (!vad_triggered) {
        recording_frame_count++;
        if (energy >= noise_floor*VAD_THRESHOLD_MULTIPLIER) {  
            vad_triggered = true;
            post_vad_frames = 0;
            uart_puts("\r\n[VAD] Triggered at frame ");
            uart_print_uint16(recording_frame_count);
            uart_puts("\r\n");
        }
        // If we haven't triggered yet and collected too many silent frames, give up
        if (recording_frame_count >= TOTAL_FRAMES * 2) {
            capture_complete = true;
            recognition_requested = false;
        }
        return;
    }

    // After VAD triggered: accumulate into segments
    online_energies[current_segment] += energy;
    online_zcrs[current_segment] += zcr;
    frames_in_segment++;
    post_vad_frames++;

    // Move to next segment when we've filled current one
    if (frames_in_segment >= FRAMES_PER_SEGMENT && current_segment < NUM_SEGMENTS - 1) {
        current_segment++;
        frames_in_segment = 0;
    }

    // Stop when we've collected enough frames
    if (post_vad_frames >= TOTAL_FRAMES) {
        capture_complete = true;
        recognition_requested = false;
    }
}

static void build_feature_vector(float feature_vector[NUM_FEATURES], float noise_floor_value) {
    // Average the accumulated energy and ZCR
    float max_energy = 0.001f;
    
    for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
        float avg_energy = online_energies[i] / FRAMES_PER_SEGMENT;
        feature_vector[i + NUM_SEGMENTS] = online_zcrs[i] / FRAMES_PER_SEGMENT;
        
        // Clamp ZCR to [0, 1]
        if (feature_vector[i + NUM_SEGMENTS] < 0.0f) {
            feature_vector[i + NUM_SEGMENTS] = 0.0f;
        }
        if (feature_vector[i + NUM_SEGMENTS] > 1.0f) {
            feature_vector[i + NUM_SEGMENTS] = 1.0f;
        }
        
        // Subtract noise floor from energy
        float adj_energy = avg_energy - noise_floor_value;
        if (adj_energy < 0.0f) {
            adj_energy = 0.0f;
        }
        
        if (adj_energy > max_energy) {
            max_energy = adj_energy;
        }
        
        feature_vector[i] = adj_energy;
    }
    
    // Normalize energies by max
    if (max_energy < 1.0f) {
        max_energy = 100.0f;  // Fallback to avoid division issues
    }
    
    for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
        feature_vector[i] = feature_vector[i] / max_energy;
        if (feature_vector[i] > 1.0f) {
            feature_vector[i] = 1.0f;
        }
        if (feature_vector[i] < 0.0f) {
            feature_vector[i] = 0.0f;
        }
    }
}

static void print_feature_vector(const float feature_vector[NUM_FEATURES]) {
    uart_puts("Features: [");
    for (uint8_t i = 0; i < NUM_FEATURES; i++) {
        uart_print_float(feature_vector[i], 3);
        if (i < NUM_FEATURES - 1) {
            uart_puts(", ");
        }
    }
    uart_puts("]\r\n");
}

static float recognize_command(const float feature_vector[NUM_FEATURES], uint8_t *out_word_index) {
    float best_distance = 1e9f;
    uint8_t best_word = 0;

    for (uint8_t word = 0; word < NUM_WORDS; word++) {
        float distance = 0.0f;
        for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
            float template_value = pgm_read_float_near(&word_templates[word][i]);
            float diff = feature_vector[i] - template_value;
            distance += ENERGY_WEIGHT * diff * diff;
        }
        for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
            float template_value = pgm_read_float_near(&word_templates[word][NUM_SEGMENTS + i]);
            float diff = feature_vector[NUM_SEGMENTS + i] - template_value;
            distance += ZCR_WEIGHT * diff * diff;
        }

        uart_puts("Template ");
        uart_print_uint16(word + 1);
        uart_puts(" (");
        uart_puts(word_names[word]);
        uart_puts("): ");
        uart_print_float(distance, 3);
        uart_puts("\r\n");

        if (distance < best_distance) {
            best_distance = distance;
            best_word = word;
        }
    }

    *out_word_index = best_word;
    return best_distance;
}

static void print_result(uint8_t word_index, float distance, bool accepted) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    if (accepted) {
        lcd_clear();
        lcd_print("Recognized:");
        lcd_set_cursor(1, 0);
        lcd_print(word_names[word_index]);
        uart_puts("Prediction: ");
        uart_puts(word_names[word_index]);
        uart_puts(" (dist=");
        uart_print_float(distance, 3);
        uart_puts(")\r\n");
    } else {
        lcd_clear();
        lcd_print("Result:");
        lcd_set_cursor(1, 0);
        lcd_print("UNKNOWN");
        uart_puts("Prediction: UNKNOWN (dist=");
        uart_print_float(distance, 3);
        uart_puts(")\r\n");
    }
    _delay_ms(2500);

}

static void finalize_recognition(void) {
    float feature_vector[NUM_FEATURES];
     // Check if VAD was triggered
    if (!vad_triggered) {
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("No speech");
        lcd_set_cursor(1, 0);
        lcd_print("detected");
        uart_puts("\r\n[Result] No speech detected\r\n");
        capture_complete = false;
        return;
    }

    build_feature_vector(feature_vector, noise_floor);

    uart_puts("\r\n[Recognition] Complete\r\n");
    print_feature_vector(feature_vector);

    uint8_t best_word;
    float best_distance = recognize_command(feature_vector, &best_word);

    if (best_distance <= REJECT_DISTANCE_THRESHOLD) {
        print_result(best_word, best_distance, true);
    } else {
        print_result(best_word, best_distance, false);
    }
    capture_complete = false;
//    receive_input();
}

// ============================================================
// Calibration and button handling
// ============================================================
static bool button_pressed(uint8_t mask) {
    return !(PIND & mask);
}

static void wait_for_button_release(uint8_t mask) {
    _delay_ms(20);
    while (!(PIND & mask));
    _delay_ms(20);
}

static void calibrate_noise_floor(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Calibrating...");
    uart_puts("\r\n[Calibration] Started - Be silent!\r\n");

    float sum_energy = 0.0f;
    uint8_t count = 0;

    while (count < CALIBRATE_FRAMES) {
        if (frame_ready) {
            frame_ready = false;
            float energy, zcr;
            compute_frame_features((const uint8_t *)frame_buffer, &energy, &zcr);
            sum_energy += energy;
            count++;
            uart_puts(".");
        }
    }

    noise_floor = sum_energy / CALIBRATE_FRAMES;

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Noise floor:");
    lcd_set_cursor(1, 0);
    lcd_print_float(noise_floor, 1);
    uart_puts("\r\n[Calibration] Noise floor = ");
    uart_print_float(noise_floor, 1);
    uart_puts("\r\n");
    _delay_ms(1000);
    receive_input();
    lcd_clear();
    lcd_print("Calib done");
    _delay_ms(1000);
    lcd_clear();
    lcd_print("Ready");
}

static void start_recognition(void) {
    reset_capture_state();
    recognition_requested = true;
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Listening...");
    uart_puts("\r\n[Recognition] Started\r\n");
}

static void init_buttons(void) {
    DDRD &= ~((1 << BUTTON_START_PIN) | (1 << BUTTON_CALIBRATE_PIN));
    PORTD |= (1 << BUTTON_START_PIN) | (1 << BUTTON_CALIBRATE_PIN);
}

static void init_system(void) {
    lcd_init();
    uart_init();
    adc_init();
    timer0_init();
    init_buttons();
    sei();
    lcd_print("Resetting");
    _delay_ms(500);
    lcd_clear();
    lcd_print("Ready!");
    receive_input();
}

int main(void) {
    init_system();
    while (1) {
        // Process UART commands (non-blocking)
        uart_process_input();
        
        if (button_pressed(1 << BUTTON_CALIBRATE_PIN)) {
            _delay_ms(20);
            if (button_pressed(1 << BUTTON_CALIBRATE_PIN)) {
                wait_for_button_release(1 << BUTTON_CALIBRATE_PIN);
                calibrate_noise_floor();
            }
        }

        if (button_pressed(1 << BUTTON_START_PIN)) {
            _delay_ms(20);
            if (button_pressed(1 << BUTTON_START_PIN)) {
                wait_for_button_release(1 << BUTTON_START_PIN);
                start_recognition();
            }
        }

        if (frame_ready) {
            frame_ready = false;
            process_frame();
        }

        if (capture_complete) {
            finalize_recognition();
            lcd_set_cursor(0, 0);
            lcd_clear();
            lcd_print("Ready!");
            receive_input();
        }
    }
    return 0;
}
