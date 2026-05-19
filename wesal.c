#define F_CPU 11059200UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "my_lcd.h"

// ============================================================
// SYSTEM SETTINGS
// ============================================================
#define UART_BAUD        9600UL
#define FRAME_SIZE       128
#define NUM_SEGMENTS     5
#define FRAMES_PER_SEG   8
#define TOTAL_FRAMES     (NUM_SEGMENTS * FRAMES_PER_SEG)

#define BUTTON_PIN       PD2
#define NOISE_BUTTON_PIN PD7
#define LED_PIN          PB0

#define NUM_WORDS        8
#define NUM_FEATURES     10

#define ZCR_WEIGHT       6.0f
#define ACCEPT_THRESHOLD 1.0f           // رفع العتبة إلى 1.0
#define MARGIN_THRESHOLD 0.1f           // هامش صغير (بدلاً من 0.2)

// ============================================================
// WORD NAMES AND TEMPLATES (من التدريب)
// ============================================================
const char* const word_names[NUM_WORDS] = {
    "Freeze", "Grab", "Jump", "Lift", "On", "Push", "Right", "Start"
};

const float word_templates[NUM_WORDS][NUM_FEATURES] = {
    {0.1142, 0.8014, 0.7066, 0.1607, 0.0959, 0.4382, 0.2133, 0.1188, 0.1565, 0.3757}, // FREEZE
    {0.2793, 0.9813, 0.5217, 0.1186, 0.1406, 0.1653, 0.1852, 0.1654, 0.0913, 0.1393}, // GRAB
    {0.3946, 0.7827, 0.5821, 0.0490, 0.4217, 0.1581, 0.2550, 0.1452, 0.1132, 0.1647}, // JUMP
    {0.6373, 0.7582, 0.0014, 0.0000, 0.0387, 0.1830, 0.2472, 0.3456, 0.2638, 0.3857}, // LIFT
    {0.8967, 0.5082, 0.0841, 0.0053, 0.1004, 0.2344, 0.1597, 0.1162, 0.1583, 0.1860}, // ON
    {1.0000, 0.2609, 0.0503, 0.0008, 0.0000, 0.2013, 0.4674, 0.6543, 0.4149, 0.1822}, // PUSH
    {0.5635, 0.8921, 0.0098, 0.0173, 0.0333, 0.2109, 0.2288, 0.1413, 0.2688, 0.3732}, // RIGHT
    {0.1709, 0.1269, 0.7267, 0.1356, 0.2365, 0.4190, 0.3277, 0.2614, 0.2074, 0.2544}, // START
};

// ============================================================
// GLOBALS
// ============================================================
float NOISE_FLOOR = 94.43f;

volatile uint16_t adc_sample = 0;
volatile uint8_t frame_buffer[FRAME_SIZE];
volatile uint16_t frame_index = 0;
volatile uint8_t frame_done = 0;
volatile uint8_t capture_enable = 0;

volatile uint8_t recording_active = 0;
volatile uint16_t recording_frame_count = 0;
volatile uint8_t recording_complete = 0;
volatile uint8_t vad_triggered = 0;

float online_energies[NUM_SEGMENTS] = {0};
float online_zcrs[NUM_SEGMENTS] = {0};
uint8_t current_segment = 0;
uint16_t frames_in_segment = 0;
volatile uint16_t post_vad_frames = 0;

static int total_tests = 0;
static int correct_tests = 0;

// ============================================================
// UART FUNCTIONS
// ============================================================
void UART_Init(void) {
    uint16_t ubrr = (F_CPU / (16UL * UART_BAUD)) - 1;
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void UART_SendChar(char c) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = c;
}

void UART_SendString(const char *s) {
    while (*s) UART_SendChar(*s++);
}

void UART_SendInt(int value) {
    char buf[10];
    itoa(value, buf, 10);
    UART_SendString(buf);
}

void UART_SendFloat(float x) {
    int scaled = (int)(x * 100);
    if (scaled < 0) {
        UART_SendChar('-');
        scaled = -scaled;
    }
    UART_SendInt(scaled / 100);
    UART_SendChar('.');
    UART_SendInt((scaled % 100) / 10);
    UART_SendInt(scaled % 10);
}

// ============================================================
// HARDWARE INIT
// ============================================================
void ADC_Init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void Timer0_Init(void) {
    TCCR0 = (1 << WGM01) | (1 << CS01);
    OCR0 = 172;
    TIMSK |= (1 << OCIE0);
}

// ============================================================
// FRAME FEATURE CALCULATION
// ============================================================
void compute_frame_features(uint8_t *buf, float *energy, float *zcr) {
    int32_t sum = 0;
    uint32_t e_acc = 0;
    uint16_t zc_count = 0;
    
    for (int i = 0; i < FRAME_SIZE; i++) {
        sum += buf[i];
    }
    int16_t mean = (int16_t)(sum / FRAME_SIZE);
    
    for (int i = 0; i < FRAME_SIZE; i++) {
        int16_t x = (int16_t)buf[i] - mean;
        e_acc += (uint32_t)((int32_t)x * x);
        
        if (i > 0) {
            int16_t prev_x = (int16_t)buf[i-1] - mean;
            if ((x > 0 && prev_x < 0) || (x < 0 && prev_x > 0)) {
                zc_count++;
            }
        }
    }
    *energy = (float)e_acc / (float)FRAME_SIZE;
    *zcr = (float)zc_count / (float)(FRAME_SIZE - 1);
}

// ============================================================
// INTERRUPTS - مع VAD
// ============================================================
ISR(TIMER0_COMP_vect) { 
    ADCSRA |= (1 << ADSC); 
}

ISR(ADC_vect) {
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    adc_sample = (high << 8) | low;
    
    if (capture_enable || recording_active) {
        frame_buffer[frame_index++] = (uint8_t)(adc_sample >> 2);
        
        if (frame_index >= FRAME_SIZE) {
            frame_index = 0;
            
            if (capture_enable) {
                frame_done = 1;
                capture_enable = 0;
            }
            
            if (recording_active) {
                float e, z;
                compute_frame_features((uint8_t*)frame_buffer, &e, &z);
                
                if (!vad_triggered) {
                    recording_frame_count++;
                    if (e >= 100.0f) {  // عتبة VAD ثابتة
                        vad_triggered = 1;
                        post_vad_frames = 0;
                        UART_SendString("VAD+\r\n");
                    }
                    if (recording_frame_count >= TOTAL_FRAMES * 2) {
                        recording_active = 0;
                        recording_complete = 1;
                    }
                    return;
                }
                
                online_energies[current_segment] += e;
                online_zcrs[current_segment] += z;
                frames_in_segment++;
                post_vad_frames++;
                
                if (frames_in_segment >= FRAMES_PER_SEG && current_segment < NUM_SEGMENTS - 1) {
                    current_segment++;
                    frames_in_segment = 0;
                }
                
                if (post_vad_frames >= TOTAL_FRAMES) {
                    recording_active = 0;
                    recording_complete = 1;
                }
            }
        }
    }
}

// ============================================================
// CAPTURE FUNCTIONS
// ============================================================
void capture_one_frame(void) {
    frame_done = 0;
    frame_index = 0;
    capture_enable = 1;
    while (!frame_done);
}

void start_recording(void) {
    for(int i = 0; i < NUM_SEGMENTS; i++) {
        online_energies[i] = 0;
        online_zcrs[i] = 0;
    }
    current_segment = 0;
    frames_in_segment = 0;
    recording_frame_count = 0;
    post_vad_frames = 0;
    vad_triggered = 0;
    recording_active = 1;
    recording_complete = 0;
    frame_index = 0;
    
    _delay_ms(80);
    
    while (!recording_complete);
}

// ============================================================
// DYNAMIC NOISE MEASUREMENT
// ============================================================
float measure_dynamic_noise(void) {
    float total = 0;
    float e, z;
    int samples = 0;
    
    for(int i = 0; i < 8; i++) {
        capture_one_frame();
        compute_frame_features((uint8_t*)frame_buffer, &e, &z);
        
        if(e > 10 && e < 300) {
            total += e;
            samples++;
        }
        _delay_ms(30);
    }
    
    return (samples > 2) ? (total / samples) : NOISE_FLOOR;
}

// ============================================================
// FEATURE EXTRACTION
// ============================================================
void extract_features(float out[NUM_FEATURES], float dynamic_noise) {
    float raw_e[NUM_SEGMENTS];
    float max_e = 0.001f;
    
    for(int i = 0; i < NUM_SEGMENTS; i++) {
        raw_e[i] = online_energies[i] / FRAMES_PER_SEG;
        out[i+5] = online_zcrs[i] / FRAMES_PER_SEG;
        if(out[i+5] < 0) out[i+5] = 0;
        if(out[i+5] > 1) out[i+5] = 1;
        if(raw_e[i] > max_e) max_e = raw_e[i];
    }
    
    if(max_e < 1) max_e = 100;
    
    float noise_ratio = dynamic_noise / max_e;
    if(noise_ratio > 0.4f) noise_ratio = 0.4f;
    if(noise_ratio < 0.1f) noise_ratio = 0.1f;
    
    for(int i = 0; i < NUM_SEGMENTS; i++) {
        float n = raw_e[i] / max_e - noise_ratio;
        if(n < 0) n = 0;
        if(n > 1) n = 1;
        out[i] = n;
    }
}

// ============================================================
// DISTANCE CALCULATION
// ============================================================
float calculate_distance(float *f1, float *f2) {
    float d = 0;
    for(int i = 0; i < 5; i++) {
        float df = f1[i] - f2[i];
        d += df * df;
    }
    for(int i = 5; i < 10; i++) {
        float df = f1[i] - f2[i];
        d += df * df * ZCR_WEIGHT;
    }
    return d;
}

// ============================================================
// RECOGNITION - مبسط بدون margin
// ============================================================
int recognize(float *features) {
    int best = -1;
    float min_dist = 999.0f;
    
    UART_SendString("Dist: ");
    for(int w = 0; w < NUM_WORDS; w++) {
        float dist = calculate_distance(features, (float*)word_templates[w]);
        UART_SendString(word_names[w]);
        UART_SendChar('=');
        UART_SendFloat(dist);
        UART_SendChar(' ');
        
        if(dist < min_dist) {
            min_dist = dist;
            best = w;
        }
    }
    UART_SendString("\r\n");
    
    // قبول بسيط: إذا كانت المسافة أقل من العتبة
    if(min_dist < ACCEPT_THRESHOLD) {
        UART_SendString("=> ");
        UART_SendString(word_names[best]);
        UART_SendString("\r\n");
        return best;
    } else {
        UART_SendString("=> REJECTED\r\n");
        return -1;
    }
}

// ============================================================
// RECORD AND TEST
// ============================================================
int record_and_test(void) {
    float features[NUM_FEATURES];
    float dyn_noise;
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Press PD2");
    LCD_String_xy(1, 0, "to test");
    UART_SendString("\r\nPress PD2...\r\n");
    
    while(PIND & (1 << BUTTON_PIN));
    _delay_ms(50);
    
    dyn_noise = measure_dynamic_noise();
    UART_SendString("[N]");
    UART_SendFloat(dyn_noise);
    UART_SendString("\r\n");
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Listening...");
    LCD_String_xy(1, 0, "Speak now!");
    
    PORTB |= (1 << LED_PIN);
    start_recording();
    PORTB &= ~(1 << LED_PIN);
    
    if (!vad_triggered) {
        UART_SendString("No speech\r\n");
        return -2;
    }
    
    extract_features(features, dyn_noise);
    
    UART_SendString("E:");
    for(int i = 0; i < 5; i++) {
        UART_SendFloat(features[i]);
        if(i<4) UART_SendString(",");
    }
    UART_SendString(" Z:");
    for(int i = 5; i < 10; i++) {
        UART_SendFloat(features[i]);
        if(i<9) UART_SendString(",");
    }
    UART_SendString("\r\n");
    
    return recognize(features);
}

// ============================================================
// GLOBAL NOISE CALIBRATION
// ============================================================
void calibrate_noise(void) {
    float total = 0;
    float e, z;
    int samples = 0;
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Noise Calib");
    LCD_String_xy(1, 0, "BE SILENT!");
    UART_SendString("\r\n=== NOISE CALIBRATION ===\r\n");
    
    for(int i = 0; i < 20; i++) {
        capture_one_frame();
        compute_frame_features((uint8_t*)frame_buffer, &e, &z);
        
        if(e > 10 && e < 300) {
            total += e;
            samples++;
            UART_SendString(".");
        }
        _delay_ms(80);
    }
    UART_SendString("\r\n");
    
    NOISE_FLOOR = (samples > 5) ? (total / samples) : 80.0f;
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Noise Floor:");
    char buf[15];
    sprintf(buf, "%.0f", NOISE_FLOOR);
    LCD_String_xy(1, 0, buf);
    UART_SendString(">>> Noise: ");
    UART_SendFloat(NOISE_FLOOR);
    UART_SendString("\r\n");
    _delay_ms(1500);
}

// ============================================================
// MAIN
// ============================================================
int main(void) {
    UART_Init();
    ADC_Init();
    Timer0_Init();
    LCD_Init();
    
    DDRB |= (1 << LED_PIN);
    PORTB &= ~(1 << LED_PIN);
    
    DDRD &= ~((1 << BUTTON_PIN) | (1 << NOISE_BUTTON_PIN));
    PORTD |= (1 << BUTTON_PIN) | (1 << NOISE_BUTTON_PIN);
    
    sei();
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Tester v2.1");
    LCD_String_xy(1, 0, "Simple Mode");
    UART_SendString("\r\n========================================\r\n");
    UART_SendString("TESTER v2.1 - SIMPLE RECOGNITION\r\n");
    UART_SendString("========================================\r\n");
    _delay_ms(1500);
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Press PD7");
    LCD_String_xy(1, 0, "Calibrate");
    UART_SendString("\r\nPress PD7 to calibrate...\r\n");
    
    while(PIND & (1 << NOISE_BUTTON_PIN));
    _delay_ms(50);
    calibrate_noise();
    
    LCD_Clear();
    LCD_String_xy(0, 0, "Ready!");
    LCD_String_xy(1, 0, "Press PD2");
    UART_SendString("\r\nReady! Say: Go/Stop/Left/Right/Up/Down/Yes/No\r\n");
    _delay_ms(1500);
    
    while(1) {
        UART_SendString("\r\n--- Test ---\r\n");
        
        int result = record_and_test();
        
        if(result >= 0) {
            total_tests++;
            correct_tests++;
            
            LCD_Clear();
            LCD_String_xy(0, 0, "RECOGNIZED:");
            LCD_String_xy(1, 0, (char*)word_names[result]);
            UART_SendString("✓ ");
            UART_SendString(word_names[result]);
            UART_SendString("\r\n");
            
            PORTB |= (1 << LED_PIN);
            _delay_ms(200);
            PORTB &= ~(1 << LED_PIN);
            
        } else if(result == -2) {
            UART_SendString("No speech, try again\r\n");
            continue;
        } else {
            total_tests++;
            LCD_Clear();
            LCD_String_xy(0, 0, "UNKNOWN");
            LCD_String_xy(1, 0, "Try again");
            UART_SendString("✗ UNKNOWN\r\n");
        }
        
        char stat[20];
        sprintf(stat, "Acc: %d/%d", correct_tests, total_tests);
        LCD_String_xy(1, 0, stat);
        UART_SendString("Stats: ");
        UART_SendInt(correct_tests);
        UART_SendString("/");
        UART_SendInt(total_tests);
        UART_SendString("\r\n");
        
        _delay_ms(2000);
    }
}