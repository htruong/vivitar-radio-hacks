#include <avr/sleep.h>

static const byte LED_PINS_COUNT = 7;
static const byte SCREEN_SEGMENTS = 42;
static const byte DIGIT_LED_SEGMENTS = 7;

static const byte PORTE_POS = 6;
static const byte PORTE_POS_MASK = 1 << 6;

// Due to the way that the Arduino Pro micro is laid out
// We almost have every pin on the same PORT, but then we can't
// First pin has to be on port PE6
// Do not change this, if you change this,
// The draw_screen_fast() won't work correctly no more
byte ledPins[LED_PINS_COUNT] = {7, 15, 16, 14, 8, 9, 10};

byte indicator_led = 17;

volatile unsigned int current_scan_i = 0;
volatile unsigned int current_scan_offset = 0;
volatile bool screen_buf[SCREEN_SEGMENTS];
volatile unsigned long screen_last_refresh;
volatile unsigned int _cycle = 0;

byte screen_segment[LED_PINS_COUNT * LED_PINS_COUNT] ={
  00, 12, 13, 16,  3,  4,  8,
  17, 00, 19, 20, 23, 22, 11,
  18, 24, 00, 26, 28,  2,  6,
  14, 25, 32, 00, 29, 39,  7,
  15, 21, 33, 27, 00, 37, 41,
  30,  1, 31, 38, 40, 00, 36,
   9,  0,  5, 10, 34, 35, 00 
};

bool digit_segments[][DIGIT_LED_SEGMENTS] = {
  { 1,1,1,1,1,1,0 },
  { 0,1,1,0,0,0,0 },
  { 1,1,0,1,1,0,1 },
  { 1,1,1,1,0,0,1 },
  { 0,1,1,0,0,1,1 },
  { 1,0,1,1,0,1,1 },
  { 1,0,1,1,1,1,1 },
  { 1,1,1,0,0,0,0 },
  { 1,1,1,1,1,1,1 },
  { 1,1,1,1,0,1,1 },
  { 0,0,0,0,0,0,0 },  
};

byte lcd_digit_start_segment[] = { 12, 19, 27, 35 };

void draw_number(byte number, byte digit) {
  byte start_segment = lcd_digit_start_segment[digit];
  for (byte seg = 0; seg < DIGIT_LED_SEGMENTS; seg++) {
    screen_buf[seg + start_segment] = digit_segments[number][seg];
  }
}

void draw_screen_fast() {
  byte pinIo =    0; // Everything is INPUT
  byte pinVal =   0; // Everything is LOW
  byte currBit =  1;
  
  for (byte j = 0; j < LED_PINS_COUNT; j++) {
    if (current_scan_i == j) {
      // Turn the current scanline on
      // Set it as OUTPUT HIGH
      pinIo |= currBit;
      pinVal |= currBit;
    } else {
      if (screen_buf[ screen_segment[current_scan_offset + j] ]) {
        // Force Turn the segment on
        // Set it as OUTPUT LOW
        pinIo |= currBit;
      }
    }
    currBit = currBit << 1;
  }
  
  // Move the first LED pin to port PE6
  // Rest is PortB
  DDRB = (DDRB & 1) | (pinIo & B11111110);
  PORTB = (PORTB & 1) | (pinVal & B11111110);
  DDRB = pinIo;
  PORTB = pinVal;

  DDRE = (DDRE & ~PORTE_POS_MASK) | ((pinIo & 1) << PORTE_POS);
  PORTE = (PORTE & ~PORTE_POS_MASK) | ((pinVal & 1) << PORTE_POS);
  
  current_scan_i = (current_scan_i + 1) % LED_PINS_COUNT;
  current_scan_offset = current_scan_i * 7;
}

void draw_screen() {
  for (byte i = 0; i < LED_PINS_COUNT; i++) {
    pinMode(ledPins[i], INPUT);
  }
  
  for (byte j = 0; j < LED_PINS_COUNT; j++) {
    if (current_scan_i == j) {
      pinMode(ledPins[current_scan_i], OUTPUT);
      digitalWrite(ledPins[current_scan_i], HIGH);
    } else {
      if (screen_buf[ screen_segment[current_scan_offset + j] ]) {
        pinMode(ledPins[j], OUTPUT);
        digitalWrite(ledPins[j], LOW);
      }
    }
  }
  
  current_scan_i = (current_scan_i + 1) % LED_PINS_COUNT;
  current_scan_offset = current_scan_i * 7;
}

void draw_4digits(unsigned int num) {
  draw_number(num / 1000, 0);
  draw_number((num % 1000) / 100, 1);
  draw_number((num % 100) / 10, 2);
  draw_number(num % 10, 3);
}

void setup_screen() {
  // Empty the screen buffer
  for (byte i = 0; i < SCREEN_SEGMENTS; i++) {
    screen_buf[i] = false;
  }
}

void refresh_screen() {
  //if (millis() > screen_last_refresh) { 
    draw_screen_fast();
    screen_last_refresh = millis();
  //}
}

void setup() {
  setup_screen();
  delay(5000);
  pinMode(indicator_led, OUTPUT);
  digitalWrite(indicator_led, HIGH);
  
  cli();
  OCR0A = 255;
  TIMSK0 = 1<<OCIE0A;
  TCCR0A = (0<<WGM02) + (1<<WGM01) + (0<<WGM00);
  TCCR0B = (1<<CS02) + (0<<CS01) + (0<<CS00);
  set_sleep_mode( SLEEP_MODE_IDLE );
  sleep_enable();
  sei();
  sleep_cpu();
  
}

void loop() {
  sleep_cpu();
}

ISR( TIMER0_COMPA_vect ) {
  _cycle ++;
  draw_4digits(_cycle / 100 % 10000);
  screen_buf[34] = _cycle / 500 % 2;
  byte circle_segment = _cycle / 250 % 3;
  for (byte i = 0; i < 3; i++) {
    screen_buf[5 + i] = (circle_segment == i);
    screen_buf[8 + i] = (circle_segment == i);
  };
  refresh_screen();
}
