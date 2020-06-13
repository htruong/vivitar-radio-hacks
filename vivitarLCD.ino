#include "LowPower.h"
#include <HID-Project.h>

volatile unsigned long _millis = 0;



static const byte SERIAL_CMD_FIXED_LENGTH = 7;
static const byte SERIAL_BYTE_START_MAGIC = 'C';
static const byte SERIAL_BYTE_ACK_OK = 'K';
static const byte SERIAL_BYTE_ACK_FAIL = 'F';
byte serial_cmd[SERIAL_CMD_FIXED_LENGTH];

enum serial_state_enum {
  BYTE_START_MAGIC,
  BYTE_CMD,
  BYTE_PARAM_1,
  BYTE_PARAM_2,
  BYTE_PARAM_3,
  BYTE_PARAM_4,
  BYTE_CHECKSUM
};


volatile byte serial_state = BYTE_START_MAGIC;

bool screen_spin_circle = false;
bool screen_blink_colon = false;
bool screen_blink_dot   = false;
bool screen_draw_clock  = false;

static const byte LED_PINS_COUNT = 7;
static const byte SCREEN_SEGMENTS = 42;
static const byte DIGIT_LED_SEGMENTS = 7;

static const byte PORTE_POS = 6;
static const byte PORTE_POS_MASK = 1 << 6;
static const byte SCREEN_SCANLINE_MS = 1;

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

/// Encoder and buttons
const int encoder_pin_a = 6;
const int encoder_pin_b = 5;
const int push_buttons_pin = A0;
const int mm_sample_delay = 100;
const int debouncer_ms = 5;
const int keyrepeat_ms = 500;

unsigned long last_mm_sample_time = 0;
unsigned long debounce_millis = 0;
unsigned long last_key_press = 0;

boolean encoder_a_l = LOW;
boolean encoder_b_l = LOW;


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


void setup_keyboard() {
  pinMode(encoder_pin_a, INPUT);
  pinMode(encoder_pin_b, INPUT);
  pinMode(push_buttons_pin, INPUT);
  digitalWrite(encoder_pin_a, HIGH);
  digitalWrite(encoder_pin_b, HIGH);
  digitalWrite(push_buttons_pin, HIGH);

  digitalWrite(A0, HIGH);
  ADCSRA =  bit (ADEN);                                // turn ADC on
  ADCSRA |= bit (ADPS2);  // Prescaler of 128
  ADMUX =   bit (REFS0) | (analogPinToChannel(0) & 0x07);     //Vcc reference, hardware analog channel. Use analogPinToChannel() to map Arduino analog input to correct hardware channel
  bitSet (ADCSRA, ADSC);  // start first conversion

  Consumer.begin();
}

void process_multimedia_keys()
{
  if ( _millis < (last_mm_sample_time + mm_sample_delay) ) {
    return;
  }
  
  // Multimedia buttons
  unsigned int pushBtnRead = ADC;
  if (pushBtnRead <= 264) {
    if (pushBtnRead > 200) {
      press_key_no_repeat(MEDIA_NEXT); // Next
    } else if (pushBtnRead > 150) {
      press_key_no_repeat(MEDIA_PREVIOUS); // Prev
    } else if (pushBtnRead > 115) {
      press_key_no_repeat(MEDIA_STOP); // Stop/M
    } else {
      press_key_no_repeat(MEDIA_PLAY_PAUSE); // Play pause
    }
  }
  last_mm_sample_time = _millis;
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
  draw_4digits(8888);
}

void refresh_screen() {
  draw_screen_fast();
}

void press_key(uint16_t k) {
  Serial.print(k);
    Consumer.write(k);
}

void press_key_no_repeat(uint16_t k) {
  if ( _millis > (last_key_press + keyrepeat_ms) ) {
    press_key(k);
    last_key_press = _millis;
  }
}


void process_keyboard_events() {
  if ( _millis < debounce_millis + debouncer_ms) {
    return;
  }
  
  boolean encoder_a = digitalRead(encoder_pin_a);
  boolean encoder_b = digitalRead(encoder_pin_b);

  if ((encoder_a == HIGH) && (encoder_b == HIGH)) {
    if ((encoder_a_l == LOW) && (encoder_b_l == HIGH)) {
      press_key(MEDIA_VOL_UP);      
    } else if ((encoder_b_l == LOW) && (encoder_a_l == HIGH)) {
      press_key(MEDIA_VOL_DOWN);
    }
  }
  
  encoder_a_l = encoder_a;
  encoder_b_l = encoder_b;

  /****************************************/
  debounce_millis = _millis;
  process_multimedia_keys();
}

void setup() {
  setup_screen();
  setup_keyboard();
  Serial.begin(115200);
  //Serial.write("Screen v2\n");
  pinMode(indicator_led, OUTPUT);
  //digitalWrite(indicator_led, LOW); // This LED is using a lot of power...
}

void loop() {
  LowPower.idle(SLEEP_15MS, ADC_ON, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
                  TIMER0_OFF, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
                  
  process_keyboard_events();
  process_screen_events();
  process_serial();
  _millis += 1;
}

void spin_circle() {
  if (screen_spin_circle) {
    byte circle_segment = _millis / 250 % 3;
    for (byte i = 0; i < 3; i++) {
      screen_buf[5 + i] = (circle_segment == i);
      screen_buf[8 + i] = (circle_segment == i);
    };
  }
}

void clear_spin_circle() {
  for (byte i = 0; i < 3; i++) {
    screen_buf[5 + i] = false;
    screen_buf[8 + i] = false;
  };  
}

void blink_segments() {
  if (screen_blink_colon) {
    screen_buf[26] = _millis / 500 % 2;
  }

  if (screen_blink_dot) {
    screen_buf[34] = _millis / 500 % 2;
  }
}


void process_screen_events() {
  if( _millis % 250 == 0 ) {
    if (screen_draw_clock) {
      byte secs = _millis / 1000 % 60;
      byte minutes = _millis / 1000 / 60 % 100;
      byte hours = _millis / 1000 / 60 / 60 % 24;
      draw_4digits(hours*100 + minutes);
    }
    blink_segments();
    spin_circle();
  }
  refresh_screen();
}

void consume_serial(byte b) {
  if (serial_state == BYTE_START_MAGIC) {
    if (b != SERIAL_BYTE_START_MAGIC) {
      Serial.write(SERIAL_BYTE_ACK_FAIL);
    } else {
      serial_state ++;
    }
  } else {
    serial_cmd[serial_state - 1] = b;
    serial_state = (serial_state + 1) % (SERIAL_CMD_FIXED_LENGTH + 1);
    if (serial_state == BYTE_START_MAGIC) {
      Serial.write(SERIAL_BYTE_ACK_OK);
      serial_process_command();
    }
  }
}

void serial_process_command() {
  //Serial.write("[Got command: ");
  //Serial.write(serial_cmd, SERIAL_CMD_FIXED_LENGTH);
  //Serial.write("\n]");

  // Commands
  // Draw clock:         C0|1
  // Set time:           THHMM (24 hours)
  // Set Precise time:   PBBBB (bytes)
  // Draw spinning disc: S0|1
  // Toggle screen bit:  BPP0|1 (PP is pixel number)

  if (serial_cmd[0] == 'C') {
    screen_draw_clock = (serial_cmd[1] == '1');
    screen_blink_colon = screen_draw_clock ;
  } else if (serial_cmd[0] == 'S') {
    screen_spin_circle = (serial_cmd[1] == '1');
    if (!screen_spin_circle) { 
      clear_spin_circle();
    }
  } else if (serial_cmd[0] == 'T' | serial_cmd[0] == 'P') {
    unsigned long offset_char = 0;
    if (serial_cmd[0] == 'T') {
      offset_char = '0';
    }
    unsigned long hh1 = serial_cmd[1]-offset_char;
    unsigned long hh2 = serial_cmd[2]-offset_char;
    unsigned long mm1 = serial_cmd[3]-offset_char;
    unsigned long mm2 = serial_cmd[4]-offset_char;

    if (serial_cmd[0] == 'T') {
      _millis = ((hh1*10 + hh2) * 60 * 60 + (mm1*10 + mm2) * 60) * 1000;
    } else {
      _millis = (hh1 << 24 | hh1 << 16 | mm1 << 8 | mm2);
    }
    
    debounce_millis = _millis;
    last_mm_sample_time = _millis;
  } else if (serial_cmd[0] == 'B') {
    unsigned long offset_char = '0';
    byte pp1 = serial_cmd[1]-offset_char;
    byte pp2 = serial_cmd[2]-offset_char;
    byte pos = pp1*10 + pp2;
    screen_buf[pos] = (serial_cmd[3] == '1');
  }
}

void process_serial() {
  while (Serial.available() > 0) {
    consume_serial(Serial.read());
  }
}
