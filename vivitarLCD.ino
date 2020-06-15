#include "LowPower.h"
#include "HID-Project.h"

volatile unsigned long _millis = 0;


// Keyboard
const int kbd_encoder_pin_a = 6;
const int kbd_encoder_pin_b = 5;
const int kbd_push_buttons_pin = A0;
const int kbd_adc_sample_delay_ms = 100;
const int kbd_debouncer_ms = 5;
const int kbd_keyrepeat_ms = 500;

unsigned long kbd_last_mm_sample_time = 0;
unsigned long kbd_debounce_millis = 0;
unsigned long kbd_last_key_press = 0;

boolean kbd_encoder_a_l = LOW;
boolean kbd_encoder_b_l = LOW;


// Screen
bool scr_scr_spin_circle = false;
bool scr_blink_colon = false;
bool scr_blink_dot   = false;
bool scr_draw_clock  = false;

static const byte SCR_LED_PINS_COUNT = 7;
static const byte SCR_SEGMENTS = 42;
static const byte SCR_DIGIT_LED_SEGMENTS = 7;

static const byte SCR_PORTE_POS = 6;
static const byte SCR_PORTE_POS_MASK = 1 << 6;
static const byte SCR_SCANLINE_MS = 1;

// Due to the way that the Arduino Pro micro is laid out
// We almost have every pin on the same PORT, but then we can't
// First pin has to be on port PE6
// Do not change this, if you change this,
// The scr_draw_fast() won't work correctly no more
byte scr_led_pins[SCR_LED_PINS_COUNT] = {7, 15, 16, 14, 8, 9, 10};

volatile unsigned int scr_current_scan_i = 0;
volatile unsigned int scr_current_scan_offset = 0;
volatile bool scr_buf[SCR_SEGMENTS];

byte scr_segment[SCR_LED_PINS_COUNT * SCR_LED_PINS_COUNT] ={
  00, 12, 13, 16,  3,  4,  8,
  17, 00, 19, 20, 23, 22, 11,
  18, 24, 00, 26, 28,  2,  6,
  14, 25, 32, 00, 29, 39,  7,
  15, 21, 33, 27, 00, 37, 41,
  30,  1, 31, 38, 40, 00, 36,
  9,  0,  5, 10, 34, 35, 00 
};

bool scr_digit_segments[][SCR_DIGIT_LED_SEGMENTS] = {
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

byte scr_lcd_digit_start_segment[] = { 12, 19, 27, 35 };


// Serial command interface
static const byte SER_CMD_FIXED_LENGTH = 6;
static const byte SER_BYTE_START_MAGIC = 'C';
static const byte SER_BYTE_ACK_OK = 'K';
static const byte SER_BYTE_ACK_FAIL = 'F';
byte ser_cmd[SER_CMD_FIXED_LENGTH];

enum ser_state_enum {
  BYTE_START_MAGIC,
  BYTE_CMD,
  BYTE_PARAM_1,
  BYTE_PARAM_2,
  BYTE_PARAM_3,
  BYTE_PARAM_4
};

volatile byte ser_state = BYTE_START_MAGIC;

///////////////////////////////////////////
/* Keyboard handling functions */

void kbd_setup() {
  pinMode(kbd_encoder_pin_a, INPUT);
  pinMode(kbd_encoder_pin_b, INPUT);
  pinMode(kbd_push_buttons_pin, INPUT);
  digitalWrite(kbd_encoder_pin_a, HIGH);
  digitalWrite(kbd_encoder_pin_b, HIGH);
  digitalWrite(kbd_push_buttons_pin, HIGH);

  digitalWrite(A0, HIGH);
  ADCSRA =  bit (ADEN);
  ADCSRA |= bit (ADPS2);
  ADMUX =   bit (REFS0) | (analogPinToChannel(0) & 0x07);
  bitSet (ADCSRA, ADSC);

  Consumer.begin();
}

void kbd_handle_multimedia_keys()
{
  if ( _millis < (kbd_last_mm_sample_time + kbd_adc_sample_delay_ms) ) {
    return;
  }
  
  // Multimedia buttons
  unsigned int pushBtnRead = ADC;
  if (pushBtnRead <= 264) {
    if (pushBtnRead > 200) {
      kbd_press_key_no_repeat(MEDIA_NEXT); // Next
    } else if (pushBtnRead > 150) {
      kbd_press_key_no_repeat(MEDIA_PREVIOUS); // Prev
    } else if (pushBtnRead > 115) {
      kbd_press_key_no_repeat(MEDIA_STOP); // Stop/M
    } else {
      kbd_press_key_no_repeat(MEDIA_PLAY_PAUSE); // Play pause
    }
  }
  kbd_last_mm_sample_time = _millis;
} 

void kbd_handle_volume_knob() {
  if ( _millis < kbd_debounce_millis + kbd_debouncer_ms) {
    return;
  }

  boolean encoder_a = digitalRead(kbd_encoder_pin_a);
  boolean encoder_b = digitalRead(kbd_encoder_pin_b);

  if ((encoder_a == HIGH) && (encoder_b == HIGH)) {
    if ((kbd_encoder_a_l == LOW) && (kbd_encoder_b_l == HIGH)) {
      press_key(MEDIA_VOL_UP);      
    } else if ((kbd_encoder_b_l == LOW) && (kbd_encoder_a_l == HIGH)) {
      press_key(MEDIA_VOL_DOWN);
    }
  }
  
  kbd_encoder_a_l = encoder_a;
  kbd_encoder_b_l = encoder_b;

  kbd_debounce_millis = _millis;
}

void kbd_press_key_no_repeat(uint16_t k) {
  if ( _millis > (kbd_last_key_press + kbd_keyrepeat_ms) ) {
    kbd_press_key(k);
    kbd_last_key_press = _millis;
  }
}

void kbd_press_key(uint16_t k) {
  Serial.print(k);
  Consumer.write(k);
}


void kbd_events_process() {
  kbd_handle_volume_knob();
  kbd_handle_multimedia_keys();
}

///////////////////////////////////////////
/* LED/LCD Screen handling function */

void scr_draw_numer(byte number, byte digit) {
  if (number > 10 || digit > 3) {
    return;
  }
  byte start_segment = scr_lcd_digit_start_segment[digit];
  for (byte seg = 0; seg < SCR_DIGIT_LED_SEGMENTS; seg++) {
    scr_buf[seg + start_segment] = scr_digit_segments[number][seg];
  }
}

void scr_draw_fast() {
  byte pinIo =    0; // Everything is INPUT
  byte pinVal =   0; // Everything is LOW
  byte currBit =  1;
  
  for (byte j = 0; j < SCR_LED_PINS_COUNT; j++) {
    if (scr_current_scan_i == j) {
      // Turn the current scanline on
      // Set it as OUTPUT HIGH
      pinIo |= currBit;
      pinVal |= currBit;
    } else {
      if (scr_buf[ scr_segment[scr_current_scan_offset + j] ]) {
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

  DDRE = (DDRE & ~SCR_PORTE_POS_MASK) | ((pinIo & 1) << SCR_PORTE_POS);
  PORTE = (PORTE & ~SCR_PORTE_POS_MASK) | ((pinVal & 1) << SCR_PORTE_POS);
  
  scr_current_scan_i = (scr_current_scan_i + 1) % SCR_LED_PINS_COUNT;
  scr_current_scan_offset = scr_current_scan_i * 7;
}

void scr_draw() {
  for (byte i = 0; i < SCR_LED_PINS_COUNT; i++) {
    pinMode(scr_led_pins[i], INPUT);
  }
  
  for (byte j = 0; j < SCR_LED_PINS_COUNT; j++) {
    if (scr_current_scan_i == j) {
      pinMode(scr_led_pins[scr_current_scan_i], OUTPUT);
      digitalWrite(scr_led_pins[scr_current_scan_i], HIGH);
    } else {
      if (scr_buf[ scr_segment[scr_current_scan_offset + j] ]) {
        pinMode(scr_led_pins[j], OUTPUT);
        digitalWrite(scr_led_pins[j], LOW);
      }
    }
  }
  
  scr_current_scan_i = (scr_current_scan_i + 1) % SCR_LED_PINS_COUNT;
  scr_current_scan_offset = scr_current_scan_i * 7;
}

void scr_draw_4digits(unsigned int num) {
  scr_draw_numer(num / 1000, 0);
  scr_draw_numer((num % 1000) / 100, 1);
  scr_draw_numer((num % 100) / 10, 2);
  scr_draw_numer(num % 10, 3);
}

void scr_spin_circle() {
  if (scr_scr_spin_circle) {
    byte circle_segment = _millis / 250 % 3;
    for (byte i = 0; i < 3; i++) {
      scr_buf[5 + i] = (circle_segment == i);
      scr_buf[8 + i] = (circle_segment == i);
    };
  }
}

void scr_clear_spin_circle() {
  for (byte i = 0; i < 3; i++) {
    scr_buf[5 + i] = false;
    scr_buf[8 + i] = false;
  };  
}

void scr_blink_segments() {
  if (scr_blink_colon) {
    scr_buf[26] = _millis / 500 % 2;
  }

  if (scr_blink_dot) {
    scr_buf[34] = _millis / 500 % 2;
  }
}

void scr_events_process() {
  if( _millis % 50 == 0 ) {
    if (scr_draw_clock) {
      byte secs = _millis / 1000 % 60;
      byte minutes = _millis / 1000 / 60 % 60;
      byte hours = _millis / 1000 / 60 / 60 % 24;
      scr_draw_4digits(hours*100 + minutes);
    }
    scr_blink_segments();
    scr_spin_circle();
  }
  scr_draw_fast();
}

void scr_setup() {
  // Empty the screen buffer
  for (byte i = 0; i < SCR_SEGMENTS; i++) {
    scr_buf[i] = false;
  }
  scr_draw_4digits(8888);
}

///////////////////////////////////////////
/* Serial command interface handling function */

void ser_consume_char(byte b) {
  if (ser_state == BYTE_START_MAGIC) {
    if (b != SER_BYTE_START_MAGIC) {
      Serial.write(SER_BYTE_ACK_FAIL);
    } else {
      ser_state ++;
    }
  } else {
    ser_cmd[ser_state - 1] = b;
    ser_state = (ser_state + 1) % (SER_CMD_FIXED_LENGTH);
    if (ser_state == BYTE_START_MAGIC) {
      Serial.write(SER_BYTE_ACK_OK);
      ser_process_command();
    }
  }
}

void ser_process_command() {
  // Commands
  // Draw clock:         Cb (b=0|1)
  // Set time:           THHMM (24 hours)
  // Set Precise time:   PBBBB (bytes)
  // Display number:     DNNNN
  // Display digit:      GpN (p=0..3, N=0..9-)
  // Draw spinning disc: Sb (b=0|1)
  // Toggle screen bit:  BPPb (PP is pixel number, b=0|1)
  // Set raw display bit:Rlbbb (l=0|1 low or high, b=byte)
  
  // So send something like CC100\n will draw the clock

  byte cmd = ser_cmd[0];
  
  switch (cmd) {
    case 'C':
    ser_process_drawclock(cmd);
    break;
    
    case 'S':
    ser_process_scr_spin_circle(cmd);
    break;

    case 'G':
    ser_process_display_digit(cmd);
    break;

    case 'B':
    ser_process_display_bit(cmd);
    break;
    
    case 'R':
    ser_process_raw_buffer_data(cmd);
    break;
    
    case 'T':
    case 'P':
    case 'D':
    ser_process_time_or_number(cmd);
    break;
    
    default:
    break;
  }
}

void ser_process_raw_buffer_data(byte cmd) {
  byte offset = ser_cmd[1];
  for (byte i = 0; i < 3; i++) {
    unsigned long _byte = ser_cmd[2 + i];
    for (byte j = 0; j < 8; j++) {
      byte dest_offset = offset * 24 + i * 8 + j;
      if (dest_offset >= SCR_SEGMENTS) {
        return;
      }
      scr_buf[dest_offset] = _byte & 0x1;
      _byte = _byte >> 1;
    }
  }
}

void ser_process_time_or_number(byte cmd) {  
  byte offset_char = '0';
  if (cmd == 'P') {
    offset_char = 0;
  }
  
  unsigned long hh1 = ser_cmd[1]-offset_char;
  unsigned long hh2 = ser_cmd[2]-offset_char;
  unsigned long mm1 = ser_cmd[3]-offset_char;
  unsigned long mm2 = ser_cmd[4]-offset_char;

  if (cmd == 'D') {
    scr_draw_clock = false;
    scr_blink_colon = false;
    scr_draw_4digits(hh1 * 1000 + hh2 * 100 + mm1 * 10 + mm2);
  } else {
    if (cmd == 'T') {
      _millis = ((hh1*10 + hh2) * 60 * 60 + (mm1*10 + mm2) * 60) * 1000;
    } else {
      _millis = (hh1 << 24 | hh1 << 16 | mm1 << 8 | mm2);
    }

    kbd_debounce_millis = _millis;
    kbd_last_mm_sample_time = _millis;
  }
}

void ser_process_display_bit(byte cmd) {
  byte offset_char = '0';
  byte pp1 = ser_cmd[1]-offset_char;
  byte pp2 = ser_cmd[2]-offset_char;
  
  byte pos = pp1*10 + pp2;
  scr_buf[pos] = (ser_cmd[3] == '1');
}

void ser_process_drawclock(byte cmd) {
  scr_draw_clock = (ser_cmd[1] == '1');
  scr_blink_colon = scr_draw_clock;
  scr_buf[34] = false;
}

void ser_process_scr_spin_circle(byte cmd) {
  scr_scr_spin_circle = (ser_cmd[1] == '1');
  if (!scr_scr_spin_circle) { 
    scr_clear_spin_circle();
  }
}

void ser_process_display_digit(byte cmd) {
  byte offset_char = '0';
  byte pos = ser_cmd[1] - offset_char;
  if (ser_cmd[2] == '-') {
    scr_draw_numer(10, pos);
  } else {
    scr_draw_numer(ser_cmd[2] - offset_char, pos);
  }
}

void ser_events_process() {
  while (Serial.available() > 0) {
    byte c = Serial.read();
    if (c != '\n') {
      ser_consume_char(c);
    }
  }
}

void ser_setup() {
  Serial.begin(115200);
}


///////////////////////////////////////////

void setup() {
  kbd_setup();
  scr_setup();
  ser_setup();
}

void loop() {
  // I have checked and this LowPower actually makes the uC sleep for exactly 1ms
  // Somehow if you have the USB data lines connected, then this SLEEP_15MS becomes SLEEP_1MS
  // I don't know how but I gave up finding out.
  
  LowPower.idle(SLEEP_15MS, ADC_ON, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
    TIMER0_OFF, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
  
  kbd_events_process();
  scr_events_process();
  ser_events_process();
  _millis += 1;
}
