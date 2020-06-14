#include "LowPower.h"
#include "HID-Project.h"

volatile unsigned long _millis = 0;

static const byte SER_CMD_FIXED_LENGTH = 6;
static const byte SER_BYTE_START_MAGIC = 'C';
static const byte SER_BYTE_ACK_OK = 'K';
static const byte SER_BYTE_ACK_FAIL = 'F';
byte SER_cmd[SER_CMD_FIXED_LENGTH];

enum SER_state_enum {
  BYTE_START_MAGIC,
  BYTE_CMD,
  BYTE_PARAM_1,
  BYTE_PARAM_2,
  BYTE_PARAM_3,
  BYTE_PARAM_4
};

volatile byte SER_state = BYTE_START_MAGIC;

bool SCR_spin_circle = false;
bool SCR_blink_colon = false;
bool SCR_blink_dot   = false;
bool SCR_draw_clock  = false;

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
// The SCR_draw_fast() won't work correctly no more
byte SCR_led_pins[SCR_LED_PINS_COUNT] = {7, 15, 16, 14, 8, 9, 10};

volatile unsigned int SCR_current_scan_i = 0;
volatile unsigned int SCR_current_scan_offset = 0;
volatile bool SCR_buf[SCR_SEGMENTS];

byte SCR_segment[SCR_LED_PINS_COUNT * SCR_LED_PINS_COUNT] ={
  00, 12, 13, 16,  3,  4,  8,
  17, 00, 19, 20, 23, 22, 11,
  18, 24, 00, 26, 28,  2,  6,
  14, 25, 32, 00, 29, 39,  7,
  15, 21, 33, 27, 00, 37, 41,
  30,  1, 31, 38, 40, 00, 36,
   9,  0,  5, 10, 34, 35, 00 
};

bool SCR_digit_segments[][SCR_DIGIT_LED_SEGMENTS] = {
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

byte SCR_lcd_digit_start_segment[] = { 12, 19, 27, 35 };

/// Encoder and buttons
const int KBD_encoder_pin_a = 6;
const int KBD_encoder_pin_b = 5;
const int KBD_push_buttons_pin = A0;
const int KBD_adc_sample_delay_ms = 100;
const int KBD_debouncer_ms = 5;
const int KBD_keyrepeat_ms = 500;

unsigned long KBD_last_mm_sample_time = 0;
unsigned long KBD_debounce_millis = 0;
unsigned long KBD_last_key_press = 0;

boolean KBD_encoder_a_l = LOW;
boolean KBD_encoder_b_l = LOW;


void SCR_draw_numer(byte number, byte digit) {
  if (number > 10 || digit > 3) {
    return;
  }
  byte start_segment = SCR_lcd_digit_start_segment[digit];
  for (byte seg = 0; seg < SCR_DIGIT_LED_SEGMENTS; seg++) {
    SCR_buf[seg + start_segment] = SCR_digit_segments[number][seg];
  }
}

void SCR_draw_fast() {
  byte pinIo =    0; // Everything is INPUT
  byte pinVal =   0; // Everything is LOW
  byte currBit =  1;
  
  for (byte j = 0; j < SCR_LED_PINS_COUNT; j++) {
    if (SCR_current_scan_i == j) {
      // Turn the current scanline on
      // Set it as OUTPUT HIGH
      pinIo |= currBit;
      pinVal |= currBit;
    } else {
      if (SCR_buf[ SCR_segment[SCR_current_scan_offset + j] ]) {
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
  
  SCR_current_scan_i = (SCR_current_scan_i + 1) % SCR_LED_PINS_COUNT;
  SCR_current_scan_offset = SCR_current_scan_i * 7;
}

void SCR_draw() {
  for (byte i = 0; i < SCR_LED_PINS_COUNT; i++) {
    pinMode(SCR_led_pins[i], INPUT);
  }
  
  for (byte j = 0; j < SCR_LED_PINS_COUNT; j++) {
    if (SCR_current_scan_i == j) {
      pinMode(SCR_led_pins[SCR_current_scan_i], OUTPUT);
      digitalWrite(SCR_led_pins[SCR_current_scan_i], HIGH);
    } else {
      if (SCR_buf[ SCR_segment[SCR_current_scan_offset + j] ]) {
        pinMode(SCR_led_pins[j], OUTPUT);
        digitalWrite(SCR_led_pins[j], LOW);
      }
    }
  }
  
  SCR_current_scan_i = (SCR_current_scan_i + 1) % SCR_LED_PINS_COUNT;
  SCR_current_scan_offset = SCR_current_scan_i * 7;
}


void KBD_setup() {
  pinMode(KBD_encoder_pin_a, INPUT);
  pinMode(KBD_encoder_pin_b, INPUT);
  pinMode(KBD_push_buttons_pin, INPUT);
  digitalWrite(KBD_encoder_pin_a, HIGH);
  digitalWrite(KBD_encoder_pin_b, HIGH);
  digitalWrite(KBD_push_buttons_pin, HIGH);

  digitalWrite(A0, HIGH);
  ADCSRA =  bit (ADEN);                                // turn ADC on
  ADCSRA |= bit (ADPS2);  // Prescaler of 128
  ADMUX =   bit (REFS0) | (analogPinToChannel(0) & 0x07);     //Vcc reference, hardware analog channel. Use analogPinToChannel() to map Arduino analog input to correct hardware channel
  bitSet (ADCSRA, ADSC);  // start first conversion

  Consumer.begin();
}

void KBD_handle_multimedia_keys()
{
  if ( _millis < (KBD_last_mm_sample_time + KBD_adc_sample_delay_ms) ) {
    return;
  }
  
  // Multimedia buttons
  unsigned int pushBtnRead = ADC;
  if (pushBtnRead <= 264) {
    if (pushBtnRead > 200) {
      KBD_press_key_no_repeat(MEDIA_NEXT); // Next
    } else if (pushBtnRead > 150) {
      KBD_press_key_no_repeat(MEDIA_PREVIOUS); // Prev
    } else if (pushBtnRead > 115) {
      KBD_press_key_no_repeat(MEDIA_STOP); // Stop/M
    } else {
      KBD_press_key_no_repeat(MEDIA_PLAY_PAUSE); // Play pause
    }
  }
  KBD_last_mm_sample_time = _millis;
}

void SCR_draw_4digits(unsigned int num) {
  SCR_draw_numer(num / 1000, 0);
  SCR_draw_numer((num % 1000) / 100, 1);
  SCR_draw_numer((num % 100) / 10, 2);
  SCR_draw_numer(num % 10, 3);
}

void SCR_setup() {
  // Empty the screen buffer
  for (byte i = 0; i < SCR_SEGMENTS; i++) {
    SCR_buf[i] = false;
  }
  SCR_draw_4digits(8888);
}

void KBD_press_key(uint16_t k) {
  Serial.print(k);
    Consumer.write(k);
}

void KBD_press_key_no_repeat(uint16_t k) {
  if ( _millis > (KBD_last_key_press + KBD_keyrepeat_ms) ) {
    KBD_press_key(k);
    KBD_last_key_press = _millis;
  }
}

void KBD_handle_volume_knob() {
  if ( _millis < KBD_debounce_millis + KBD_debouncer_ms) {
    return;
  }

  boolean encoder_a = digitalRead(KBD_encoder_pin_a);
  boolean encoder_b = digitalRead(KBD_encoder_pin_b);

  if ((encoder_a == HIGH) && (encoder_b == HIGH)) {
    if ((KBD_encoder_a_l == LOW) && (KBD_encoder_b_l == HIGH)) {
      press_key(MEDIA_VOL_UP);      
    } else if ((KBD_encoder_b_l == LOW) && (KBD_encoder_a_l == HIGH)) {
      press_key(MEDIA_VOL_DOWN);
    }
  }
  
  KBD_encoder_a_l = encoder_a;
  KBD_encoder_b_l = encoder_b;

  KBD_debounce_millis = _millis;
}


void KBD_events_process() {
  KBD_handle_volume_knob();
  KBD_handle_multimedia_keys();
}

void spin_circle() {
  if (SCR_spin_circle) {
    byte circle_segment = _millis / 250 % 3;
    for (byte i = 0; i < 3; i++) {
      SCR_buf[5 + i] = (circle_segment == i);
      SCR_buf[8 + i] = (circle_segment == i);
    };
  }
}

void SCR_clear_spin_circle() {
  for (byte i = 0; i < 3; i++) {
    SCR_buf[5 + i] = false;
    SCR_buf[8 + i] = false;
  };  
}

void SCR_blink_segments() {
  if (SCR_blink_colon) {
    SCR_buf[26] = _millis / 500 % 2;
  }

  if (SCR_blink_dot) {
    SCR_buf[34] = _millis / 500 % 2;
  }
}


void SCR_events_process() {
  if( _millis % 50 == 0 ) {
    if (SCR_draw_clock) {
      byte secs = _millis / 1000 % 60;
      byte minutes = _millis / 1000 / 60 % 60;
      byte hours = _millis / 1000 / 60 / 60 % 24;
      SCR_draw_4digits(hours*100 + minutes);
    }
    SCR_blink_segments();
    spin_circle();
  }
  SCR_draw_fast();
}

void SER_consume_char(byte b) {
  if (SER_state == BYTE_START_MAGIC) {
    if (b != SER_BYTE_START_MAGIC) {
      Serial.write(SER_BYTE_ACK_FAIL);
    } else {
      SER_state ++;
    }
  } else {
    SER_cmd[SER_state - 1] = b;
    SER_state = (SER_state + 1) % (SER_CMD_FIXED_LENGTH);
    if (SER_state == BYTE_START_MAGIC) {
      Serial.write(SER_BYTE_ACK_OK);
      SER_process_command();
    }
  }
}

void SER_process_command() {
  //Serial.write("[Got command: ");
  //Serial.write(SER_cmd, SER_CMD_FIXED_LENGTH);
  //Serial.write("\n]");

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

  byte cmd = SER_cmd[0];
  
  switch (cmd) {
    case 'C':
      SER_process_drawclock(cmd);
      break;
      
    case 'S':
      SER_process_spin_circle(cmd);
      break;

    case 'G':
      SER_process_display_digit(cmd);
      break;

    case 'B':
      SER_process_display_bit(cmd);
      break;
      
    case 'R':
      SER_process_raw_buffer_data(cmd);
      break;
      
    case 'T':
    case 'P':
    case 'D':
      SER_process_time_or_number(cmd);
      break;
      
    default:
      break;
  }
}

void SER_process_raw_buffer_data(byte cmd) {
  byte offset = SER_cmd[1];
  for (byte i = 0; i < 3; i++) {
    unsigned long _byte = SER_cmd[2 + i];
    for (byte j = 0; j < 8; j++) {
      byte dest_offset = offset * 24 + i * 8 + j;
      if (dest_offset >= SCR_SEGMENTS) {
        return;
      }
      SCR_buf[dest_offset] = _byte & 0x1;
      _byte = _byte >> 1;
    }
  }
}

void SER_process_time_or_number(byte cmd) {  
  byte offset_char = '0';
  if (cmd == 'P') {
    offset_char = 0;
  }
  
  unsigned long hh1 = SER_cmd[1]-offset_char;
  unsigned long hh2 = SER_cmd[2]-offset_char;
  unsigned long mm1 = SER_cmd[3]-offset_char;
  unsigned long mm2 = SER_cmd[4]-offset_char;

  if (cmd == 'D') {
    SCR_draw_clock = false;
    SCR_blink_colon = false;
    SCR_draw_4digits(hh1 * 1000 + hh2 * 100 + mm1 * 10 + mm2);
  } else {
    if (cmd == 'T') {
      _millis = ((hh1*10 + hh2) * 60 * 60 + (mm1*10 + mm2) * 60) * 1000;
    } else {
      _millis = (hh1 << 24 | hh1 << 16 | mm1 << 8 | mm2);
    }

    KBD_debounce_millis = _millis;
    KBD_last_mm_sample_time = _millis;
  }
}

void SER_process_display_bit(byte cmd) {
  byte offset_char = '0';
  byte pp1 = SER_cmd[1]-offset_char;
  byte pp2 = SER_cmd[2]-offset_char;
  
  byte pos = pp1*10 + pp2;
  SCR_buf[pos] = (SER_cmd[3] == '1');
}

void SER_process_drawclock(byte cmd) {
  SCR_draw_clock = (SER_cmd[1] == '1');
  SCR_blink_colon = SCR_draw_clock;
  SCR_buf[34] = false;
}

void SER_process_spin_circle(byte cmd) {
  SCR_spin_circle = (SER_cmd[1] == '1');
  if (!SCR_spin_circle) { 
    SCR_clear_spin_circle();
  }
}

void SER_process_display_digit(byte cmd) {
  byte offset_char = '0';
  byte pos = SER_cmd[1] - offset_char;
  if (SER_cmd[2] == '-') {
    SCR_draw_numer(10, pos);
  } else {
    SCR_draw_numer(SER_cmd[2] - offset_char, pos);
  }
}

void SER_events_process() {
  while (Serial.available() > 0) {
    byte c = Serial.read();
    if (c != '\n') {
      SER_consume_char(c);
    }
  }
}

void SER_setup() {
  Serial.begin(115200);
}

void setup() {
  SCR_setup();
  KBD_setup();
  SER_setup();
}

void loop() {
  // I have checked and this LowPower actually makes the uC sleep for exactly 1ms
  // Somehow if you have the USB data lines connected, then this SLEEP_15MS becomes SLEEP_1MS
  // I don't know how but I gave up finding out.
  
  LowPower.idle(SLEEP_15MS, ADC_ON, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
                  TIMER0_OFF, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
                  
  KBD_events_process();
  SCR_events_process();
  SER_events_process();
  _millis += 1;
}
