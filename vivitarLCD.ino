static const byte LED_PINS_COUNT = 7;
static const byte SCREEN_SEGMENTS = 42;
static const byte DIGIT_LED_SEGMENTS = 7;
byte ledPins[LED_PINS_COUNT] = {PD0, PD1, PD2, PD3, PD4, PD5, PD6};
unsigned int current_scan_i = 0;
unsigned int current_scan_offset = 0;
bool screen_buf[SCREEN_SEGMENTS];
bool output_once = false;
unsigned long lastRefresh;

byte screen_segment[LED_PINS_COUNT * LED_PINS_COUNT] =
  {
    00, 12, 13, 16,  3,  4,  8,
    17, 00, 19, 20, 23, 22, 11,
    18, 24, 00, 26, 28,  2,  6,
    14, 25, 32, 00, 29, 39,  7,
    15, 21, 33, 27, 00, 37, 41,
    20,  1, 31, 38, 40, 00, 36,
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

void draw_screen() {
  for (byte i = 0; i < LED_PINS_COUNT; i++) {
    pinMode(ledPins[i], INPUT);
  }
  
  for (byte j = 0; j < LED_PINS_COUNT; j++) {
    if (current_scan_i == j) {
      pinMode(ledPins[current_scan_i], OUTPUT);
      digitalWrite(ledPins[current_scan_i], HIGH);
      continue;
    }
    
    if (screen_buf[ screen_segment[current_scan_offset + j] ] == true) {
      pinMode(ledPins[j], OUTPUT);
      digitalWrite(ledPins[j], LOW);
    } else {
      digitalWrite(ledPins[j], HIGH);
    }
  }
  
  current_scan_i = (current_scan_i + 1) % LED_PINS_COUNT;
  current_scan_offset = current_scan_i * 7;
}



void setup() {
  for (byte i = 0; i < LED_PINS_COUNT; i++) {
    pinMode(ledPins[i], INPUT);
    digitalWrite(ledPins[i], HIGH);
  }
 
  // Empty the screen buffer
  for (byte i = 0; i < SCREEN_SEGMENTS; i++) {
    screen_buf[i] = false;
  }
  screen_buf[11] = true;
  draw_number(8, 0);
  draw_number(2, 1);
  draw_number(2, 2);
  draw_number(5, 3);
}

void loop() {
  if (millis() > lastRefresh + 15) { 
    draw_screen();
    lastRefresh = millis();
  }
}
