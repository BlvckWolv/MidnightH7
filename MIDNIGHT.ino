//
// Portenta Breakout → ST7789 "no-driver" bring-up (bit-banged SPI using Breakout pins)
// ------------------------------------------------------------------------------------------------
// Goal: prove pixels with ZERO dependency on SPI mapping or Adafruit internals.
// We directly wiggle the SPI1 header pins using Arduino_PortentaBreakout symbols.
//
// Pins (from your table):
//   CS   = SPI1_CS   (PI_0)
//   SCK  = SPI1_CK   (PI_1)
//   MOSI = SPI1_COPI (PC_3)
//   DC   = GPIO_2    (PD_4)
//   RST  = GPIO_1    (PC_15)
//
// What this sketch does:
//   • Bit-bangs SPI Mode 0 at a safe pace
//   • Hard-resets panel
//   • Tries 4 attempts in a loop (3s each):
//       1) 170x320, no inversion, X offset 35 (common 1.9" ST7789)
//       2) 170x320, inversion,   X offset 35
//       3) 170x320, no inversion, X offset 0
//       4) 240x320, no inversion, X offset 0
//     (It also tries a DC/RST swap pass if the first pass shows nothing.)
//   • On each attempt: prints a label on Serial and fills big colored bands
//
// Power note: If your module has an onboard LDO / level shifters, feed VCC=5V. Else use 3V3.
//
// Requirements: Board Portenta H7; Library Arduino_PortentaBreakout
//
#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>

// -------- Pin symbols (from Arduino_PortentaBreakout) --------
const int PIN_CS   = SPI1_CS;    // PI_0
const int PIN_SCK  = SPI1_CK;    // PI_1
const int PIN_MOSI = SPI1_COPI;  // PC_3
const int PIN_DC   = GPIO_2;     // PD_4
const int PIN_RST  = GPIO_1;     // PC_15
const breakoutPin PIN_BL = PWM3;  // Backlight control
const int PIN_VIBR = PWM4; // Vibration motor control (using PWM4 as GPIO output)

// Button pins (all with external 10k pull-up resistors)
const int BTN_RIGHT  = GPIO_3;   // Right button
const int BTN_DOWN   = GPIO_4;   // Down button  
const int BTN_LEFT   = GPIO_5;   // Left button
const int BTN_UP     = GPIO_6;   // Up button
const int BTN_OK     = GPIO_0;   // Middle/OK button

// -------- Bit-bang SPI helpers (Mode 0) --------
static inline void SCK_L(){ digitalWrite(PIN_SCK, LOW); }
static inline void SCK_H(){ digitalWrite(PIN_SCK, HIGH); }
static inline void MOSI_W(uint8_t v){ digitalWrite(PIN_MOSI, v); }
static inline void CS_L(){ digitalWrite(PIN_CS, LOW); }
static inline void CS_H(){ digitalWrite(PIN_CS, HIGH); }
static inline void DC_C(){ digitalWrite(PIN_DC, LOW); }   // command
static inline void DC_D(){ digitalWrite(PIN_DC, HIGH); }  // data

static inline void bb_delay(){ __asm__ __volatile__("nop;nop;nop;nop;nop;"); }

static void spiWriteByte(uint8_t b){
  for (int i=7; i>=0; --i){
    MOSI_W((b>>i)&1);
    SCK_H(); bb_delay();
    SCK_L(); bb_delay();
  }
}

static void wrCmd(uint8_t c){
  CS_L(); DC_C(); spiWriteByte(c); CS_H();
}

static void wrDat(uint8_t d){
  CS_L(); DC_D(); spiWriteByte(d); CS_H();
}

static void wrDat16(uint16_t d){
  CS_L(); DC_D(); spiWriteByte(d>>8); spiWriteByte(d&0xFF); CS_H();
}

// -------- Backlight control --------
static void setBacklight(uint8_t brightness){
  // brightness: 0-255, but invert for MOSFET (high PWM = dim, low PWM = bright)
  uint8_t invertedBrightness = 255 - brightness;
  Breakout.analogWrite(PIN_BL, invertedBrightness);
}

// -------- Vibration motor control --------
static void setVibration(bool on){
  // Simple on/off control using GPIO
  digitalWrite(PIN_VIBR, on ? HIGH : LOW);
}

static void vibrateShort(){
  // Quick tactile feedback for navigation buttons
  setVibration(true);   // Turn on
  delay(80);            // Increased to 80ms for better feel
  setVibration(false);  // Turn off
}

static void vibrateLong(){
  // Longer feedback for confirmations
  setVibration(true);   // Turn on
  delay(200);           // 200ms duration
  setVibration(false);  // Turn off
}

static void vibratePattern(){
  // Double-pulse pattern for OK button (special confirmation)
  setVibration(true);
  delay(60);            // Slightly shorter pulses
  setVibration(false);
  delay(40);            // Shorter gap
  setVibration(true);
  delay(60);            // Second pulse
  setVibration(false);
}

// Test function - call this from Serial Monitor
static void testVibration(){
  Serial.println("=== VIBRATION TEST ===");
  Serial.println("1. Short pulse...");
  vibrateShort();
  delay(500);
  Serial.println("2. Long pulse...");
  vibrateLong();
  delay(500);
  Serial.println("3. Pattern...");
  vibratePattern();
  Serial.println("Vibration test complete!");
}

// -------- Multi-button handling with debouncing --------
struct ButtonState {
  int pin;
  const char* name;
  bool lastState;
  bool currentState;
  unsigned long lastDebounceTime;
  bool wasPressed;
};

static ButtonState buttons[] = {
  {BTN_RIGHT, "RIGHT", HIGH, HIGH, 0, false},
  {BTN_DOWN,  "DOWN",  HIGH, HIGH, 0, false},
  {BTN_LEFT,  "LEFT",  HIGH, HIGH, 0, false},
  {BTN_UP,    "UP",    HIGH, HIGH, 0, false},
  {BTN_OK,    "OK",    HIGH, HIGH, 0, false}
};
static const int numButtons = 5;
static const unsigned long debounceDelay = 50;  // 50ms debounce

static void updateButtons(){
  for(int i = 0; i < numButtons; i++){
    bool reading = digitalRead(buttons[i].pin);
    
    if (reading != buttons[i].lastState) {
      buttons[i].lastDebounceTime = millis();
    }
    
    if ((millis() - buttons[i].lastDebounceTime) > debounceDelay) {
      if (reading != buttons[i].currentState) {
        buttons[i].currentState = reading;
        
        if (buttons[i].currentState == LOW) {  // Button pressed
          Serial.print("*** ");
          Serial.print(buttons[i].name);
          Serial.println(" BUTTON PRESSED ***");
          buttons[i].wasPressed = true;
        } else {  // Button released
          Serial.print("*** ");
          Serial.print(buttons[i].name);
          Serial.println(" BUTTON RELEASED ***");
        }
      }
    }
    
    buttons[i].lastState = reading;
  }
}

static void checkAllButtons(){
  static unsigned long lastPrint = 0;
  
  // Print all button states every 2000ms for debugging (less frequent)
  if (millis() - lastPrint > 2000) {
    Serial.print("Button states: ");
    for(int i = 0; i < numButtons; i++){
      bool reading = digitalRead(buttons[i].pin);
      Serial.print(buttons[i].name);
      Serial.print(":");
      Serial.print(reading ? "HIGH" : "LOW");
      if(i < numButtons-1) Serial.print(", ");
    }
    Serial.println();
    
    // Extra focus on UP and DOWN buttons
    Serial.print("UP (GPIO_6) raw: ");
    Serial.print(digitalRead(BTN_UP) ? "HIGH" : "LOW");
    Serial.print(", DOWN (GPIO_4) raw: ");
    Serial.println(digitalRead(BTN_DOWN) ? "HIGH" : "LOW");
    
    lastPrint = millis();
  }
  
  // Update button states and detect presses
  updateButtons();
  
  // Check for any button presses and handle them
  for(int i = 0; i < numButtons; i++){
    if(buttons[i].wasPressed){
      Serial.print("==> ");
      Serial.print(buttons[i].name);
      Serial.println(" button action detected!");
      
      // Handle specific button actions with vibration feedback
      if(buttons[i].pin == BTN_OK){
        Serial.println("    -> OK button: Action confirmed!");
        vibratePattern();  // Special double-pulse for OK button
      } else if(buttons[i].pin == BTN_UP){
        Serial.println("    -> UP button: Navigate up!");
        vibrateShort();    // Quick pulse for navigation
      } else if(buttons[i].pin == BTN_DOWN){
        Serial.println("    -> DOWN button: Navigate down!");
        vibrateShort();    // Quick pulse for navigation
      } else if(buttons[i].pin == BTN_LEFT){
        Serial.println("    -> LEFT button: Navigate left!");
        vibrateShort();    // Quick pulse for navigation
      } else if(buttons[i].pin == BTN_RIGHT){
        Serial.println("    -> RIGHT button: Navigate right!");
        vibrateShort();    // Quick pulse for navigation
      }
      
      buttons[i].wasPressed = false;  // Reset the flag
    }
  }
}

// -------- Minimal ST7789 init sequence --------
static void panelReset(int pinRST){
  pinMode(pinRST, OUTPUT);
  digitalWrite(pinRST, HIGH); delay(5);
  digitalWrite(pinRST, LOW);  delay(20);
  digitalWrite(pinRST, HIGH); delay(150);
}

static void st7789_init_common(bool invert){
  wrCmd(0x01); delay(150);          // SWRESET
  wrCmd(0x11); delay(120);          // SLPOUT

  wrCmd(0x3A); wrDat(0x55);         // COLMOD=16-bit
  wrCmd(0x36); wrDat(0x00);         // MADCTL: RGB (0x00) (try 0x08 for BGR if needed)

  // Light-touch power settings that work on many boards
  wrCmd(0xB7); wrDat(0xC6);         // VCOMS
  wrCmd(0xBB); wrDat(0x1B);         // VCOM
  wrCmd(0xC0); wrDat(0x2C);         // LCMCTRL
  wrCmd(0xC2); wrDat(0x01); wrDat(0xFF); // VDV/VRH enable
  wrCmd(0xC3); wrDat(0x0F);         // VRH
  wrCmd(0xC4); wrDat(0x20);         // VDV
  wrCmd(0xC6); wrDat(0x0F);         // FRCTRL2: frame rate
  wrCmd(0xD0); wrDat(0xA4); wrDat(0xA1); // Power control
  if (invert) { wrCmd(0x21); } else { wrCmd(0x20); }  // INVON/INVOFF

  wrCmd(0x13);                      // NORON
  wrCmd(0x29); delay(20);           // DISPON
}

// Set column/row window (with optional X offset)
static void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t xofs){
  uint16_t x0 = x + xofs;
  uint16_t x1 = x0 + w - 1;
  uint16_t y0 = y;
  uint16_t y1 = y + h - 1;
  wrCmd(0x2A); // CASET
  wrDat16(x0); wrDat16(x1);
  wrCmd(0x2B); // RASET
  wrDat16(y0); wrDat16(y1);
  wrCmd(0x2C); // RAMWR
}

static void fillColorBands(uint16_t W, uint16_t H, uint16_t xofs){
  const uint16_t cols[] = { 0xF800, 0xFFE0, 0x07E0, 0x07FF, 0x001F, 0xF81F, 0xFFFF };
  const int bands = 7;
  int h = H / bands;
  for (int i=0;i<bands;i++){
    int y = i*h;
    int hh = (i==bands-1) ? (H - y) : h;
    setAddrWindow(0, y, W, hh, xofs);
    uint32_t px = (uint32_t)W * hh;
    uint16_t c = cols[i];
    CS_L(); DC_D();
    for (uint32_t k=0;k<px;k++){ spiWriteByte(c>>8); spiWriteByte(c&0xFF); }
    CS_H();
  }
}

// Try one configuration
static void attempt(const char* label, bool invert, uint16_t W, uint16_t H, uint16_t xofs, int pinDC, int pinRST){
  Serial.println(label);
  // set pin modes
  pinMode(PIN_CS, OUTPUT);  CS_H();
  pinMode(PIN_SCK, OUTPUT); SCK_L();
  pinMode(PIN_MOSI, OUTPUT); MOSI_W(LOW);
  pinMode(pinDC, OUTPUT); digitalWrite(pinDC, HIGH);
  panelReset(pinRST);
  
  // For DC/RST swapping, we need to update the global DC pin reference
  *(int*)&PIN_DC = pinDC;

  st7789_init_common(invert);
  fillColorBands(W, H, xofs);
  delay(3000);
}

// Demo animation after we find working config
static void runDemo(uint16_t W, uint16_t H, uint16_t xofs){
  Serial.println("Running demo animation...");
  
  // Animated color sweep
  for(int frame = 0; frame < 20; frame++){
    uint16_t colors[] = {
      0xF800 + (frame * 0x0841),  // Red to yellow
      0x07E0 + (frame * 0x0020),  // Green variations  
      0x001F + (frame * 0x0800),  // Blue to magenta
      0xFFFF - (frame * 0x1084)   // White to cyan
    };
    
    for(int i = 0; i < 4; i++){
      setAddrWindow(0, i * (H/4), W, H/4, xofs);
      uint32_t pixels = (uint32_t)W * (H/4);
      CS_L(); DC_D();
      for(uint32_t p = 0; p < pixels; p++){
        spiWriteByte(colors[i] >> 8);
        spiWriteByte(colors[i] & 0xFF);
      }
      CS_H();
    }
    delay(50);
  }
}

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("MIDNIGHT01: Bit-banged ST7789 Test + Demo");
  Serial.println("==========================================");

  // Setup backlight
  pinMode(PIN_BL, OUTPUT);
  Serial.println("Testing backlight...");
  setBacklight(255);  // Full brightness for 1 second
  delay(1000);
  setBacklight(200);  // Then 80% brightness
  Serial.println("Backlight test complete");

  // Setup vibration motor
  pinMode(PIN_VIBR, OUTPUT);
  Serial.println("Testing vibration motor...");
  Serial.println("Wire: PWM4 -> [680R] -> PN2222A base, PN2222A collector -> motor(-), motor(+) -> 3.3V");
  
  // Test vibration motor
  Serial.println("Testing vibration motor...");
  vibrateLong();
  delay(500);
  Serial.println("Vibration test complete");

  // Setup all 5 buttons with external 10k pull-up resistors
  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(BTN_LEFT, INPUT);
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_OK, INPUT);
  
  Serial.println("5-Button Navigation Setup:");
  Serial.println("  RIGHT: GPIO_3 -> 3.3V via 10k, Button to GND");
  Serial.println("  DOWN:  GPIO_4 -> 3.3V via 10k, Button to GND");
  Serial.println("  LEFT:  GPIO_5 -> 3.3V via 10k, Button to GND");
  Serial.println("  UP:    GPIO_6 -> 3.3V via 10k, Button to GND");
  Serial.println("  OK:    GPIO_0 -> 3.3V via 10k, Button to GND");
  Serial.println("Press any button to test...");

  // LED indicator
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Display tests disabled - focusing on buttons and vibration");
  Serial.println("Starting button and vibration motor test...");
}

void loop(){
  // Check all buttons continuously
  checkAllButtons();
  
  // Check for Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "vibrate" || cmd == "v") {
      testVibration();
    } else if (cmd == "pwm") {
      Serial.println("Testing PWM4 vibration motor...");
      Serial.println("Motor ON for 1 second...");
      setVibration(true);
      delay(1000);
      setVibration(false);
      Serial.println("Motor OFF - test complete");
    }
  }
  
  // Simple LED indicator and button/vibration focus
  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
  
  // Keep backlight at steady 80% brightness
  static bool backlightSet = false;
  if (!backlightSet) {
    setBacklight(200);  // 80% brightness
    backlightSet = true;
  }
  
  delay(10);  // Small delay for button responsiveness
}
