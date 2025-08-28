/*
  Portenta H7 + Breakout + 1.9" ST7789 (170x320)
  Buttons + Vibration Motor Demo (HW SPI + Arduino_GFX, PWM ramp)

  - Display: ST7789, ROTATION=3, offsets=(35,0)  [your proven working combo]
  - Backlight: ACTIVE-LOW on PWM3 (LOW = ON), held on as plain GPIO
  - Buttons: external pull-ups, pressed = LOW
  - Vibration motor: PWM4 via Breakout.* API with ramped PWM buzz
    * VIB_ACTIVE_LOW = 0 (your driver is active-HIGH: HIGH=ON)
    * Change to 1 if your driver expects LOW=ON

  Wiring (Breakout J2 header):
    VCC→3V3, GND→GND
    SCL→SPI1_CK, SDA(MOSI)→SPI1_COPI, CS→SPI1_CS, DC→GPIO_2, RST→GPIO_1
    BLK→PWM3  (active-LOW)  [or tie to GND to force always-on]
    Motor driver IN → PWM4  (use transistor/MOSFET + flyback diode)
*/

#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <string.h>  // strlen

// ===== Display tuning (working setup) =====
#define ROTATION       3
#define COLSTART       35
#define ROWSTART       0
#define INVERT_COLORS  0
// ==========================================

// ===== Vibration configuration =====
#define VIB_ACTIVE_LOW 0   // <-- your driver is active-HIGH (HIGH = ON)
// ==================================

// -------- Pin symbols (from Arduino_PortentaBreakout) --------
const int         PIN_CS    = SPI1_CS;
const int         PIN_SCK   = SPI1_CK;
const int         PIN_MOSI  = SPI1_COPI;
const int         PIN_DC    = GPIO_2;
const int         PIN_RST   = GPIO_1;
const breakoutPin PIN_BL    = PWM3;     // Backlight (ACTIVE-LOW)
const breakoutPin PIN_VIBR  = PWM4;     // Vibration motor (Breakout API)

// Buttons (external 10k pull-ups -> pressed = LOW)
const int BTN_RIGHT = GPIO_3;
const int BTN_DOWN  = GPIO_4;
const int BTN_LEFT  = GPIO_5;
const int BTN_UP    = GPIO_6;
const int BTN_OK    = GPIO_0;

// -------- Colors (RGB565) --------
#define C565(r,g,b) ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b) >> 3) )
const uint16_t COL_BG      = C565(  0,  0,  0);
const uint16_t COL_GRID    = C565( 60, 60, 60);
const uint16_t COL_TEXT    = C565(255,255,255);
const uint16_t COL_FRAME   = C565(200,200,200);
const uint16_t COL_IDLE    = C565( 30,140,255);  // button idle fill
const uint16_t COL_ACTIVE  = C565(255,190, 40);  // button pressed highlight
const uint16_t COL_OK      = C565( 80,220,120);

// -------- Button UI layout/types (define BEFORE use) --------
struct BtnGeo   { int x, y, w, h; const char* label; };
struct BtnState { uint8_t pin; bool pressed; BtnGeo geo; const char* name; };

BtnGeo gUp, gDown, gLeft, gRight, gOk;
BtnState buttons[5];
const uint16_t DEBOUNCE_MS = 25;
uint32_t lastChange[5] = {0,0,0,0,0};

// ---- Arduino_GFX: HW SPI on Portenta's SPI1 ----
Arduino_DataBus *bus = new Arduino_HWSPI(PIN_DC, PIN_CS, &SPI);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, PIN_RST,
  ROTATION, true,        // rotation, IPS
  170, 320,              // width, height (panel native)
  COLSTART, ROWSTART,    // window A offsets
  COLSTART, ROWSTART     // window B offsets
);

// -------- Backlight control (ACTIVE-LOW) --------
static inline void backlightOn()   { pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, LOW);  }
static inline void backlightKeep() { digitalWrite(PIN_BL, LOW); }

// -------- Vibration helpers (PWM + ramp using Breakout API) --------
static inline void vibPinMode() { Breakout.pinMode(PIN_VIBR, OUTPUT); }

static inline void vibPWM(uint8_t level) {
  // level: 0..255 = 0..100% strength
  uint8_t duty = VIB_ACTIVE_LOW ? (255 - level) : level;
  Breakout.analogWrite(PIN_VIBR, duty);
}

static inline void vibOff() { vibPWM(0); }

// Smooth buzz with simple ramp up/down
void vibrate(uint16_t ms = 150, uint8_t strength = 255, bool ramp = true) {
  const uint8_t steps = 4;
  const uint16_t step_ms = 12;
  uint16_t ramp_time = ramp ? steps * step_ms * 2 : 0;

  if (ramp) {
    for (uint8_t i = 1; i <= steps; ++i) {
      uint8_t lvl = (uint16_t)strength * i / steps; // 25/50/75/100%
      vibPWM(lvl);
      delay(step_ms);
    }
  } else {
    vibPWM(strength);
  }

  if (ms > ramp_time) delay(ms - ramp_time);

  if (ramp) {
    for (int8_t i = steps - 1; i >= 0; --i) {
      uint8_t lvl = (uint16_t)strength * i / steps;
      vibPWM(lvl);
      delay(step_ms);
    }
  }

  vibOff();
}

// -------- Layout & Drawing --------
void computeLayout() {
  int W = gfx->width();   // expect 320
  int H = gfx->height();  // expect 170
  int bw = 72, bh = 40;   // button size
  int cx = W/2, cy = H/2;

  gOk   = { cx - bw/2, cy - bh/2, bw, bh, "OK" };
  gUp   = { cx - bw/2, gOk.y - (bh + 8), bw, bh, "UP" };
  gDown = { cx - bw/2, gOk.y + gOk.h + 8, bw, bh, "DOWN" };
  gLeft = { gOk.x - (bw + 12), gOk.y, bw, bh, "LEFT" };
  gRight= { gOk.x + gOk.w + 12, gOk.y, bw, bh, "RIGHT" };
}

void drawButton(const BtnGeo& b, bool pressed, uint16_t idleFill, uint16_t textCol) {
  uint16_t fill = pressed ? COL_ACTIVE : idleFill;
  gfx->fillRoundRect(b.x, b.y, b.w, b.h, 8, fill);
  gfx->drawRoundRect(b.x, b.y, b.w, b.h, 8, COL_FRAME);
  gfx->setTextColor(textCol);
  gfx->setTextSize(1);
  int16_t tx = b.x + (b.w - (int)strlen(b.label)*6)/2;
  int16_t ty = b.y + (b.h - 8)/2;
  gfx->setCursor(tx, ty);
  gfx->print(b.label);
}

void drawChrome() {
  gfx->fillScreen(COL_BG);
  // title
  gfx->setTextColor(COL_TEXT);
  gfx->setTextSize(1);
  gfx->setCursor(8, 8);
  gfx->print("Portenta H7 + ST7789 (170x320)  |  ROT=3  offs(35,0)");

  // subtle grid
  for (int x = 0; x < gfx->width(); x += 20) gfx->drawFastVLine(x, 24, gfx->height()-24, COL_GRID);
  for (int y = 24; y < gfx->height(); y += 20) gfx->drawFastHLine(0, y, gfx->width(), COL_GRID);

  computeLayout();
  // draw all buttons idle
  drawButton(gUp,   false, COL_IDLE, COL_TEXT);
  drawButton(gDown, false, COL_IDLE, COL_TEXT);
  drawButton(gLeft, false, COL_IDLE, COL_TEXT);
  drawButton(gRight,false, COL_IDLE, COL_TEXT);
  drawButton(gOk,   false, COL_OK,   COL_BG);
}

// -------- Buttons setup & handler --------
void setupButtons() {
  buttons[0] = { (uint8_t)BTN_UP,    false, gUp,    "UP"    };
  buttons[1] = { (uint8_t)BTN_DOWN,  false, gDown,  "DOWN"  };
  buttons[2] = { (uint8_t)BTN_LEFT,  false, gLeft,  "LEFT"  };
  buttons[3] = { (uint8_t)BTN_RIGHT, false, gRight, "RIGHT" };
  buttons[4] = { (uint8_t)BTN_OK,    false, gOk,    "OK"    };

  pinMode(BTN_UP,    INPUT); // external pull-ups present
  pinMode(BTN_DOWN,  INPUT);
  pinMode(BTN_LEFT,  INPUT);
  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_OK,    INPUT);
}

void updateButton(BtnState &b, uint8_t idx) {
  bool rawPressed = (digitalRead(b.pin) == LOW);   // pressed = LOW
  uint32_t now = millis();

  if (rawPressed != b.pressed && (now - lastChange[idx]) > DEBOUNCE_MS) {
    b.pressed = rawPressed;
    lastChange[idx] = now;

    // redraw this button only
    bool isOK = (b.name[0] == 'O'); // "OK"
    uint16_t idleColor = isOK ? COL_OK : COL_IDLE;
    uint16_t textColor = isOK ? COL_BG : COL_TEXT;
    drawButton(b.geo, b.pressed, idleColor, textColor);

    if (b.pressed) vibrate(120, 255, true);  // haptic tick (ramped)

    // status line
    gfx->fillRect(0, gfx->height()-18, gfx->width(), 18, COL_BG);
    gfx->setTextColor(COL_TEXT); gfx->setTextSize(1);
    gfx->setCursor(8, gfx->height()-14);
    gfx->print("Pressed: "); gfx->print(b.name);
  }
}

// -------- Arduino plumbing --------
void setup() {
  backlightOn();        // ACTIVE-LOW BL
  SPI.begin();
  gfx->begin(24000000);
  gfx->invertDisplay(INVERT_COLORS);

  drawChrome();
  setupButtons();

  // Vibration pin setup + boot buzz
  vibPinMode();
  vibOff();
  vibrate(250, 255, true);  // self-test buzz at boot
}

void loop() {
  backlightKeep();      // keep BL asserted LOW
  for (uint8_t i = 0; i < 5; ++i) updateButton(buttons[i], i);
}
