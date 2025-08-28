/*
  Portenta H7 + Breakout + 1.9" ST7789 (170x320)
  ASCII Dashboard (4 boxes: 2x2) + Backlight PWM + Haptics
  - Background: #444a7d
  - Text: size = 2
  - Boxes: two top, two bottom; touch left/right edges, equal top/bottom margins, even center gap
  - Polished ASCII box:
      +---[Label]---+
      |   Value     |
      +-------------+
  - UP/DOWN: backlight (active-LOW PWM on PWM3)
  - OK: haptic buzz (PWM4, digital pulse)
  - SPI 48 MHz, ROTATION = 1
*/

#include <Arduino.h>
#include <Arduino_PortentaBreakout.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <string.h>
#include <math.h>

// ===== Display tuning =====
#define ROTATION       1
#define COLSTART       35
#define ROWSTART       0
#define INVERT_COLORS  0

// ===== Vibration config =====
#define VIB_ACTIVE_LOW 0    // your driver is active-HIGH (HIGH=ON)
#define VIB_USE_PWM    0    // 0 = robust digital pulse; 1 = PWM (optional)

// -------- Pins (Portenta Breakout) --------
const int         PIN_CS    = SPI1_CS;
const int         PIN_SCK   = SPI1_CK;
const int         PIN_MOSI  = SPI1_COPI;
const int         PIN_DC    = GPIO_2;
const int         PIN_RST   = GPIO_1;
const breakoutPin PIN_BL    = PWM3;     // Backlight (ACTIVE-LOW, PWM)
const breakoutPin PIN_VIBR  = PWM4;     // Vibration motor input

// Buttons (external pull-ups -> pressed = LOW)
const int BTN_RIGHT = GPIO_3;
const int BTN_DOWN  = GPIO_4;
const int BTN_LEFT  = GPIO_5;
const int BTN_UP    = GPIO_6;
const int BTN_OK    = GPIO_0;

// ===== Colors (RGB565) =====
#define C565(r,g,b) ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b) >> 3) )
// Background #444a7d (68,74,125)
const uint16_t COL_BG    = C565(68, 74, 125);
const uint16_t COL_TEXT  = C565(235,239,245);  // light text

// ===== Text geometry (size-2 font) =====
const int CHAR_W = 12; // 6 * 2
const int CHAR_H = 16; // 8 * 2

// ======= Types =======
struct BtnState { uint8_t pin; bool pressed; const char* name; };
struct BoxPx   { int x; int y; };  // pixel-based placement

// ======= Globals =======
BtnState buttons[5] = {
  { (uint8_t)BTN_UP,    false, "UP"    },
  { (uint8_t)BTN_DOWN,  false, "DOWN"  },
  { (uint8_t)BTN_LEFT,  false, "LEFT"  },
  { (uint8_t)BTN_RIGHT, false, "RIGHT" },
  { (uint8_t)BTN_OK,    false, "OK"    }
};
const uint16_t DEBOUNCE_MS = 25;
uint32_t lastChange[5] = {0,0,0,0,0};

BoxPx   gBox[4];          // 4 ASCII boxes (2x2)
uint8_t gBrightness = 220;

// ---- Arduino_GFX: HW SPI on Portenta's SPI1 ----
Arduino_DataBus *bus = new Arduino_HWSPI(PIN_DC, PIN_CS, &SPI);
Arduino_GFX *gfx = new Arduino_ST7789(
  bus, PIN_RST,
  ROTATION, true,
  170, 320,
  COLSTART, ROWSTART,
  COLSTART, ROWSTART
);

// ======= Prototypes =======
void drawNiceBox(int x, int y, int wChars, const char* label, const char* value);
void computeLayoutPx();
void drawHeader();
float simTempF();
void fmtTempNoSpace(char out[5], float f);   // e.g., "80F"
void setupButtons();
void adjustBrightness(int delta);
void updateButton(BtnState &b, uint8_t idx);
void vibrate(uint16_t ms = 140);

// ===== Backlight (ACTIVE-LOW PWM) =====
static inline void blPinMode() { Breakout.pinMode(PIN_BL, OUTPUT); }
static inline void blSet(uint8_t brightness) {
  // brightness 0..255 (0=off, 255=full); invert for active-LOW
  Breakout.analogWrite(PIN_BL, 255 - brightness);
}

// ===== Haptics =====
static inline void vibPinMode() { Breakout.pinMode(PIN_VIBR, OUTPUT); }
#if VIB_USE_PWM
  static inline void vibPWM(uint8_t level) {
    uint8_t duty = VIB_ACTIVE_LOW ? (255 - level) : level;
    Breakout.analogWrite(PIN_VIBR, duty);
  }
  static inline void vibOff() { vibPWM(0); }
  void vibrate(uint16_t ms, uint8_t strength) {
    vibPWM(strength); delay(ms); vibOff();
  }
#else
  static inline void vibOn()  { Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? LOW  : HIGH); }
  static inline void vibOff() { Breakout.digitalWrite(PIN_VIBR, VIB_ACTIVE_LOW ? HIGH : LOW ); }
  void vibrate(uint16_t ms) { vibOn(); delay(ms); vibOff(); }
#endif

// ===== ASCII polished box =====
// Width in characters (wChars) -> we use 12 chars (144 px) per box.
// line1: +---[Label]----+   (label centered within top border)
// line2: |    Value     |   (value centered)
// line3: +--------------+
void drawNiceBox(int x, int y, int wChars, const char* label, const char* value) {
  if (wChars < 8) return; // need some room

  const int L = wChars;
  const int inner = L - 2;   // between the '+' or '|'

  // --- Build line1
  static char line1[64];
  int labLen = (int)strlen(label);
  if (labLen > inner - 2) labLen = inner - 2; // leave room for [] at least

  // core: "[label]" length = labLen + 2
  int coreLen = labLen + 2;
  int rem = inner - coreLen;
  int leftFill  = rem / 2;
  int rightFill = rem - leftFill;

  int pos = 0;
  line1[pos++] = '+';
  for (int i = 0; i < leftFill;  ++i) line1[pos++] = '-';
  line1[pos++] = '[';
  for (int i = 0; i < labLen;    ++i) line1[pos++] = label[i];
  line1[pos++] = ']';
  for (int i = 0; i < rightFill; ++i) line1[pos++] = '-';
  line1[pos++] = '+';
  line1[pos] = '\0';

  // --- Build line2 (centered value)
  static char line2[64];
  int valLen = (int)strlen(value);
  if (valLen > inner) valLen = inner;
  int space = inner - valLen;
  int leftPad  = space / 2;
  int rightPad = space - leftPad;

  pos = 0;
  line2[pos++] = '|';
  for (int i = 0; i < leftPad;  ++i) line2[pos++] = ' ';
  for (int i = 0; i < valLen;   ++i) line2[pos++] = value[i];
  for (int i = 0; i < rightPad; ++i) line2[pos++] = ' ';
  line2[pos++] = '|';
  line2[pos] = '\0';

  // --- Build line3 (bottom border)
  static char line3[64];
  pos = 0;
  line3[pos++] = '+';
  for (int i = 0; i < inner; ++i) line3[pos++] = '-';
  line3[pos++] = '+';
  line3[pos] = '\0';

  // Draw
  gfx->setTextColor(COL_TEXT, COL_BG);
  gfx->setCursor(x, y);                   gfx->print(line1);
  gfx->setCursor(x, y + CHAR_H);          gfx->print(line2);
  gfx->setCursor(x, y + 2 * CHAR_H);      gfx->print(line3);
}

// ===== Pixel layout for 2 columns × 2 rows =====
void computeLayoutPx() {
  // Box width: 12 chars → 144 px; height: 3 lines → 48 px
  const int BOX_W_PX = 12 * CHAR_W;   // 144
  const int BOX_H_PX = 3  * CHAR_H;   // 48

  const int W = 320, H = 170;

  // Even horizontal spacing: margins 8 px, center gap 16 px
  const int MARGIN_L = 8;
  const int CENTER_G = 16;
  const int MARGIN_R = 8;

  // x positions
  int x0 = MARGIN_L;
  int x1 = W - MARGIN_R - BOX_W_PX;   // symmetric

  // Vertical spacing: top=24, bottom=24, middle gap computed
  const int TOP  = 24;
  const int BOT  = 24;
  const int MIDG = H - TOP - BOT - 2*BOX_H_PX; // equals 26 with H=170
  int y0 = TOP;
  int y1 = TOP + BOX_H_PX + MIDG;

  // Assign 4 boxes
  gBox[0] = { x0, y0 };  // top-left
  gBox[1] = { x1, y0 };  // top-right
  gBox[2] = { x0, y1 };  // bottom-left
  gBox[3] = { x1, y1 };  // bottom-right
}

// ===== Header =====
void drawHeader() {
  // Small one-liner; boxes start at y=24 so no clash.
  gfx->setTextColor(COL_TEXT, COL_BG);
  gfx->setTextSize(2);
  gfx->setCursor(8, 2);  gfx->print("Portenta H7  ST7789  rot=1 off(35,0)");

  char buf[16];
  int pct = (int)gBrightness * 100 / 255;
  snprintf(buf, sizeof(buf), "BL:%3d%%", pct);
  int16_t tw = strlen(buf) * CHAR_W;
  gfx->setCursor(320 - 8 - tw, 2);  gfx->print(buf);
}

// ===== Temp (simulated for now) =====
float simTempF() {
  static uint32_t t0 = millis();
  float s = sinf((millis() - t0) / 3000.0f);
  return 80.0f + s * 1.0f; // ~80 F ±1
}
void fmtTempNoSpace(char out[5], float f) {
  int v = (int)roundf(f);
  snprintf(out, 5, "%dF", v); // "80F"
}

// ===== Buttons =====
void setupButtons() {
  pinMode(BTN_UP,    INPUT);
  pinMode(BTN_DOWN,  INPUT);
  pinMode(BTN_LEFT,  INPUT);
  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_OK,    INPUT);
}

void adjustBrightness(int delta) {
  int v = (int)gBrightness + delta;
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  if (v != gBrightness) {
    gBrightness = (uint8_t)v;
    blSet(gBrightness);
    drawHeader();
  }
}

void updateButton(BtnState &b, uint8_t idx) {
  bool rawPressed = (digitalRead(b.pin) == LOW);
  uint32_t now = millis();

  if (rawPressed != b.pressed && (now - lastChange[idx]) > DEBOUNCE_MS) {
    b.pressed = rawPressed;
    lastChange[idx] = now;
    if (b.pressed) {
      vibrate(120);
      if (b.pin == BTN_UP)   adjustBrightness(+16);
      if (b.pin == BTN_DOWN) adjustBrightness(-16);
    }
  }
}

// ===== Arduino plumbing =====
void setup() {
  // Backlight
  blPinMode();
  blSet(gBrightness);

  // SPI + display
  SPI.begin();
  gfx->begin(48000000);  // 48 MHz
  gfx->invertDisplay(INVERT_COLORS);

  // Text defaults
  gfx->setTextWrap(false);
  gfx->setTextSize(2);
  gfx->setTextColor(COL_TEXT, COL_BG);

  // IO
  setupButtons();
  vibPinMode();
  vibOff();

  // Screen
  gfx->fillScreen(COL_BG);
  computeLayoutPx();
  drawHeader();

  // Initial draw of 4 boxes
  char temp[5]; fmtTempNoSpace(temp, simTempF());
  drawNiceBox(gBox[0].x, gBox[0].y, 12, "Temp", temp); // top-left: Temp
  drawNiceBox(gBox[1].x, gBox[1].y, 12, "    ", "--"); // top-right placeholder
  drawNiceBox(gBox[2].x, gBox[2].y, 12, "    ", "--"); // bottom-left placeholder
  drawNiceBox(gBox[3].x, gBox[3].y, 12, "    ", "--"); // bottom-right placeholder
}

void loop() {
  // Buttons
  for (uint8_t i = 0; i < 5; ++i) updateButton(buttons[i], i);

  // Update temp every 500 ms (only the Temp box)
  static uint32_t t = 0;
  if (millis() - t > 500) {
    t = millis();
    char temp[5]; fmtTempNoSpace(temp, simTempF());
    drawNiceBox(gBox[0].x, gBox[0].y, 12, "Temp", temp);
  }
}
