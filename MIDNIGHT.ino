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
  // Rebind DC helpers to chosen pin (keep simple by writing directly)
  // init (DC pin in wrCmd/wrDat uses PIN_DC; update it temporarily)
  // We'll temporarily alias PIN_DC using a function pointer-like pattern:
  // (simplify: we just use the global PIN_DC; but for swapped pass we pass pins swapped)
  // For swapped pass we'll redefine DC/RESET calls by temporarily changing DDR—handled by parameters.

  // Minor hack: map DC_C/DC_D to chosen pin
  // (do it by writing directly in wrCmd/wrDat via digitalWrite(PIN_DC,...))
  // so we must temporarily set the global to the provided pin.
  // We'll do it by casting away const-ness (safe here for demo).
  *(int*)&PIN_DC = pinDC;

  st7789_init_common(invert);
  fillColorBands(W, H, xofs);
  delay(3000);
}

void setup(){
  Serial.begin(115200);
  delay(100);

  // PASS 1: DC=GPIO_2, RST=GPIO_1
  attempt("A) 170x320 inv=0 xofs=35", false, 170, 320, 35, GPIO_2, GPIO_1);
  attempt("A) 170x320 inv=1 xofs=35", true,  170, 320, 35, GPIO_2, GPIO_1);
  attempt("A) 170x320 inv=0 xofs=0",  false, 170, 320, 0,  GPIO_2, GPIO_1);
  attempt("A) 240x320 inv=0 xofs=0",  false, 240, 320, 0,  GPIO_2, GPIO_1);

  // PASS 2: swapped DC/RST (some boards mislabel RES/DC)
  attempt("B) 170x320 inv=0 xofs=35 (swapped DC/RST)", false, 170, 320, 35, GPIO_1, GPIO_2);
  attempt("B) 170x320 inv=1 xofs=35 (swapped DC/RST)", true,  170, 320, 35, GPIO_1, GPIO_2);
  attempt("B) 170x320 inv=0 xofs=0 (swapped DC/RST)",  false, 170, 320, 0,  GPIO_1, GPIO_2);
  attempt("B) 240x320 inv=0 xofs=0 (swapped DC/RST)",  false, 240, 320, 0,  GPIO_1, GPIO_2);
}

void loop(){}
