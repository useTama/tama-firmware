// Button and touch probe for the Cheeko Gotchi board.
//
// Exists because the two references disagree about the middle button's
// polarity, and a wrong guess makes it look like the button is dead:
//
//   SKILL.md section 9, Gotcha 4:  "volume up (GPIO 40) and volume down
//     (GPIO 39) are active LOW with internal pullups, while the middle/power
//     key (GPIO 3) is active HIGH with an external pulldown."
//
//   cheeko_hw_self_test.ino button table: {"PWR", PIN_POWER_KEY, true, ...}
//     where that third field is active_low -- i.e. active LOW, the opposite.
//
// This sketch takes no position. It prints the raw level of every candidate
// pin continuously, under three different input modes, and shows them on
// screen too. Press each physical button in turn and the truth is whichever
// line changes.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

static constexpr int PIN_BOOT_BUTTON = 0;
static constexpr int PIN_POWER_OFF   = 2;   // never drive HIGH: cuts power
static constexpr int PIN_POWER_KEY   = 3;
static constexpr int PIN_LCD_DC      = 8;
static constexpr int PIN_LCD_SCLK    = 9;
static constexpr int PIN_LCD_MOSI    = 10;
static constexpr int PIN_I2C_SCL     = 11;
static constexpr int PIN_I2C_SDA     = 12;
static constexpr int PIN_LCD_BL      = 13;
static constexpr int PIN_LCD_CS      = 14;
static constexpr int PIN_LCD_RST     = 17;
static constexpr int PIN_CHARGE_DET  = 37;
static constexpr int PIN_VOLUME_DOWN = 39;
static constexpr int PIN_VOLUME_UP   = 40;

static constexpr uint8_t CST810_ADDR = 0x15;

static constexpr int LCD_WIDTH  = 240;
static constexpr int LCD_HEIGHT = 296;

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xffff;
static constexpr uint16_t COLOR_GREEN = 0x07e0;
static constexpr uint16_t COLOR_RED   = 0xf800;
static constexpr uint16_t COLOR_AMBER = 0xfd20;
static constexpr uint16_t COLOR_CYAN  = 0x07ff;
static constexpr uint16_t COLOR_GRAY  = 0x8410;

// ---------------------------------------------------------------- display

static void lcdWriteCommand(uint8_t c) {
  digitalWrite(PIN_LCD_DC, LOW); digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(c); digitalWrite(PIN_LCD_CS, HIGH);
}
static void lcdWriteData(uint8_t d) {
  digitalWrite(PIN_LCD_DC, HIGH); digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(d); digitalWrite(PIN_LCD_CS, HIGH);
}
static void lcdSetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  lcdWriteCommand(0x2a);
  lcdWriteData(x0 >> 8); lcdWriteData(x0 & 0xff);
  lcdWriteData(x1 >> 8); lcdWriteData(x1 & 0xff);
  lcdWriteCommand(0x2b);
  lcdWriteData(y0 >> 8); lcdWriteData(y0 & 0xff);
  lcdWriteData(y1 >> 8); lcdWriteData(y1 & 0xff);
  lcdWriteCommand(0x2c);
}
static void lcdFillRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0 || y < 0 || x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
  w = min(w, LCD_WIDTH - x); h = min(h, LCD_HEIGHT - y);
  if (w <= 0 || h <= 0) return;
  static uint8_t rowBuf[LCD_WIDTH * 2];
  const int span = min(w, LCD_WIDTH);
  for (int i = 0; i < span; ++i) { rowBuf[i*2] = color >> 8; rowBuf[i*2+1] = color & 0xff; }
  lcdSetWindow(x, y, x + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH); digitalWrite(PIN_LCD_CS, LOW);
  for (int r = 0; r < h; ++r) SPI.writeBytes(rowBuf, span * 2);
  digitalWrite(PIN_LCD_CS, HIGH);
}
static void lcdFillScreen(uint16_t c) { lcdFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, c); }

static const uint8_t *glyphForChar(char c) {
  static const uint8_t glyphs[][5] = {
      {0x00,0x00,0x00,0x00,0x00},{0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},
      {0x3e,0x41,0x41,0x41,0x22},{0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},
      {0x7f,0x09,0x09,0x09,0x01},{0x3e,0x41,0x49,0x49,0x7a},{0x7f,0x08,0x08,0x08,0x7f},
      {0x00,0x41,0x7f,0x41,0x00},{0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},
      {0x7f,0x40,0x40,0x40,0x40},{0x7f,0x02,0x0c,0x02,0x7f},{0x7f,0x04,0x08,0x10,0x7f},
      {0x3e,0x41,0x41,0x41,0x3e},{0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},
      {0x7f,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7f,0x01,0x01},
      {0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},{0x7f,0x20,0x18,0x20,0x7f},
      {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
      {0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},{0x62,0x51,0x49,0x49,0x46},
      {0x22,0x41,0x49,0x49,0x36},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
      {0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},
      {0x06,0x49,0x49,0x29,0x1e},{0x40,0x30,0x0c,0x03,0x00},{0x00,0x40,0x40,0x40,0x00},
      {0x00,0x60,0x60,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x36,0x36,0x00,0x00},
  };
  if (c >= 'A' && c <= 'Z') return glyphs[1 + c - 'A'];
  if (c >= 'a' && c <= 'z') return glyphs[1 + c - 'a'];
  if (c >= '0' && c <= '9') return glyphs[27 + c - '0'];
  switch (c) {
    case '/': return glyphs[37]; case '_': return glyphs[38];
    case '.': return glyphs[39]; case '-': return glyphs[40];
    case ':': return glyphs[41]; default:  return glyphs[0];
  }
}
static void lcdDrawChar(int x, int y, char c, uint16_t fg, uint16_t bg, int s) {
  const uint8_t *g = glyphForChar(c);
  for (int col = 0; col < 5; ++col) {
    uint8_t bits = g[col];
    for (int row = 0; row < 7; ++row)
      lcdFillRect(x + col*s, y + row*s, s, s, (bits & (1 << row)) ? fg : bg);
  }
}
static void lcdDrawText(int x, int y, const char *t, uint16_t fg, uint16_t bg, int s) {
  while (*t) { lcdDrawChar(x, y, *t, fg, bg, s); x += 6*s; ++t; }
}
static void lcdInit() {
  pinMode(PIN_LCD_CS, OUTPUT); pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT); pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH); digitalWrite(PIN_LCD_BL, HIGH);
  digitalWrite(PIN_LCD_RST, LOW);  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);
  lcdWriteCommand(0x01); delay(150);
  lcdWriteCommand(0x11); delay(120);
  lcdWriteCommand(0x36); lcdWriteData(0x48);
  lcdWriteCommand(0x3a); lcdWriteData(0x55);
  lcdWriteCommand(0x20); lcdWriteCommand(0x13); lcdWriteCommand(0x29);
}

// ---------------------------------------------------------------- touch

static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
static bool touchReadRaw(int &rx, int &ry) {
  uint8_t d[5] = {};
  if (!i2cReadRegs(CST810_ADDR, 0x02, d, sizeof(d))) return false;
  if ((d[0] & 0x0f) == 0) return false;
  rx = ((d[1] & 0x0f) << 8) | d[2];
  ry = ((d[3] & 0x0f) << 8) | d[4];
  return true;
}

// ---------------------------------------------------------------- probe

struct Pin { const char *name; int gpio; };

// Every pin a physical button could plausibly be wired to on this board.
//
// Two deliberate exclusions, both of which crash or kill the board if touched:
//   GPIO 2  -- the power-off latch. Driving it HIGH cuts power (SKILL.md
//              Gotcha 8), so it is never reconfigured here.
//   GPIO 37 -- part of the OCTAL PSRAM bus. On ESP32-S3, GPIO 33-37 are
//              reserved when octal PSRAM is enabled (this board is 8 MB octal,
//              built with PSRAM=opi), so calling pinMode on 37 corrupts the
//              PSRAM interface and the chip reboots in a loop. Verified the
//              hard way: probing it produced a continuous boot loop where
//              every pin read 0. The "charge detect on GPIO 37" line in
//              SKILL.md and the board README therefore cannot be read in this
//              build configuration.
static Pin pins[] = {
  { "BOOT  g0",  PIN_BOOT_BUTTON },
  { "PWRKEY g3", PIN_POWER_KEY   },
  { "VOL-  g39", PIN_VOLUME_DOWN },
  { "VOL+  g40", PIN_VOLUME_UP   },
};
static constexpr int PIN_COUNT = sizeof(pins) / sizeof(pins[0]);

// Read each pin under all three input modes. A pin with an external pulldown
// reads 0 in plain INPUT and stays 0 with PULLDOWN; a floating pin follows
// whichever bias is applied, which is how you tell "not connected" apart from
// "connected and not pressed".
enum Mode { M_PLAIN = 0, M_PULLUP, M_PULLDOWN, MODE_COUNT };
static const char *modeName(int m) {
  return m == M_PLAIN ? "INPUT" : (m == M_PULLUP ? "PULLUP" : "PULLDOWN");
}
static int readUnder(int gpio, int mode) {
  pinMode(gpio, mode == M_PLAIN ? INPUT : (mode == M_PULLUP ? INPUT_PULLUP : INPUT_PULLDOWN));
  delayMicroseconds(600);   // let the bias settle before sampling
  return digitalRead(gpio);
}

static int baseline[PIN_COUNT][MODE_COUNT];

void setup() {
  Serial.begin(115200);

  // Gotcha 8 first, always.
  pinMode(PIN_POWER_OFF, OUTPUT);
  digitalWrite(PIN_POWER_OFF, LOW);

  SPI.begin(PIN_LCD_SCLK, -1, PIN_LCD_MOSI, PIN_LCD_CS);
  SPI.setFrequency(40000000);
  lcdInit();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  delay(400);
  Serial.println();
  Serial.println("=== Cheeko Gotchi button probe ===");
  Serial.println("Nothing pressed yet -- recording baseline levels.");

  for (int i = 0; i < PIN_COUNT; ++i) {
    Serial.printf("  %-11s", pins[i].name);
    for (int m = 0; m < MODE_COUNT; ++m) {
      baseline[i][m] = readUnder(pins[i].gpio, m);
      Serial.printf("  %s=%d", modeName(m), baseline[i][m]);
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("Now press each physical button, one at a time.");
  Serial.println("Any line printed below is a pin whose level CHANGED from baseline.");
  Serial.println("The pin that reacts to the middle button is the push-to-talk pin,");
  Serial.println("and the direction of change tells us its polarity.");
  Serial.println();

  // Leave every pin in plain INPUT for the monitoring loop: adding a pull
  // could fight the board's own external bias and produce a level that says
  // more about this sketch than about the hardware.
  for (int i = 0; i < PIN_COUNT; ++i) pinMode(pins[i].gpio, INPUT);

  lcdFillScreen(COLOR_BLACK);
  lcdDrawText(8, 8, "BUTTON PROBE", COLOR_CYAN, COLOR_BLACK, 2);
  lcdDrawText(8, 34, "PRESS EACH BUTTON", COLOR_WHITE, COLOR_BLACK, 1);
  lcdDrawText(8, 50, "WATCH SERIAL LOG", COLOR_GRAY, COLOR_BLACK, 1);
}

void loop() {
  static int last[PIN_COUNT] = { -1, -1, -1, -1 };
  static bool everChanged[PIN_COUNT] = { false, false, false, false };

  for (int i = 0; i < PIN_COUNT; ++i) {
    const int v = digitalRead(pins[i].gpio);
    if (v != last[i]) {
      last[i] = v;
      const int base = baseline[i][M_PLAIN];
      const bool active = (v != base);
      if (active) everChanged[i] = true;

      Serial.printf("%-11s -> %d  (baseline %d)  %s\n", pins[i].name, v, base,
                    active ? "<<< PRESSED" : "released");

      // Draw one row per pin so the board itself shows which pin is live.
      const int y = 80 + i * 30;
      lcdFillRect(0, y, LCD_WIDTH, 26, COLOR_BLACK);
      lcdDrawText(8, y + 4, pins[i].name, active ? COLOR_GREEN : COLOR_GRAY, COLOR_BLACK, 2);
      lcdDrawText(190, y + 4, v ? "1" : "0", active ? COLOR_GREEN : COLOR_GRAY, COLOR_BLACK, 2);
    }
  }

  // Touch too, so if the screen is the only working input we find that out here.
  static uint32_t lastTouchLog = 0;
  int rx, ry;
  if (touchReadRaw(rx, ry) && millis() - lastTouchLog > 400) {
    lastTouchLog = millis();
    Serial.printf("TOUCH raw=(%d,%d)\n", rx, ry);
  }

  delay(15);
}
