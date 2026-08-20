// Tama capture firmware for the Cheeko Gotchi board (OSTB_XIAOZHI_V1.2).
//
// Hold the middle button, talk, release — the recording POSTs to tama-server's
// /capture endpoint exactly like the iPhone Shortcut does. No wake word, no
// on-device model, no reply audio: this device only ever does the free half
// of Tama (see architecture.md section 1 — "capture is a strict prefix of ask").
//
// Pin map, display init, ES7210 register sequence and I2S config below are
// reused verbatim from raviramp36/cheeko-gotchi's cheeko_hw_self_test.ino and
// the vendor's SKILL.md — both proven working on this exact board. Only the
// WiFi/HTTP/WAV-framing/record-state-machine code below is new.
//
// Before flashing: copy secrets.h.example to secrets.h and fill in your
// WiFi credentials, server address, and device token.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <esp_idf_version.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#include "secrets.h"

// ---------------------------------------------------------------- pin map
// Source of truth: SKILL.md section 2, cross-confirmed by
// raviramp36/cheeko-gotchi's own README pin table and by hardware.md.
static constexpr int PIN_BOOT_BUTTON = 0;
static constexpr int PIN_TOUCH_IRQ   = 1;
static constexpr int PIN_POWER_OFF   = 2;  // OUTPUT: HIGH cuts power. Never touch except deliberately.
static constexpr int PIN_POWER_KEY   = 3;  // push-to-talk button. Active HIGH, external pulldown.
static constexpr int PIN_PA_CTRL     = 4;
static constexpr int PIN_I2S_MCLK    = 5;
static constexpr int PIN_I2S_DOUT    = 6;
static constexpr int PIN_I2S_DIN     = 7;
static constexpr int PIN_LCD_DC      = 8;
static constexpr int PIN_LCD_SCLK    = 9;
static constexpr int PIN_LCD_MOSI    = 10;
static constexpr int PIN_I2C_SCL     = 11;
static constexpr int PIN_I2C_SDA     = 12;
static constexpr int PIN_LCD_BL      = 13;
static constexpr int PIN_LCD_CS      = 14;
static constexpr int PIN_I2S_BCLK    = 15;
static constexpr int PIN_I2S_LRCK    = 16;
static constexpr int PIN_LCD_RST     = 17;
static constexpr int PIN_VOLUME_DOWN = 39;
static constexpr int PIN_VOLUME_UP   = 40;

static constexpr uint8_t ES8311_ADDR = 0x18;
static constexpr uint8_t ES7210_ADDR = 0x40;

// UNVERIFIED on real hardware, and it fails silently if wrong. hardware.md
// trap 4: board.json claims the ES7210 runs at 24 kHz, while the proven
// bring-up I2S config (and this one) clocks 16 kHz with MCLK = 256 x 16000.
// Whisper wants 16 kHz. If the ADC is actually delivering 24 kHz while the
// header below declares 16000, playback is 1.5x slow and pitched down, and the
// transcript comes back as garbage that looks like a whisper problem rather
// than a clock problem. Verify by recording a known-length phrase and checking
// the audioSeconds the server reports against a stopwatch.
static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr uint32_t AUDIO_MCLK_HZ     = 4096000;  // 256 * 16kHz
// Also unverified: the ES7210 is a 4-channel ADC whose MIC3 is a speaker
// loopback for AEC. The proven self-test reads it as plain 2-channel
// standard-I2S and its level meter worked, so 2 is the evidence-backed choice,
// but which physical mics land in L and R is not established. If transcripts
// come back polluted, this is a prime suspect.
static constexpr int AUDIO_CHANNELS         = 2;        // I2S delivers L+R frames; server downmixes.

static constexpr int LCD_WIDTH  = 240;
static constexpr int LCD_HEIGHT = 296;

static constexpr uint16_t COLOR_BLACK = 0x0000;
static constexpr uint16_t COLOR_WHITE = 0xffff;
static constexpr uint16_t COLOR_GREEN = 0x07e0;
static constexpr uint16_t COLOR_RED   = 0xf800;
static constexpr uint16_t COLOR_AMBER = 0xfd20;

static constexpr int MAX_RECORD_SECONDS = 8;
static constexpr size_t RECORD_BUFFER_BYTES =
    (size_t)MAX_RECORD_SECONDS * AUDIO_SAMPLE_RATE * AUDIO_CHANNELS * sizeof(int16_t);

// ---------------------------------------------------------------- display
// Reused verbatim from cheeko_hw_self_test.ino (SPI primitives + tiny 5x7 font).

static void lcdWriteCommand(uint8_t command) {
  digitalWrite(PIN_LCD_DC, LOW);
  digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(command);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static void lcdWriteData(uint8_t data) {
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  SPI.write(data);
  digitalWrite(PIN_LCD_CS, HIGH);
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

// Bulk-write in row-sized chunks rather than two SPI.write() calls per pixel.
// A full screen is 71,040 pixels; per-pixel writes made lcdFillScreen take
// hundreds of milliseconds, which is long enough to overrun the I2S capture
// ring mid-recording and punch holes in the audio. SKILL.md section 4 says the
// same thing ("SPI.writeBytes(buf, n) is far faster than a per-pixel loop").
static void lcdFillRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0 || y < 0 || x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
  w = min(w, LCD_WIDTH - x);
  h = min(h, LCD_HEIGHT - y);
  if (w <= 0 || h <= 0) return;

  static uint8_t rowBuf[LCD_WIDTH * 2];
  const int span = min(w, LCD_WIDTH);
  for (int i = 0; i < span; ++i) {
    rowBuf[i * 2]     = color >> 8;
    rowBuf[i * 2 + 1] = color & 0xff;
  }

  lcdSetWindow(x, y, x + w - 1, y + h - 1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
  for (int row = 0; row < h; ++row) SPI.writeBytes(rowBuf, span * 2);
  digitalWrite(PIN_LCD_CS, HIGH);
}

static void lcdFillScreen(uint16_t color) { lcdFillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, color); }

static const uint8_t *glyphForChar(char c) {
  static const uint8_t glyphs[][5] = {
      {0x00, 0x00, 0x00, 0x00, 0x00}, {0x7e, 0x11, 0x11, 0x11, 0x7e},
      {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
      {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41},
      {0x7f, 0x09, 0x09, 0x09, 0x01}, {0x3e, 0x41, 0x49, 0x49, 0x7a},
      {0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
      {0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41},
      {0x7f, 0x40, 0x40, 0x40, 0x40}, {0x7f, 0x02, 0x0c, 0x02, 0x7f},
      {0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
      {0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e},
      {0x7f, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
      {0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
      {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x7f, 0x20, 0x18, 0x20, 0x7f},
      {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x70, 0x08, 0x07},
      {0x61, 0x51, 0x49, 0x45, 0x43}, {0x3e, 0x51, 0x49, 0x45, 0x3e},
      {0x00, 0x42, 0x7f, 0x40, 0x00}, {0x62, 0x51, 0x49, 0x49, 0x46},
      {0x22, 0x41, 0x49, 0x49, 0x36}, {0x18, 0x14, 0x12, 0x7f, 0x10},
      {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3c, 0x4a, 0x49, 0x49, 0x30},
      {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
      {0x06, 0x49, 0x49, 0x29, 0x1e}, {0x40, 0x30, 0x0c, 0x03, 0x00},
      {0x00, 0x40, 0x40, 0x40, 0x00}, {0x00, 0x60, 0x60, 0x00, 0x00},
      {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x36, 0x36, 0x00, 0x00},
      {0x08, 0x14, 0x22, 0x41, 0x00}, {0x41, 0x22, 0x14, 0x08, 0x00},
      {0x14, 0x08, 0x3e, 0x08, 0x14}, {0x08, 0x08, 0x3e, 0x08, 0x08},
  };
  if (c >= 'A' && c <= 'Z') return glyphs[1 + c - 'A'];
  if (c >= 'a' && c <= 'z') return glyphs[1 + c - 'a'];
  if (c >= '0' && c <= '9') return glyphs[27 + c - '0'];
  switch (c) {
    case ' ': return glyphs[0];
    case '/': return glyphs[37];
    case '_': return glyphs[38];
    case '.': return glyphs[39];
    case '-': return glyphs[40];
    case ':': return glyphs[41];
    default:  return glyphs[0];  // unmapped punctuation renders blank, not as
                                 // some unrelated glyph
  }
}

static void lcdDrawChar(int x, int y, char c, uint16_t color, uint16_t bg, int scale) {
  const uint8_t *glyph = glyphForChar(c);
  for (int col = 0; col < 5; ++col) {
    uint8_t bits = glyph[col];
    for (int row = 0; row < 7; ++row) {
      lcdFillRect(x + col * scale, y + row * scale, scale, scale, (bits & (1 << row)) ? color : bg);
    }
  }
}

static void lcdDrawTextCentered(int y, const char *text, uint16_t color, uint16_t bg, int scale) {
  // Truncate to what actually fits rather than letting x go negative — every
  // lcdFillRect below would then hit its x<0 guard and the whole line would
  // render as blank screen with no hint anything was wrong.
  const int charW = 6 * scale;
  const int maxChars = LCD_WIDTH / charW;
  int len = (int)strlen(text);
  if (len > maxChars) len = maxChars;

  int x = (LCD_WIDTH - len * charW) / 2;
  if (x < 0) x = 0;
  for (int i = 0; i < len; ++i) { lcdDrawChar(x, y, text[i], color, bg, scale); x += charW; }
}

static void lcdInit() {
  pinMode(PIN_LCD_CS, OUTPUT);
  pinMode(PIN_LCD_DC, OUTPUT);
  pinMode(PIN_LCD_RST, OUTPUT);
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_CS, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
  digitalWrite(PIN_LCD_RST, LOW);  delay(20);
  digitalWrite(PIN_LCD_RST, HIGH); delay(120);

  lcdWriteCommand(0x01); delay(150);  // SWRESET
  lcdWriteCommand(0x11); delay(120);  // SLPOUT
  lcdWriteCommand(0x36);
  // NOTE: the two proven references disagree here. SKILL.md says 0x40 (MX only);
  // cheeko_hw_self_test.ino (the more specific, later reference) uses 0x48 (MX+BGR).
  // Going with 0x48 to match the working self-test tool. If colours look swapped
  // on real hardware, this is the first thing to try flipping — see SKILL.md
  // Gotcha 2 for the fill-screen-solid-colour diagnostic.
  lcdWriteData(0x48);
  lcdWriteCommand(0x3a); lcdWriteData(0x55);  // COLMOD, 16-bit RGB565
  lcdWriteCommand(0x20);                       // INVOFF — this panel does not want inversion
  lcdWriteCommand(0x13);                       // NORON
  lcdWriteCommand(0x29);                       // DISPON
}

// ---------------------------------------------------------------- I2C + codecs

static bool i2cWriteReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool es8311Init() {
  bool ok = true;
  ok &= i2cWriteReg(ES8311_ADDR, 0x00, 0x1f); delay(20);
  ok &= i2cWriteReg(ES8311_ADDR, 0x00, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x00, 0x80);
  ok &= i2cWriteReg(ES8311_ADDR, 0x01, 0x3f);
  ok &= i2cWriteReg(ES8311_ADDR, 0x02, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x03, 0x10);
  ok &= i2cWriteReg(ES8311_ADDR, 0x04, 0x10);
  ok &= i2cWriteReg(ES8311_ADDR, 0x05, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x06, 0x03);
  ok &= i2cWriteReg(ES8311_ADDR, 0x07, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x08, 0xff);
  ok &= i2cWriteReg(ES8311_ADDR, 0x09, 0x0c);
  ok &= i2cWriteReg(ES8311_ADDR, 0x0a, 0x0c);
  ok &= i2cWriteReg(ES8311_ADDR, 0x0d, 0x01);
  ok &= i2cWriteReg(ES8311_ADDR, 0x0e, 0x02);
  ok &= i2cWriteReg(ES8311_ADDR, 0x12, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x13, 0x10);
  ok &= i2cWriteReg(ES8311_ADDR, 0x1c, 0x6a);
  ok &= i2cWriteReg(ES8311_ADDR, 0x31, 0x00);
  ok &= i2cWriteReg(ES8311_ADDR, 0x32, 0xbf);
  ok &= i2cWriteReg(ES8311_ADDR, 0x37, 0x08);
  return ok;
}

// ES7210 is the only codec Tama actually reads from — ES8311's mic inputs are
// not wired on this board (SKILL.md section 1). es8311Init() above is still
// called because both codecs share one I2S bus (see audioInit()); trimming it
// is a safe later simplification once real hardware confirms RX-only mode
// works standalone, but for a first build, matching the exact proven
// full-duplex configuration removes a whole class of "what did I change"
// debugging if bring-up doesn't work first try.
static bool es7210Init() {
  bool ok = true;
  ok &= i2cWriteReg(ES7210_ADDR, 0x00, 0xff);
  ok &= i2cWriteReg(ES7210_ADDR, 0x00, 0x41);
  ok &= i2cWriteReg(ES7210_ADDR, 0x01, 0x3f);
  ok &= i2cWriteReg(ES7210_ADDR, 0x09, 0x30);
  ok &= i2cWriteReg(ES7210_ADDR, 0x0a, 0x30);
  ok &= i2cWriteReg(ES7210_ADDR, 0x23, 0x2a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x22, 0x0a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x20, 0x0a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x21, 0x2a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x08, 0x00);
  ok &= i2cWriteReg(ES7210_ADDR, 0x40, 0x43);
  ok &= i2cWriteReg(ES7210_ADDR, 0x41, 0x70);
  ok &= i2cWriteReg(ES7210_ADDR, 0x42, 0x70);
  ok &= i2cWriteReg(ES7210_ADDR, 0x07, 0x20);
  ok &= i2cWriteReg(ES7210_ADDR, 0x02, 0xc1);
  ok &= i2cWriteReg(ES7210_ADDR, 0x11, 0x60);
  ok &= i2cWriteReg(ES7210_ADDR, 0x12, 0x00);
  ok &= i2cWriteReg(ES7210_ADDR, 0x01, 0x34);
  ok &= i2cWriteReg(ES7210_ADDR, 0x06, 0x00);
  ok &= i2cWriteReg(ES7210_ADDR, 0x47, 0x08);
  ok &= i2cWriteReg(ES7210_ADDR, 0x48, 0x08);
  ok &= i2cWriteReg(ES7210_ADDR, 0x4b, 0x00);
  ok &= i2cWriteReg(ES7210_ADDR, 0x4c, 0xff);
  ok &= i2cWriteReg(ES7210_ADDR, 0x43, 0x1a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x44, 0x1a);
  ok &= i2cWriteReg(ES7210_ADDR, 0x40, 0x43);
  ok &= i2cWriteReg(ES7210_ADDR, 0x00, 0x71);
  ok &= i2cWriteReg(ES7210_ADDR, 0x00, 0x41);
  return ok;
}

static bool audioInit() {
  // Amp enabled only for the codec init handshake, then dropped again below.
  // This firmware never plays audio, and MIC3 on the ES7210 is a speaker
  // loopback for AEC (SKILL.md section 1) — leaving a 3 W class-D amp live
  // would idle its noise floor straight into the capture path, on a
  // battery-powered device, for no benefit.
  pinMode(PIN_PA_CTRL, OUTPUT);
  digitalWrite(PIN_PA_CTRL, HIGH);

  i2s_config_t cfg = {};
  cfg.mode                 = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate          = AUDIO_SAMPLE_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = 0;
  // The reference self-test used 4x128 frames = 32 ms of ring. That is enough
  // for a tool whose loop does nothing else, but this firmware also repaints a
  // screen and runs a network stack, and anything that stalls the loop past the
  // ring length silently overwrites captured samples with no error surfaced
  // (i2s_read cannot report an overrun). 8x256 frames = 128 ms buys real
  // headroom for ~8 KB of DMA memory.
  cfg.dma_buf_count        = 8;
  cfg.dma_buf_len          = 256;
  cfg.use_apll             = false;
  cfg.tx_desc_auto_clear   = true;
  cfg.fixed_mclk           = AUDIO_MCLK_HZ;

  i2s_pin_config_t pins = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
  pins.mck_io_num = PIN_I2S_MCLK;
#endif
  pins.bck_io_num   = PIN_I2S_BCLK;
  pins.ws_io_num    = PIN_I2S_LRCK;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num  = PIN_I2S_DIN;

  i2s_driver_uninstall(I2S_NUM_0);
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr) != ESP_OK) return false;
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_NUM_0);

  bool dac_ok = es8311Init();
  bool adc_ok = es7210Init();
  Serial.printf("ES8311 init: %s, ES7210 init: %s\n", dac_ok ? "ok" : "FAILED", adc_ok ? "ok" : "FAILED");

  digitalWrite(PIN_PA_CTRL, LOW);  // amp back off — see comment at the top of this function
  return adc_ok;  // only the mic path is load-bearing for capture
}

// ---------------------------------------------------------------- WAV framing

// Wraps raw PCM in a minimal 44-byte RIFF/WAVE header so tama-server's ffmpeg
// pipeline can decode it without guessing the format. Server already handles
// arbitrary channel counts and sample rates (audio.ts normalises to 16kHz
// mono), so this device does zero audio processing of its own.
static size_t writeWavHeader(uint8_t *out, uint32_t dataBytes, uint32_t sampleRate, uint16_t channels) {
  uint32_t byteRate   = sampleRate * channels * sizeof(int16_t);
  uint16_t blockAlign = channels * sizeof(int16_t);
  uint32_t riffSize   = 36 + dataBytes;

  size_t i = 0;
  memcpy(out + i, "RIFF", 4); i += 4;
  memcpy(out + i, &riffSize, 4); i += 4;
  memcpy(out + i, "WAVE", 4); i += 4;
  memcpy(out + i, "fmt ", 4); i += 4;
  uint32_t fmtSize = 16; memcpy(out + i, &fmtSize, 4); i += 4;
  uint16_t audioFormat = 1; memcpy(out + i, &audioFormat, 2); i += 2;  // PCM
  memcpy(out + i, &channels, 2); i += 2;
  memcpy(out + i, &sampleRate, 4); i += 4;
  memcpy(out + i, &byteRate, 4); i += 4;
  memcpy(out + i, &blockAlign, 2); i += 2;
  uint16_t bitsPerSample = 16; memcpy(out + i, &bitsPerSample, 2); i += 2;
  memcpy(out + i, "data", 4); i += 4;
  memcpy(out + i, &dataBytes, 4); i += 4;
  return i;  // always 44
}

// ---------------------------------------------------------------- state machine

enum class AppState { Idle, Recording, Sending, Result };
static AppState state = AppState::Idle;

// The WAV header lives in the same buffer as the PCM, at a reserved 44-byte
// prefix, so the whole thing is one contiguous region and can go out in a
// single HTTPClient::POST(buf, len) call — no second full-size copy to merge
// a header and a separate PCM buffer together.
static constexpr size_t WAV_HEADER_BYTES = 44;
static uint8_t *recordBuffer = nullptr;   // PSRAM: WAV_HEADER_BYTES + RECORD_BUFFER_BYTES
static size_t recordedBytes = 0;
static uint32_t recordStartedAtMs = 0;
static uint32_t resultShownAtMs = 0;
static char resultLine[40] = "";
static uint16_t resultColor = COLOR_WHITE;
static int lastShownSec = -1;

// Idempotency keys must stay unique across power cycles, and this board has a
// one-key on/off switch so power cycling is routine, not rare. millis() alone
// resets to 0 every boot: two captures taken at a similar offset after two
// different boots would collide, and the server would replay the first note's
// response while silently discarding the second recording (tama-server's
// idempotency.ts treats a seen key as a duplicate by design). A counter in NVS
// survives reboot and OTA, so every capture this device ever makes gets its own
// key.
static Preferences prefs;
static uint32_t captureSeq = 0;

static uint32_t nextCaptureSeq() {
  captureSeq = prefs.getUInt("seq", 0) + 1;
  prefs.putUInt("seq", captureSeq);
  return captureSeq;
}

static void drawIdle() {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(120, "TAMA", COLOR_WHITE, COLOR_BLACK, 3);
  lcdDrawTextCentered(170, "HOLD TO TALK", COLOR_AMBER, COLOR_BLACK, 1);
}

// Split in two on purpose: the full-screen clear and the static label are
// drawn once when recording starts, and each per-second tick repaints only the
// small timer strip. A full repaint every second is long enough to overrun the
// I2S ring and drop audio, which is the one thing this device must not do.
static void drawRecordingStatic() {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(120, "RECORDING", COLOR_RED, COLOR_BLACK, 2);
}

static void drawRecordingTimer(int elapsedSec) {
  char line[16];
  snprintf(line, sizeof(line), "%ds OF %ds", elapsedSec, MAX_RECORD_SECONDS);
  lcdFillRect(0, 160, LCD_WIDTH, 8, COLOR_BLACK);  // just the timer strip
  lcdDrawTextCentered(160, line, COLOR_WHITE, COLOR_BLACK, 1);
}

static void drawSending() {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(140, "SENDING...", COLOR_AMBER, COLOR_BLACK, 2);
}

static void drawResult() {
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(140, resultLine, resultColor, COLOR_BLACK, 2);
}

// Active HIGH with an external pulldown on this board (SKILL.md Gotcha 4).
static bool powerKeyPressed() { return digitalRead(PIN_POWER_KEY) == HIGH; }

// Set once the button has been observed released, so one press produces
// exactly one recording. See the Idle case in loop().
static bool armed = true;

static void startRecording() {
  recordedBytes = 0;
  lastShownSec = -1;  // file-scope, so a previous sub-second capture cannot
                      // suppress the first timer paint of the next recording
  recordStartedAtMs = millis();
  i2s_zero_dma_buffer(I2S_NUM_0);
  state = AppState::Recording;
  drawRecordingStatic();
  drawRecordingTimer(0);
}

static void continueRecording() {
  size_t bytesRead = 0;
  size_t remaining = RECORD_BUFFER_BYTES - recordedBytes;
  if (remaining == 0) { stopRecordingAndSend(); return; }

  // Keep each read well inside the DMA ring (128 ms) so a read never has to
  // wait on samples that do not exist yet.
  size_t chunk = min(remaining, (size_t)2048);
  if (i2s_read(I2S_NUM_0, recordBuffer + WAV_HEADER_BYTES + recordedBytes, chunk, &bytesRead,
               pdMS_TO_TICKS(50)) == ESP_OK) {
    recordedBytes += bytesRead;
  }

  int elapsedSec = recordedBytes / (AUDIO_SAMPLE_RATE * AUDIO_CHANNELS * sizeof(int16_t));
  if (elapsedSec != lastShownSec) { drawRecordingTimer(elapsedSec); lastShownSec = elapsedSec; }

  if (!powerKeyPressed() || recordedBytes >= RECORD_BUFFER_BYTES) {
    stopRecordingAndSend();
  }
}

static void showResult(const char *text, uint16_t color) {
  strncpy(resultLine, text, sizeof(resultLine) - 1);
  resultColor = color;
  resultShownAtMs = millis();
  state = AppState::Result;
  drawResult();
}

static void stopRecordingAndSend() {
  state = AppState::Sending;
  drawSending();

  if (recordedBytes < AUDIO_SAMPLE_RATE * AUDIO_CHANNELS * sizeof(int16_t) / 4) {
    showResult("TOO SHORT", COLOR_RED);
    return;
  }

  // Backfill the WAV header into the reserved prefix now that recordedBytes
  // is final. Header + PCM are already one contiguous buffer, so this is a
  // single POST with no extra copy.
  writeWavHeader(recordBuffer, recordedBytes, AUDIO_SAMPLE_RATE, AUDIO_CHANNELS);
  size_t totalLen = WAV_HEADER_BYTES + recordedBytes;

  // One key for this recording, reused across every retry below. That is the
  // whole point of the header: a retry after a lost response must replay, not
  // write a second note.
  char idemKey[64];
  snprintf(idemKey, sizeof(idemKey), "%s-%lu", WiFi.macAddress().c_str(),
           (unsigned long)nextCaptureSeq());

  // tama-server's documented retry policy (README "Client retry policy"):
  // 401 -> stop, never retry. 413/422 -> drop, tell the user. 429/503/5xx and
  // timeouts -> retry with backoff, same key. Anything 2xx -> done.
  const int MAX_ATTEMPTS = 4;
  int backoffMs = 400;
  int httpCode = 0;

  for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
    HTTPClient http;
    String url = String("http://") + TAMA_SERVER_HOST + ":" + String(TAMA_SERVER_PORT) + "/capture";
    http.begin(url);
    http.setTimeout(20000);  // whisper transcription happens inside this request
    http.addHeader("Content-Type", "audio/wav");
    http.addHeader("Authorization", String("Bearer ") + TAMA_DEVICE_TOKEN);
    http.addHeader("Idempotency-Key", idemKey);

    // No RTC on this board — send how long ago the speech happened rather than
    // pretending to know wall-clock time. Measured from the START of the
    // recording and evaluated per attempt, so upload and retry time do not
    // push the note into the wrong day (architecture.md section 4, bug 1).
    http.addHeader("X-Tama-Captured-Age-Ms", String((unsigned long)(millis() - recordStartedAtMs)));

    httpCode = http.POST(recordBuffer, totalLen);
    Serial.printf("POST /capture attempt %d -> %d\n", attempt, httpCode);

    if (httpCode == 200) {
      String body = http.getString();
      Serial.printf("  %s\n", body.c_str());
      http.end();
      showResult("SAVED", COLOR_GREEN);
      return;
    }

    // Terminal: retrying cannot help and would only burn battery.
    if (httpCode == 401) { http.end(); showResult("UNAUTHORIZED", COLOR_RED); return; }
    if (httpCode == 413) { http.end(); showResult("TOO LARGE", COLOR_RED); return; }
    if (httpCode == 422) { http.end(); showResult("NO SPEECH", COLOR_AMBER); return; }

    http.end();

    if (attempt < MAX_ATTEMPTS) {
      char line[24];
      snprintf(line, sizeof(line), "RETRY %d", attempt + 1);
      lcdFillRect(0, 180, LCD_WIDTH, 8, COLOR_BLACK);
      lcdDrawTextCentered(180, line, COLOR_AMBER, COLOR_BLACK, 1);
      delay(backoffMs);
      backoffMs *= 2;
    }
  }

  // Out of attempts. The recording is genuinely lost here — a flash-backed
  // queue that survives this is real work and belongs in v2's offline-queue
  // story (hardware.md), not bolted on now. Say so on screen rather than
  // showing a green SAVED that did not happen.
  if (httpCode > 0) {
    char line[24];
    snprintf(line, sizeof(line), "FAILED %d", httpCode);
    showResult(line, COLOR_RED);
  } else {
    showResult("NO CONNECTION", COLOR_RED);
  }
}

// ---------------------------------------------------------------- setup/loop

void setup() {
  Serial.begin(115200);

  // Gotcha 8: this must be driven LOW immediately, or the board can power
  // itself off. Do this before anything else.
  pinMode(PIN_POWER_OFF, OUTPUT);
  digitalWrite(PIN_POWER_OFF, LOW);

  pinMode(PIN_POWER_KEY, INPUT);  // active HIGH, external pulldown already on the board

  SPI.begin(PIN_LCD_SCLK, -1, PIN_LCD_MOSI, PIN_LCD_CS);  // no MISO on this bus
  SPI.setFrequency(40000000);
  lcdInit();
  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(140, "BOOTING...", COLOR_WHITE, COLOR_BLACK, 2);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  prefs.begin("tama", false);  // NVS namespace for the capture sequence counter

  if (!audioInit()) {
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(140, "MIC INIT FAILED", COLOR_RED, COLOR_BLACK, 1);
    Serial.println("halting: es7210 init failed, check I2C wiring");
    while (true) delay(1000);
  }

  recordBuffer = (uint8_t *)heap_caps_malloc(WAV_HEADER_BYTES + RECORD_BUFFER_BYTES, MALLOC_CAP_SPIRAM);
  if (!recordBuffer) {
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(140, "PSRAM ALLOC FAILED", COLOR_RED, COLOR_BLACK, 1);
    Serial.println("halting: PSRAM allocation failed, check PSRAM=opi build flag");
    while (true) delay(1000);
  }

  lcdFillScreen(COLOR_BLACK);
  lcdDrawTextCentered(140, "CONNECTING WIFI", COLOR_WHITE, COLOR_BLACK, 1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) delay(250);

  if (WiFi.status() != WL_CONNECTED) {
    lcdFillScreen(COLOR_BLACK);
    lcdDrawTextCentered(140, "WIFI FAILED", COLOR_RED, COLOR_BLACK, 2);
    Serial.println("halting: could not join WiFi, check secrets.h");
    while (true) delay(1000);
  }

  Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  drawIdle();
}

void loop() {
  switch (state) {
    case AppState::Idle:
      // Require a real press edge: the button must have been seen released
      // since the last capture. Without this, holding past the 8 s cap ends
      // one recording and immediately starts another, chopping a single
      // utterance into several notes with the upload window cut out of each.
      if (powerKeyPressed()) {
        if (armed) { armed = false; startRecording(); }
      } else {
        armed = true;
      }
      delay(10);
      break;
    case AppState::Recording:
      continueRecording();
      break;
    case AppState::Sending:
      // stopRecordingAndSend() runs synchronously and transitions state itself.
      break;
    case AppState::Result:
      if (millis() - resultShownAtMs > 2000) { state = AppState::Idle; drawIdle(); }
      delay(10);
      break;
  }
}
