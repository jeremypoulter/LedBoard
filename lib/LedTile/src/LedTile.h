#pragma once

// LedTile: ESP32 driver for the LED Tile v2, a 64 x 16 RGB panel built from 48
// MBI5034 constant-current shift registers (see the project README for the
// reverse-engineered protocol).
//
// The panel itself has no brightness control, so brightness is produced by
// binary code modulation (BCM): each of the four address states is shown as a
// series of weighted bit planes, played out by I2S DMA with no CPU involvement.
// All seven panel signals, including the shift clock, are encoded as data bits
// in the DMA stream.
//
// Typical use:
//
//   LedTile tile;
//   tile.begin();                      // default ESP-WROVER-KIT pin map
//   tile.setPixel(3, 4, 255, 0, 0);
//   tile.show();                       // only changed pixels are re-rendered

#include <Arduino.h>

#include "i2s_parallel.h"

struct LedTileConfig {
  // Defaults match the ESP-WROVER-KIT wiring documented in the README. The LCD
  // on that board must be unplugged, as it shares most of these GPIOs.
  int8_t pinD1 = 26;   // serial data, top bank (rows 0-7)
  int8_t pinD2 = 18;   // serial data, bottom bank (rows 8-15)
  int8_t pinLat = 19;
  int8_t pinOe = 21;   // active low
  int8_t pinA0 = 22;
  int8_t pinA1 = 23;
  int8_t pinClk = 25;

  // Colour depth in bits per channel (1-8). Each extra bit adds a complete
  // shift+latch pass per address, so it costs RAM and refresh rate.
  uint8_t bcmBits = 8;
  // Display length of the shortest bit plane, in panel clock cycles. Together
  // with bcmBits this sets maximum brightness: the 390-cycle shift overhead is
  // paid per plane, but only the display windows light the panel.
  uint16_t baseCycles = 4;
  // Display length of the extra "floor" plane that lights values the gamma
  // curve would otherwise round to fully off. Must be < baseCycles.
  uint16_t floorCycles = 2;

  uint32_t clockHz = 2000000;  // panel shift clock
  uint8_t i2sPort = 1;         // 1 (proven) or 0 (untested)

  Print *log = nullptr;        // optional sink for boot diagnostics
};

class LedTile {
 public:
  static constexpr uint8_t kWidth = 64;
  static constexpr uint8_t kHeight = 16;
  static constexpr uint8_t kPinCount = 7;

  enum Transport { TRANSPORT_DMA, TRANSPORT_BITBANG };

  LedTile() {}
  ~LedTile() { end(); }
  LedTile(const LedTile &) = delete;
  LedTile &operator=(const LedTile &) = delete;

  // Allocates the DMA waveform, self-checks it and starts output. Returns false
  // (see lastError()) on bad config or allocation failure; nothing is left
  // allocated in that case.
  bool begin(const LedTileConfig &cfg = LedTileConfig());
  // Stops output, blanks the panel, releases the pins and frees all memory.
  void end();
  bool running() const { return _running; }

  // ---- pixels. Writes go to a framebuffer; show() renders what changed. ----
  void setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);
  void setPixel(uint8_t x, uint8_t y, uint32_t rgb) {
    setPixel(x, y, (uint8_t)(rgb >> 16), (uint8_t)(rgb >> 8), (uint8_t)rgb);
  }
  uint32_t getPixel(uint8_t x, uint8_t y) const;
  // Blanks the framebuffer; the panel follows on the next show(), which only
  // rewrites the pixels that were lit, so redrawing every frame does not flicker.
  void clear();
  void show();

  // ---- appearance. Changing these re-renders every pixel on the next show().
  void setBrightness(uint8_t b);
  uint8_t brightness() const { return _bri; }
  // Perceptual (CIE 1931) curve blended with linear: strength 0 = linear,
  // 1 = full curve. Disabling is equivalent to strength 0.
  void setGamma(bool enabled);
  bool gammaEnabled() const { return _gammaOn; }
  void setGammaStrength(float strength);
  float gammaStrength() const { return _gammaStrength; }

  // ---- timing ----
  bool setClockHz(uint32_t hz);  // restarts the DMA stream
  uint32_t clockHz() const { return _cfg.clockHz; }
  uint32_t refreshHz() const;    // whole-panel refresh; 0 when bit-banging

  // ---- transport. Bit-bang replays the same waveform through plain GPIO and
  // is only for debugging; call service() regularly while it is selected.
  void setTransport(Transport t);
  Transport transport() const { return _transport; }
  void service();

  // ---- info ----
  // Fills out[kPinCount] with GPIO numbers (D1, D2, LAT, OE, A0, A1, CLK) so a
  // host framework can reserve them.
  uint8_t getPins(uint8_t *out) const;
  uint8_t bcmBits() const { return _cfg.bcmBits; }
  uint32_t levels() const { return 1u << _cfg.bcmBits; }
  size_t waveformBytes() const;
  const char *lastError() const { return _error; }
  bool dmaStatus(I2SParallelStatus *out) const;
  bool waitForFrame(uint32_t timeoutUs);

  // ---- raw diagnostics: bypass the framebuffer and light a stream bit on
  // every plane (full brightness). Used by the geometry test patterns. Raw bits
  // are invisible to the framebuffer, so call rawClear() (which blanks the panel
  // immediately) before switching back to setPixel().
  void rawClear();
  void rawSetWord(uint8_t addr, uint8_t bank, uint8_t group, uint8_t colour,
                  uint16_t bits);
  void rawFillCycles(uint8_t addr, int count);

 private:
  struct PixelSlot {
    uint8_t addr, bank, group, halfIdx, bitInByte;
  };
  static bool slot(uint8_t x, uint8_t y, PixelSlot *out);

  static constexpr int kMaxPlanes = 9;  // 8 bit planes + floor plane
  static constexpr int kPixels = kWidth * kHeight;

  int planeWeight(int p) const;
  void logf(const char *fmt, ...) const;
  bool setError(const char *msg);
  void freeWaveform();
  void buildWaveform();
  bool validateWaveform() const;
  void buildLut();
  void markAllDirty() { _allDirty = true; }
  void renderPixel(uint8_t x, uint8_t y);
  void setStreamBit(uint8_t addr, int plane, uint8_t bank, uint8_t byteIdx,
                    uint8_t bitInByte, bool on);
  bool startI2S();
  void stopI2S();
  void releasePinsToGpio();
  void bitbangFrame();

  LedTileConfig _cfg;
  bool _running = false;
  bool _i2sOn = false;
  Transport _transport = TRANSPORT_DMA;
  const char *_error = "";

  uint32_t *_wave[4] = {nullptr, nullptr, nullptr, nullptr};
  int _planes = 0;              // bcmBits + 1 (the floor plane is last)
  int _planeStart[kMaxPlanes];  // cycle offset of each plane within an address
  int _addrBlock = 0;           // cycles per address
  size_t _wordsPerAddr = 0;     // uint32 words per address chunk

  uint8_t _bri = 255;
  bool _gammaOn = true;
  float _gammaStrength = 0.5f;
  uint16_t _lut[256];           // input value -> BCM level

  uint8_t _fb[kPixels * 3];     // r, g, b per pixel
  uint8_t _dirty[kPixels / 8];
  bool _allDirty = false;
};
