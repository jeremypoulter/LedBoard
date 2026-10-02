#include "LedTile.h"

#include <esp_heap_caps.h>
#include <math.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include <stdarg.h>
#include <string.h>

namespace {

// Bit assignments on the 16-bit I2S bus. The order must match the pin array
// handed to i2sParallelBegin().
const uint16_t BUS_D1 = 1u << 0;
const uint16_t BUS_D2 = 1u << 1;
const uint16_t BUS_LAT = 1u << 2;
const uint16_t BUS_OE = 1u << 3;  // active low at the panel: a SET bit blanks
const uint16_t BUS_A0 = 1u << 4;
const uint16_t BUS_A1 = 1u << 5;
const uint16_t BUS_CLK = 1u << 6;

// 48 chips x 16 bits = 768 outputs; split over two banks that is 384 bits per
// data line per address, shifted as 8 groups of [B:16, G:16, R:16].
const int SHIFT_CYCLES = 384;
// Keep CLK low before, during and after LAT.
const int LATCH_CYCLES = 6;
// Each shift cycle is encoded as CLK low, low, high, high.
const int SAMPLES_PER_CYCLE = 4;
const int ADDR_COUNT = 4;

const uint8_t BANK_TOP = 0;

// Both halfwords of every waveform word are identical, so the I2S sample-pair
// ordering cannot change the edges the panel sees.
inline uint32_t dupHalf(uint16_t v) { return (uint32_t)v | ((uint32_t)v << 16); }

}  // namespace

// ------------------------------------------------------------------ helpers

void LedTile::logf(const char *fmt, ...) const {
  if (!_cfg.log) return;
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  _cfg.log->print(buf);
}

bool LedTile::setError(const char *msg) {
  _error = msg;
  logf("LedTile: %s\n", msg);
  return false;
}

int LedTile::planeWeight(int p) const {
  return (p < _cfg.bcmBits) ? ((int)_cfg.baseCycles << p) : (int)_cfg.floorCycles;
}

// Pixel -> stream position. Every step is confirmed on hardware:
//   bank      = y / 8           D1 drives rows 0-7, D2 drives rows 8-15
//   rowInBank = y % 8
//   address   = rowInBank % 4   each address serves rowInBank and rowInBank + 4
//   byte      low byte -> the higher row, high byte -> the lower row
//   group     = x / 8           group 0 is leftmost
//   bit       = 7 - (x % 8)     higher bits sit further left
bool LedTile::slot(uint8_t x, uint8_t y, PixelSlot *out) {
  if (x >= kWidth || y >= kHeight) return false;
  const uint8_t rowInBank = y % 8;
  const bool higherRow = rowInBank >= 4;
  uint8_t bit = 7 - (x % 8);
  if (!higherRow) bit += 8;
  out->addr = rowInBank % 4;
  out->bank = y / 8;
  out->group = x / 8;
  out->halfIdx = (bit >= 8) ? 0 : 1;  // 0 = high byte, sent first
  out->bitInByte = bit & 7;
  return true;
}

// ----------------------------------------------------------------- lifecycle

bool LedTile::begin(const LedTileConfig &cfg) {
  if (_running) end();
  _cfg = cfg;
  _error = "";

  if (cfg.bcmBits < 1 || cfg.bcmBits > 8) return setError("bcmBits must be 1-8");
  if (cfg.baseCycles < 1 || cfg.floorCycles < 1 || cfg.floorCycles >= cfg.baseCycles)
    return setError("need 1 <= floorCycles < baseCycles");
  if (cfg.clockHz < 100000 || cfg.clockHz > 8000000) return setError("clockHz out of range");

  _planes = cfg.bcmBits + 1;
  int offset = 0;
  for (int p = 0; p < _planes; ++p) {
    _planeStart[p] = offset;
    offset += SHIFT_CYCLES + LATCH_CYCLES + planeWeight(p);
  }
  _addrBlock = offset;
  _wordsPerAddr = (size_t)_addrBlock * 2;

  logf("LedTile: %u-bit BCM (%u levels), base %u / floor %u cycles\n", cfg.bcmBits,
       (unsigned)(1u << cfg.bcmBits), cfg.baseCycles, cfg.floorCycles);
  logf("LedTile: heap before alloc: DMA-capable free %u, largest block %u\n",
       (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
       (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  // One internal-RAM chunk per address rather than one large block, so only
  // ADDR_COUNT medium-sized blocks have to be found. The I2S descriptor chain
  // plays them back to back.
  for (int a = 0; a < ADDR_COUNT; ++a) {
    _wave[a] = (uint32_t *)heap_caps_malloc(_wordsPerAddr * sizeof(uint32_t), MALLOC_CAP_DMA);
    if (!_wave[a]) {
      logf("LedTile: chunk %d (%u bytes) failed; DMA free %u, largest %u\n", a,
           (unsigned)(_wordsPerAddr * sizeof(uint32_t)),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
      freeWaveform();
      return setError("could not allocate DMA waveform");
    }
  }
  logf("LedTile: waveform %u bytes in %d chunks; DMA free after alloc %u\n",
       (unsigned)waveformBytes(), ADDR_COUNT,
       (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));

  memset(_fb, 0, sizeof(_fb));
  memset(_dirty, 0, sizeof(_dirty));
  _allDirty = false;
  buildLut();
  buildWaveform();
  if (!validateWaveform()) {
    freeWaveform();
    return setError("invalid DMA clock/latch waveform");
  }

  _transport = TRANSPORT_DMA;
  if (!startI2S()) {
    freeWaveform();
    return setError("I2S setup failed");
  }
  _running = true;
  logf("LedTile: running, panel clock %u Hz, refresh %u Hz\n", (unsigned)_cfg.clockHz,
       (unsigned)refreshHz());
  return true;
}

void LedTile::end() {
  if (_transport == TRANSPORT_DMA) stopI2S();
  if (_running || _wave[0]) releasePinsToGpio();
  _running = false;
  freeWaveform();
}

void LedTile::freeWaveform() {
  for (int a = 0; a < ADDR_COUNT; ++a) {
    if (_wave[a]) heap_caps_free(_wave[a]);
    _wave[a] = nullptr;
  }
}

size_t LedTile::waveformBytes() const {
  return _wave[0] ? ADDR_COUNT * _wordsPerAddr * sizeof(uint32_t) : 0;
}

uint8_t LedTile::getPins(uint8_t *out) const {
  const int8_t pins[kPinCount] = {_cfg.pinD1, _cfg.pinD2, _cfg.pinLat, _cfg.pinOe,
                                  _cfg.pinA0, _cfg.pinA1, _cfg.pinClk};
  for (uint8_t i = 0; i < kPinCount; ++i) out[i] = (uint8_t)pins[i];
  return kPinCount;
}

// ----------------------------------------------------------------- waveform

void LedTile::buildWaveform() {
  for (int a = 0; a < ADDR_COUNT; ++a) {
    for (int p = 0; p < _planes; ++p) {
      const int block = SHIFT_CYCLES + LATCH_CYCLES + planeWeight(p);
      // The address lines only move at the first plane of each address; later
      // planes of the same address hold steady. While the first plane shifts,
      // the previous address is held, matching the original bit-bang driver.
      const int shiftAddr = (p == 0) ? (a + ADDR_COUNT - 1) % ADDR_COUNT : a;
      for (int cc = 0; cc < block; ++cc) {
        const int address = (cc < SHIFT_CYCLES) ? shiftAddr : a;
        uint16_t v = 0;
        if (address & 1) v |= BUS_A0;
        if (address & 2) v |= BUS_A1;
        // Blanked for the whole shift + latch window, plus the last cycle of
        // the block for margin at the boundary.
        if (cc < SHIFT_CYCLES + LATCH_CYCLES || cc == block - 1) v |= BUS_OE;
        if (cc == SHIFT_CYCLES + 1 || cc == SHIFT_CYCLES + 2) v |= BUS_LAT;
        const uint16_t high = v | ((cc < SHIFT_CYCLES) ? BUS_CLK : 0);
        const int idx = _planeStart[p] + cc;
        _wave[a][2 * idx] = dupHalf(v);
        _wave[a][2 * idx + 1] = dupHalf(high);
      }
    }
  }
}

// Structural check of the encoded waveform before any hardware is touched:
// exactly SHIFT_CYCLES rising clocks and one clock-low latch per plane, the
// panel blanked whenever it shifts or latches, and the address steady whenever
// it is lit. Catches layout off-by-ones as a boot-time error.
bool LedTile::validateWaveform() const {
  uint16_t previous = BUS_OE;
  for (int a = 0; a < ADDR_COUNT; ++a) {
    for (int p = 0; p < _planes; ++p) {
      const int block = SHIFT_CYCLES + LATCH_CYCLES + planeWeight(p);
      unsigned shifted = 0, latches = 0;
      for (int lc = 0; lc < block; ++lc) {
        const int idx = _planeStart[p] + lc;
        for (int s = 0; s < SAMPLES_PER_CYCLE; ++s) {
          const uint32_t pair = _wave[a][2 * idx + (s >> 1)];
          if ((uint16_t)pair != (uint16_t)(pair >> 16)) return false;
          const uint16_t value = (uint16_t)pair;
          if ((value & BUS_CLK) && !(previous & BUS_CLK)) {
            if (!(value & BUS_OE) || (value & BUS_LAT) || shifted >= (unsigned)SHIFT_CYCLES)
              return false;
            ++shifted;
            if ((previous ^ value) & (BUS_D1 | BUS_D2)) return false;
          }
          if (value & BUS_LAT) {
            if ((value & BUS_CLK) || !(value & BUS_OE)) return false;
            if (!(previous & BUS_LAT)) {
              if (previous & BUS_CLK) return false;
              if (shifted != (unsigned)SHIFT_CYCLES) return false;
              ++latches;
            }
          }
          if (!(value & BUS_OE) || (value & BUS_LAT)) {
            const unsigned address = ((value & BUS_A0) ? 1 : 0) | ((value & BUS_A1) ? 2 : 0);
            if (address != (unsigned)a || latches != 1 || (value & BUS_CLK)) return false;
          }
          previous = value;
        }
      }
      if (shifted != (unsigned)SHIFT_CYCLES || latches != 1) return false;
    }
  }
  return true;
}

// ------------------------------------------------------------------- pixels

// Stream byte i goes out MSB first, so bit b of stream byte i sits at cycle
// i * 8 + (7 - b), in the shift window of every plane. The bit is set in both
// the low and the high clock phase so it is stable across the rising edge.
void LedTile::setStreamBit(uint8_t addr, int plane, uint8_t bank, uint8_t byteIdx,
                           uint8_t bitInByte, bool on) {
  const uint32_t m = dupHalf(bank == BANK_TOP ? BUS_D1 : BUS_D2);
  const int idx = _planeStart[plane] + byteIdx * 8 + (7 - bitInByte);
  uint32_t &w0 = _wave[addr][2 * idx];
  uint32_t &w1 = _wave[addr][2 * idx + 1];
  if (on) {
    w0 |= m;
    w1 |= m;
  } else {
    w0 &= ~m;
    w1 &= ~m;
  }
}

void LedTile::setPixel(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  if (x >= kWidth || y >= kHeight) return;
  const int i = y * kWidth + x;
  uint8_t *p = &_fb[i * 3];
  if (p[0] == r && p[1] == g && p[2] == b) return;
  p[0] = r;
  p[1] = g;
  p[2] = b;
  _dirty[i >> 3] |= (uint8_t)(1u << (i & 7));
}

uint32_t LedTile::getPixel(uint8_t x, uint8_t y) const {
  if (x >= kWidth || y >= kHeight) return 0;
  const uint8_t *p = &_fb[(y * kWidth + x) * 3];
  return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

void LedTile::clear() {
  for (int i = 0; i < kPixels; ++i) {
    uint8_t *p = &_fb[i * 3];
    if (p[0] | p[1] | p[2]) {
      p[0] = p[1] = p[2] = 0;
      _dirty[i >> 3] |= (uint8_t)(1u << (i & 7));
    }
  }
}

void LedTile::rawClear() {
  memset(_fb, 0, sizeof(_fb));
  memset(_dirty, 0, sizeof(_dirty));
  _allDirty = false;
  if (!_wave[0]) return;
  const uint32_t m = dupHalf(BUS_D1 | BUS_D2);
  for (int a = 0; a < ADDR_COUNT; ++a)
    for (int p = 0; p < _planes; ++p)
      for (int c = 0; c < SHIFT_CYCLES; ++c) {
        const int idx = _planeStart[p] + c;
        _wave[a][2 * idx] &= ~m;
        _wave[a][2 * idx + 1] &= ~m;
      }
}

void LedTile::renderPixel(uint8_t x, uint8_t y) {
  PixelSlot s;
  if (!slot(x, y, &s)) return;
  const uint8_t *p = &_fb[(y * kWidth + x) * 3];
  const uint8_t in[3] = {p[2], p[1], p[0]};  // stream order is B, G, R
  for (uint8_t c = 0; c < 3; ++c) {
    const uint16_t level = _lut[in[c]];
    // A value the curve rounds to level 0 still lights the dedicated floor
    // plane rather than going fully dark, so "dim" never reads as "off".
    const bool useFloor = in[c] != 0 && _bri != 0 && level == 0;
    const uint8_t byteIdx = s.group * 6 + c * 2 + s.halfIdx;
    for (int pl = 0; pl < _cfg.bcmBits; ++pl)
      setStreamBit(s.addr, pl, s.bank, byteIdx, s.bitInByte, (level >> pl) & 1);
    setStreamBit(s.addr, _cfg.bcmBits, s.bank, byteIdx, s.bitInByte, useFloor);
  }
}

void LedTile::show() {
  if (!_running) return;
  if (_allDirty) {
    for (int y = 0; y < kHeight; ++y)
      for (int x = 0; x < kWidth; ++x) renderPixel(x, y);
  } else {
    for (int i = 0; i < kPixels; ++i)
      if (_dirty[i >> 3] & (1u << (i & 7))) renderPixel(i % kWidth, i / kWidth);
  }
  memset(_dirty, 0, sizeof(_dirty));
  _allDirty = false;
}

// --------------------------------------------------------------- appearance

// The 8-bit input is mapped to a BCM level in one step: perceptual curve (CIE
// 1931 lightness blended with linear by gammaStrength), then brightness, then
// quantisation to the configured depth.
//
// A raw power-law gamma was tried first and was wrong at low depth: it crushed
// over a third of the range to fully off. The CIE curve has a linear segment
// near black, and the floor plane covers whatever still rounds to zero.
void LedTile::buildLut() {
  const float maxLevel = (float)((1u << _cfg.bcmBits) - 1);
  const float s = _gammaOn ? _gammaStrength : 0.0f;
  for (int i = 0; i < 256; ++i) {
    const float lstar = (i / 255.0f) * 100.0f;
    float y;
    if (lstar <= 8.0f) {
      y = lstar / 903.3f;
    } else {
      const float t = (lstar + 16.0f) / 116.0f;
      y = t * t * t;
    }
    float blended = i + (y * 255.0f - i) * s;
    if (blended < 0.0f) blended = 0.0f;
    if (blended > 255.0f) blended = 255.0f;
    const float scaled = (blended / 255.0f) * (_bri / 255.0f);
    _lut[i] = (uint16_t)(scaled * maxLevel + 0.5f);
  }
}

void LedTile::setBrightness(uint8_t b) {
  if (b == _bri) return;
  _bri = b;
  buildLut();
  markAllDirty();
}

void LedTile::setGamma(bool enabled) {
  if (enabled == _gammaOn) return;
  _gammaOn = enabled;
  buildLut();
  markAllDirty();
}

void LedTile::setGammaStrength(float strength) {
  if (strength < 0.0f) strength = 0.0f;
  if (strength > 1.0f) strength = 1.0f;
  _gammaStrength = strength;
  buildLut();
  markAllDirty();
}

// ------------------------------------------------------------------- output

bool LedTile::startI2S() {
  const int8_t pins[kPinCount] = {_cfg.pinD1, _cfg.pinD2, _cfg.pinLat, _cfg.pinOe,
                                  _cfg.pinA0, _cfg.pinA1, _cfg.pinClk};
  I2SSegment segs[ADDR_COUNT];
  for (int a = 0; a < ADDR_COUNT; ++a) {
    segs[a].buf = _wave[a];
    segs[a].bytes = _wordsPerAddr * sizeof(uint32_t);
  }
  // WS stays internal: CLK is encoded in the data like every other signal.
  if (!i2sParallelBegin(pins, kPinCount, -1, _cfg.clockHz * SAMPLES_PER_CYCLE, segs,
                        ADDR_COUNT, false, _cfg.i2sPort))
    return false;
  _i2sOn = true;
  return true;
}

void LedTile::stopI2S() {
  if (!_i2sOn) return;
  i2sParallelStop();
  _i2sOn = false;
}

// Hands the pins back from the I2S matrix to plain GPIO, blanked.
void LedTile::releasePinsToGpio() {
  const int8_t pins[kPinCount] = {_cfg.pinD1, _cfg.pinD2, _cfg.pinLat, _cfg.pinOe,
                                  _cfg.pinA0, _cfg.pinA1, _cfg.pinClk};
  for (uint8_t i = 0; i < kPinCount; ++i) {
    gpio_matrix_out(pins[i], SIG_GPIO_OUT_IDX, false, false);
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  digitalWrite(_cfg.pinOe, HIGH);  // blank
}

bool LedTile::setClockHz(uint32_t hz) {
  if (hz < 100000 || hz > 8000000) return false;
  _cfg.clockHz = hz;
  if (_running && _transport == TRANSPORT_DMA) {
    stopI2S();
    return startI2S();
  }
  return true;
}

uint32_t LedTile::refreshHz() const {
  if (!_running || _transport != TRANSPORT_DMA) return 0;
  return i2sParallelActualHz() / (uint32_t)(ADDR_COUNT * SAMPLES_PER_CYCLE * _addrBlock);
}

void LedTile::setTransport(Transport t) {
  if (!_running || t == _transport) return;
  if (t == TRANSPORT_BITBANG) {
    stopI2S();
    releasePinsToGpio();
    _transport = t;
  } else {
    _transport = t;
    startI2S();
  }
}

void LedTile::service() {
  if (_running && _transport == TRANSPORT_BITBANG) bitbangFrame();
}

// One full pass over the waveform driven by the CPU. Deliberately mirrors the
// original bit-banged driver: digitalWrite for data setup time, latch with the
// clock stopped, and a plain delay (scaled by plane weight) for the display.
// A coarse debugging aid, not cycle-accurate.
void LedTile::bitbangFrame() {
  for (int a = 0; a < ADDR_COUNT; ++a) {
    for (int p = 0; p < _planes; ++p) {
      digitalWrite(_cfg.pinOe, HIGH);
      for (int c = 0; c < SHIFT_CYCLES; ++c) {
        const uint16_t v = (uint16_t)_wave[a][2 * (_planeStart[p] + c)];
        digitalWrite(_cfg.pinD1, (v & BUS_D1) ? HIGH : LOW);
        digitalWrite(_cfg.pinD2, (v & BUS_D2) ? HIGH : LOW);
        digitalWrite(_cfg.pinClk, HIGH);
        digitalWrite(_cfg.pinClk, LOW);
      }
      digitalWrite(_cfg.pinA0, a & 1);
      digitalWrite(_cfg.pinA1, (a >> 1) & 1);
      digitalWrite(_cfg.pinLat, HIGH);
      delayMicroseconds(1);
      digitalWrite(_cfg.pinLat, LOW);
      digitalWrite(_cfg.pinOe, LOW);
      delayMicroseconds(planeWeight(p));
      digitalWrite(_cfg.pinOe, HIGH);
    }
  }
}

// -------------------------------------------------------------- diagnostics

bool LedTile::dmaStatus(I2SParallelStatus *out) const {
  if (!_i2sOn) return false;
  i2sParallelGetStatus(out);
  return true;
}

bool LedTile::waitForFrame(uint32_t timeoutUs) {
  return _i2sOn && i2sParallelWaitForFrame(timeoutUs);
}

void LedTile::rawSetWord(uint8_t addr, uint8_t bank, uint8_t group, uint8_t colour,
                         uint16_t bits) {
  if (!_wave[0] || addr >= ADDR_COUNT) return;
  for (uint8_t b = 0; b < 16; ++b) {
    const uint8_t byteIdx = group * 6 + colour * 2 + ((b >= 8) ? 0 : 1);
    const bool on = (bits >> b) & 1;
    for (int p = 0; p < _planes; ++p) setStreamBit(addr, p, bank, byteIdx, b & 7, on);
  }
}

void LedTile::rawFillCycles(uint8_t addr, int count) {
  if (!_wave[0] || addr >= ADDR_COUNT) return;
  if (count > SHIFT_CYCLES) count = SHIFT_CYCLES;
  const uint32_t m = dupHalf(BUS_D1 | BUS_D2);
  for (int p = 0; p < _planes; ++p)
    for (int c = 0; c < count; ++c) {
      const int idx = _planeStart[p] + c;
      _wave[addr][2 * idx] |= m;
      _wave[addr][2 * idx + 1] |= m;
    }
}
