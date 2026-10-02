#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Minimal 16-bit I2S parallel ("LCD mode") output for the ESP32, driving a
// circular DMA chain that repeats forever with no CPU involvement.
//
// The ESP32 routes I2S through the GPIO matrix, so any output-capable pin can
// carry any of the bus signals -- the panel wiring does not have to change.
//
//   busPins    up to 16 GPIOs, index 0 = bus bit 0. Use -1 to skip a bit.
//   busWidth   how many entries of busPins are populated
//   clkPin     GPIO that receives the generated pixel clock
//   clockHz    target pixel clock; the divider is integer so the real rate is
//              reported back by i2sParallelActualHz()
//   buffer     DMA-capable memory (heap_caps_malloc with MALLOC_CAP_DMA)
//   lengthBytes size of that buffer; it is played end to end, then looped
//
// clockInvert selects which clock edge the receiver samples on. The MBI5034
// shifts on the rising edge; if data lands one bit out, this is the flag to
// flip first.
// The playback buffer is given as several segments that are played back to
// back, then looped. They need not be contiguous with each other, which lets a
// large waveform be built from several smaller internal-RAM allocations rather
// than one huge block. Each segment must be DMA-capable and a multiple of 4 bytes.
typedef struct {
  void *buf;
  size_t bytes;
} I2SSegment;

// port selects I2S0 or I2S1. I2S1 is the one proven on hardware; I2S0 is
// untested and is shared with the DAC/ADC paths and often claimed by audio
// code, so prefer I2S1 unless something else (e.g. WLED's I2S pixel outputs)
// already owns it.
bool i2sParallelBegin(const int8_t *busPins, uint8_t busWidth, int8_t clkPin,
                      uint32_t clockHz, const I2SSegment *segments,
                      size_t segmentCount, bool clockInvert, uint8_t port = 1);

// Real pixel clock after integer division, in Hz. Zero until begin() succeeds.
uint32_t i2sParallelActualHz(void);

void i2sParallelStop(void);

// Descriptor addresses can repeat even while a circular DMA chain is running.
typedef struct {
  uint32_t descBase;   // address of descriptor 0
  uint32_t descCount;
  uint32_t linkDscr;   // descriptor the engine is currently on
  uint32_t eofDesc;
  uint32_t intRaw;     // interrupt status; a descriptor error shows up here
  uint32_t state;
  bool clockEnabled;
  bool txStarted;
  bool dmaParked;
  bool descriptorError;
} I2SParallelStatus;

void i2sParallelGetStatus(I2SParallelStatus *out);

// Clears the latched TX EOF flag and waits for a new end-of-frame DMA event.
bool i2sParallelWaitForFrame(uint32_t timeoutUs);
