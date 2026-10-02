#include "i2s_parallel.h"

#include <Arduino.h>
#include <driver/periph_ctrl.h>
#include <esp_heap_caps.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>
#include <soc/lldesc.h>

// A single DMA descriptor can carry at most 4095 bytes. Held well below that so
// the chain is several descriptors long and out_link_dscr visibly moves, which
// is what distinguishes a running engine from a stalled one.
#define MAX_DMA_LEN 2048

// I2S1 is used rather than I2S0, which is entangled with the DAC and ADC paths.
#define I2S_PORT I2S1

static lldesc_t *sDesc = NULL;
static int sDescCount = 0;
static uint32_t sActualHz = 0;

uint32_t i2sParallelActualHz(void) { return sActualHz; }

bool i2sParallelBegin(const int8_t *busPins, uint8_t busWidth, int8_t clkPin,
                      uint32_t clockHz, const I2SSegment *segments,
                      size_t segmentCount, bool clockInvert) {
  if (!busPins || !segments || segmentCount == 0 || clockHz == 0 ||
      busWidth == 0 || busWidth > 16) {
    return false;
  }

  // ---- descriptor chain, linked back to itself so playback loops forever ----
  sDescCount = 0;
  for (size_t i = 0; i < segmentCount; ++i) {
    if (!segments[i].buf || segments[i].bytes == 0 || (segments[i].bytes & 3)) {
      return false;
    }
    sDescCount += (segments[i].bytes + MAX_DMA_LEN - 1) / MAX_DMA_LEN;
  }
  sDesc = (lldesc_t *)heap_caps_malloc(sDescCount * sizeof(lldesc_t),
                                       MALLOC_CAP_DMA);
  if (!sDesc) {
    return false;
  }

  int d = 0;
  for (size_t i = 0; i < segmentCount; ++i) {
    // Split each segment evenly rather than filling descriptors to the brim,
    // so every chunk stays 4-byte aligned and no runt descriptor ends it.
    const int pieces = (segments[i].bytes + MAX_DMA_LEN - 1) / MAX_DMA_LEN;
    const size_t chunk = ((segments[i].bytes / pieces) + 3) & ~((size_t)3);
    uint8_t *p = (uint8_t *)segments[i].buf;
    size_t remaining = segments[i].bytes;
    for (int k = 0; k < pieces; ++k, ++d) {
      size_t n = remaining > chunk ? chunk : remaining;
      sDesc[d].size = n;
      sDesc[d].length = n;
      sDesc[d].buf = p;
      // EOF on the wrap point only, so out_eof_des_addr and the OUT_EOF flag
      // report a real lap of the chain rather than staying blank.
      sDesc[d].eof = (d == sDescCount - 1) ? 1 : 0;
      sDesc[d].sosf = 0;
      sDesc[d].owner = 1;
      sDesc[d].offset = 0;
      sDesc[d].qe.stqe_next = &sDesc[(d + 1) % sDescCount];
      p += n;
      remaining -= n;
    }
  }

  // Reset as well as enable: begin() is called again whenever the clock or the
  // transport changes, and without the reset the peripheral keeps whatever
  // state the previous run left behind.
  periph_module_reset(PERIPH_I2S1_MODULE);
  periph_module_enable(PERIPH_I2S1_MODULE);

  // ---- pin routing ----
  //
  // In 16-bit parallel mode the data is taken from the HIGH half of the I2S
  // shift register, so bus bit 0 is signal DATA_OUT8, not DATA_OUT0. This is
  // the single most common thing to get wrong here.
  for (uint8_t i = 0; i < busWidth; ++i) {
    if (busPins[i] < 0) {
      continue;
    }
    pinMode(busPins[i], OUTPUT);
    gpio_matrix_out(busPins[i], I2S1O_DATA_OUT8_IDX + i, false, false);
  }
  if (clkPin >= 0) {
    pinMode(clkPin, OUTPUT);
    gpio_matrix_out(clkPin, I2S1O_WS_OUT_IDX, clockInvert, false);
  }

  // ---- clock ----
  //
  // The peripheral runs from the 160 MHz PLL_D2 clock:
  //     f_bck = 160 MHz / clkm_div_num / tx_bck_div_num
  // and the TRM adds, for LCD master transmit mode, that "the frequency of WS
  // is half of f_bck". WS is the pin being used as the pixel clock and one
  // 16-bit bus word goes out per WS period, so the extra factor of 2 belongs in
  // the divider -- without it the panel is clocked at half the requested rate.
  // tx_bck_div_num must be at least 2, so hold it there and divide the rest.
  const uint32_t kBaseHz = 160000000UL;
  const uint32_t kBckDiv = 2;
  const uint32_t kWsDiv = 2;
  uint32_t divN = kBaseHz / (clockHz * kBckDiv * kWsDiv);
  if (divN < 2) {
    divN = 2;
  }
  if (divN > 255) {
    divN = 255;
  }
  sActualHz = kBaseHz / (divN * kBckDiv * kWsDiv);

  // The RX side is configured to match TX throughout. It is unused here, but
  // the datasheet asks for it to be set sensibly even in LCD mode.
  I2S_PORT.sample_rate_conf.val = 0;
  I2S_PORT.sample_rate_conf.tx_bits_mod = 16;
  I2S_PORT.sample_rate_conf.rx_bits_mod = 16;
  I2S_PORT.sample_rate_conf.tx_bck_div_num = kBckDiv;
  I2S_PORT.sample_rate_conf.rx_bck_div_num = kBckDiv;

  I2S_PORT.clkm_conf.val = 0;
  I2S_PORT.clkm_conf.clka_en = 0;  // PLL_D2, not APLL
  I2S_PORT.clkm_conf.clkm_div_a = 1;
  I2S_PORT.clkm_conf.clkm_div_b = 0;
  I2S_PORT.clkm_conf.clkm_div_num = divN;
  // Clearing clkm_conf also disables the module's internal clock.
  I2S_PORT.clkm_conf.clk_en = 1;

  // ---- LCD (parallel) mode ----
  I2S_PORT.conf2.val = 0;
  I2S_PORT.conf2.lcd_en = 1;
  I2S_PORT.conf2.lcd_tx_wrx2_en = 0;
  I2S_PORT.conf2.lcd_tx_sdx2_en = 0;

  // Clears tx_start and tx_slave_mod along with everything else, so nothing
  // survives from a previous begin().
  I2S_PORT.conf.val = 0;
  // The panel waveform duplicates halfwords, so pair ordering cannot change its edges.
  I2S_PORT.conf.tx_right_first = 1;
  I2S_PORT.conf.tx_msb_right = 1;

  // ---- FIFO fed from DMA ----
  I2S_PORT.fifo_conf.val = 0;
  I2S_PORT.fifo_conf.tx_data_num = 32;
  I2S_PORT.fifo_conf.rx_data_num = 32;
  I2S_PORT.fifo_conf.dscr_en = 1;
  I2S_PORT.fifo_conf.tx_fifo_mod = 1;  // 16-bit single channel
  I2S_PORT.fifo_conf.tx_fifo_mod_force_en = 1;
  I2S_PORT.fifo_conf.rx_fifo_mod_force_en = 1;

  I2S_PORT.conf_chan.val = 0;
  I2S_PORT.conf_chan.tx_chan_mod = 1;
  I2S_PORT.conf_chan.rx_chan_mod = 1;

  // ---- reset, AFTER the configuration and not before ----
  //
  // Flush the FIFO and DMA state after selecting the transfer format.
  I2S_PORT.conf.rx_fifo_reset = 1;
  I2S_PORT.conf.rx_fifo_reset = 0;
  I2S_PORT.conf.tx_fifo_reset = 1;
  I2S_PORT.conf.tx_fifo_reset = 0;

  I2S_PORT.lc_conf.in_rst = 1;
  I2S_PORT.lc_conf.in_rst = 0;
  I2S_PORT.lc_conf.out_rst = 1;
  I2S_PORT.lc_conf.out_rst = 0;
  // The AHB bridge between the DMA engine and the I2S FIFO has its own reset,
  // separate from in_rst/out_rst above.
  I2S_PORT.lc_conf.ahbm_fifo_rst = 1;
  I2S_PORT.lc_conf.ahbm_fifo_rst = 0;
  I2S_PORT.lc_conf.ahbm_rst = 1;
  I2S_PORT.lc_conf.ahbm_rst = 0;

  // Do not inherit link commands from the previous transfer.
  I2S_PORT.in_link.val = 0;
  I2S_PORT.out_link.val = 0;

  I2S_PORT.conf.rx_reset = 1;
  I2S_PORT.conf.tx_reset = 1;
  I2S_PORT.conf.rx_reset = 0;
  I2S_PORT.conf.tx_reset = 0;

  I2S_PORT.conf1.val = 0;
  I2S_PORT.conf1.tx_stop_en = 0;
  I2S_PORT.conf1.tx_pcm_bypass = 1;

  I2S_PORT.timing.val = 0;

  // ---- go ----
  I2S_PORT.lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
  // Stale flags would otherwise be reported forever by i2sParallelGetStatus().
  I2S_PORT.int_clr.val = 0xFFFFFFFF;
  // out_link.addr is a 20-bit field; mask explicitly rather than relying on
  // the bitfield to truncate a full pointer.
  I2S_PORT.out_link.addr = ((uint32_t)&sDesc[0]) & 0x000FFFFF;
  I2S_PORT.out_link.stop = 0;
  I2S_PORT.out_link.start = 1;
  I2S_PORT.conf.tx_start = 1;

  return true;
}

void i2sParallelStop(void) {
  I2S_PORT.conf.tx_start = 0;
  I2S_PORT.out_link.start = 0;
  I2S_PORT.out_link.stop = 1;
  // The engine may have a descriptor fetch in flight; let it retire before the
  // descriptor memory is handed back to the heap.
  delayMicroseconds(100);
  if (sDesc) {
    heap_caps_free(sDesc);
    sDesc = NULL;
    sDescCount = 0;
  }
  sActualHz = 0;
}

void i2sParallelGetStatus(I2SParallelStatus *out) {
  if (!out) {
    return;
  }
  out->descBase = (uint32_t)sDesc;
  out->descCount = (uint32_t)sDescCount;
  out->linkDscr = I2S_PORT.out_link_dscr;
  out->eofDesc = I2S_PORT.out_eof_des_addr;
  out->intRaw = I2S_PORT.int_raw.val;
  out->state = I2S_PORT.state.val;
  out->clockEnabled = I2S_PORT.clkm_conf.clk_en;
  out->txStarted = I2S_PORT.conf.tx_start;
  out->dmaParked = I2S_PORT.out_link.park;
  out->descriptorError = I2S_PORT.int_raw.out_dscr_err;
}

bool i2sParallelWaitForFrame(uint32_t timeoutUs) {
  if (!sDesc || !I2S_PORT.conf.tx_start) {
    return false;
  }
  // Only a newly asserted EOF proves progress; the EOF address stays latched.
  I2S_PORT.int_clr.val = I2S_OUT_EOF_INT_CLR;
  const uint32_t started = micros();
  do {
    if (I2S_PORT.int_raw.val & I2S_OUT_EOF_INT_RAW) {
      return true;
    }
    delayMicroseconds(10);
  } while ((uint32_t)(micros() - started) < timeoutUs);
  return false;
}
