# LED Tile v2 in WLED

The sibling WLED checkout contains an optional native `BusLedTile` output. This
is not a HUB75 output and requires a custom firmware build.

## Build

With `LedBoard` and `WLED` next to each other, copy
`reference/wled_platformio_override.ini` to `WLED/platformio_override.ini`.
The latter is intentionally ignored by WLED's Git configuration. The `LedTile`
library is linked from this repository, not copied into WLED.

From the WLED directory:

```powershell
npm.cmd ci
npm.cmd run build
pio run -e esp32_ledtile
```

The sample override also defines `esp32_huzzah32_ledtile` for the original
Adafruit HUZZAH32 Feather. Build/upload that target with:

```powershell
pio run -e esp32_huzzah32_ledtile
pio run -e esp32_huzzah32_ledtile --target upload
```

The output is `WLED/.pio/build/esp32_ledtile/firmware.bin`. Building does not flash
the controller. The default environment uses WLED's Arduino 3 / ESP-IDF 5 setup,
the `esp-wrover-kit` board, and no usermods. Do not enable HUB75 or audio-reactive
hardware in this initial configuration.

### Windows build environment

If the platform's installer cannot update the Python environment running
PlatformIO, use a separate core directory. A short cache path also avoids the
Windows archive-extraction path-length limit, without changing the registry:

```powershell
$env:PLATFORMIO_CORE_DIR = "$env:LOCALAPPDATA\Temp\opencode\wled-ledtile-pio"
$env:PLATFORMIO_CACHE_DIR = "$env:USERPROFILE\.platformio\.cache"
$env:PLATFORMIO_PLATFORMS_DIR = "$env:USERPROFILE\.platformio\platforms"
$env:PLATFORMIO_PACKAGES_DIR = "$env:LOCALAPPDATA\Temp\opencode\wled-ledtile-pio\packages"
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
& $pio run -e esp32_huzzah32_ledtile --target upload
```

Use the same environment in that PowerShell window for later `pio` commands;
the variables apply only to that terminal process. A dedicated packages directory
prevents WLED's Arduino Core 3 framework/toolchain from colliding with the
standalone LedBoard demo's pinned framework packages.

## Wiring and configuration

For the WROVER profile, the default physical pins are:

| Signal | D1 | D2 | LAT | OE | A0 | A1 | CLK |
|---|---|---|---|---|---|---|---|
| GPIO | 26 | 18 | 19 | 21 | 22 | 23 | 25 |

### Original Adafruit HUZZAH32 Feather pinout

The HUZZAH32 override targets the original Feather with the ESP32-WROOM module,
not the newer Feather ESP32 V2. Suggested panel mapping (signal order is
D1, D2, LAT, OE, A0, A1, CLK):

| Signal | D1 | D2 | LAT | OE | A0 | A1 | CLK |
|---|---:|---:|---:|---:|---:|---:|---:|
| ESP32 GPIO | 16 | 17 | 25 | 26 | 27 | 21 | 4 |
| HUZZAH32 label | RX | TX | A1 | A0 | 27 | 21 | A5 |

GPIO16/17 are the Feather's Serial1 RX/TX (not its USB serial pins). This map
avoids the boot-strapping GPIOs, GPIO13's onboard red LED, GPIO32/33's optional
32 kHz crystal function, USB/UART0, JTAG and common SPI pins. GPIO21 is
available as a general-purpose pin, but is also commonly used for I2C SDA; leave
WLED global I2C unconfigured or change the panel pin map. WLED's pin manager
will reject an I2C/panel pin conflict. The build disables the default GPIO0
button so the panel doesn't claim a boot-strapping pin. All panel inputs use
3.3V logic; use a separate fused 5V supply with common ground as above.

The HUZZAH32 has no PSRAM, so the environment does not define `BOARD_HAS_PSRAM`.
Its Feather USB-UART is configured at 115200 baud for conservative upload
reliability. Confirm GPIO labels against the exact board revision before wiring.
Pinout source: [Adafruit HUZZAH32 Feather pinouts](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts).

**WROVER-KIT only:** disconnect its onboard LCD. Use a separate, appropriately fused
5V supply for the panel, with common ground. Verify signal-header orientation and
pin numbering on the actual panel before wiring. Never power it through USB or
the ESP32 dev board. **WLED automatic current limiting does not cover LedTile.**

Override physical pins at build time with `-D WLED_LEDTILE_PINS=26,18,19,21,22,23,25`
(D1, D2, LAT, OE, A0, A1, CLK). The driver reserves all seven pins through WLED's
PinManager. The panel clock defaults to 2 MHz and can be overridden using
`-D WLED_LEDTILE_CLOCK_HZ=2000000`.

On a fresh install, the default output is one 64x16 panel at index 0 and the
matrix layout is horizontal, top-left, non-serpentine. With existing WLED settings,
select **LED Tile v2 (64x16)** in LED Preferences and set this layout in 2D
Configuration. Existing layouts are not overwritten.

The five `pin` entries in WLED's bus configuration are **not GPIO numbers**:

```json
{"type":72,"start":0,"len":1024,"pin":[64,16,4,16,2],"skip":0,"rev":false}
```

They hold width, height, BCM bits, base cycles and floor cycles. Width/height are
fixed; bits must be 1-8, base 2-255, and floor 1 through base-1. WLED retains its
colour-gamma controls; the library's separate gamma is disabled to avoid applying
correction twice. Global brightness is applied once by LedTile. Reversed rotates
the tile 180 degrees.

## Memory and limitations

The conservative default is 4-bit BCM, base 16, floor 2. It allocates **70,304
waveform bytes**, plus DMA descriptors and the library framebuffer. Every BCM
plane has its own one-cycle blanking margin, preserving exact binary ON times.

WLED accounts for this allocation in its LED budget. The bus also checks the
largest internal DMA block and preserves at least 32 KiB of DMA-capable free heap
at startup. Those checks do not guarantee sufficient memory for every effect,
transition, web operation or OTA update: hardware stress testing is still needed.
Classic ESP32 PSRAM cannot store this DMA waveform.

- One active tile only; no panel chaining in this version.
- Classic ESP32 only. S3/C3/etc. require a different low-level backend.
- LedTile builds reserve I2S1 exclusively: additional strips must use RMT, and
  automatic fallback from RMT to I2S is disabled.
- No automatic current estimation or limiting for the panel.
- Updates modify the active waveform; tearing remains possible. No second DMA
  waveform is allocated.
- Four bits give 16 BCM levels plus the special dim floor, not full 24-bit colour.
  Higher depth is selectable but may exceed WLED's memory budget or DMA headroom.

## Hardware acceptance checklist

1. Confirm successful startup, waveform size, refresh rate and heap diagnostics.
2. Verify RGB channels, four corners, row order and 180-degree reversal.
3. Test global brightness 0/1/255, gradients, fades and low-level floor output.
4. Run 2D effects and realtime input; inspect tearing and rendering performance.
5. Save/reload settings, reboot, and repeatedly reconfigure the output.
6. Exercise Wi-Fi reconnects, web requests, presets and OTA.
7. Run an extended soak test while monitoring free internal heap and largest block.

Do not raise the memory limit or depth until the conservative profile passes.
