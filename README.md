# ADC

Flexible multi-channel Teensy 4.0 ADC capture project with high-rate USB streaming, optional SD logging, and Python-based capture/plot tools.

This project samples up to 14 analog inputs and streams `uint16` samples over native USB Serial to a host PC.

The firmware is designed for:

- configurable channel count (1–14 channels)
- high sustained capture rate (dual-DMA mode for 1–2 channels, ~1.3 Msps/channel)
- independent USB streaming and SD logging outputs
- minimal ISR work in DMA mode
- efficient scanning mode for many channels
- buffered host streaming with Python decoding

## Overview

The active firmware is in `src/main.cpp`.

**Key Features:**
- **Configuration switches** near the top of the file:
  - `WRITE_SERIAL`: enable USB streaming and firmware prompts (default: `true`)
  - `WRITE_LOG`: enable SD card logging to `adc_capture.bin` (default: `false`)
  - `N_CHANNELS`: number of channels to capture, 1–14 (default: `2`)
- **Dual acquisition modes:**
  - **DMA mode** (N_CHANNELS ≤ 2): dual-ADC DMA pipeline with per-channel ISRs for maximum speed
  - **Scan mode** (N_CHANNELS > 2): round-robin `analogRead()` for flexibility across many pins
- **Stream format:**
  - DMA mode: interleaved `uint16` samples `[ch0[0], ch1[0], ch0[1], ch1[1], ...]`
  - Scan mode: frame-based samples `[ch0, ch1, ..., chN-1]` repeated
- **Ring buffers:** one per channel, 200 blocks each (100 KB total)
- **Startup behavior:** automatically discards one warmup block to skip ADC transient

The host-side script is `tools/serial_capture.py`.

It:

- waits for the Teensy prompt
- starts capture
- reads the binary stream in large chunks
- decodes samples into separate channel arrays (with dynamic deinterleaving)
- prints per-channel capture statistics
- optionally saves data to `.npy` format
- plots all channels on a single figure with legend

## Hardware

- Teensy 4.0 (dual ADC modules)
- Analog inputs: `A0–A13` (14 channels total, 7 per ADC module)
- USB connection to host PC

Notes:

- Teensy USB CDC does not use the configured monitor baud as a physical UART rate.
- The `115200` setting in `platformio.ini` is only for host-tool consistency.

## Stream Format

### DMA Mode (N_CHANNELS ≤ 2)

Firmware outputs raw little-endian `uint16` values interleaved:

```text
A0[0], A1[0], A0[1], A1[1], A0[2], A1[2], ...
```

Python decoding (on 512-byte blocks):

```python
A0 = block[0::2]    # Every 2nd uint16 starting at index 0
A1 = block[1::2]    # Every 2nd uint16 starting at index 1
```

### Scan Mode (N_CHANNELS > 2)

Firmware outputs frame-based samples, one frame per conversion:

```text
Frame_0: [ch0, ch1, ..., ch{N-1}]
Frame_1: [ch0, ch1, ..., ch{N-1}]
...
```

Python decoding:

```python
channels = []
for ch_idx in range(n_channels):
    channels.append(block[ch_idx::n_channels])
```

The deinterleave stride adapts to the configured channel count: `block[ch_idx::n_channels]`.

### Output Format

Saved `.npy` output is a stacked array with shape `(N_CHANNELS, samples_per_channel)`:

```
channels[0, :] = first channel
channels[1, :] = second channel
...
channels[N-1, :] = last channel
```

Each value is a 12-bit ADC sample (0–4095) stored in a `uint16`.

## Project Layout

- `src/main.cpp`: active multi-channel USB streaming firmware (configurable 1–14 channels)
- `tools/serial_capture.py`: host capture and plotting script with multi-channel decode
- `backup/main_sd_logger.cpp`: SD logging variant retained for reference
- `platformio.ini`: PlatformIO environment and monitor configuration

## Configuration

Edit `src/main.cpp` (lines 27–29) to select capture mode:

```cpp
const bool WRITE_LOG = false;      // SD card logging (adc_capture.bin)
const bool WRITE_SERIAL = true;    // USB streaming and prompts
const size_t N_CHANNELS = 2;       // Number of channels (1–14)
```

**Capture Mode:**
- Set `N_CHANNELS = 1` or `2` for maximum speed (DMA mode, ~1.3 Msps/channel observed)
- Set `N_CHANNELS > 2` for flexibility across more analog inputs (scan mode, lower rate)

**Output:**
- `WRITE_SERIAL = true`: stream data live to host via USB
- `WRITE_LOG = true`: save binary data to SD card as `adc_capture.bin`
- Both can be enabled simultaneously; performance depends on ADC settings and host throughput

**Channels in DMA mode (N_CHANNELS ≤ 2):**
- 1 channel uses `A0` on `adc0`
- 2 channels use `A0` on `adc0` and `A1` on `adc1`

**Channels in Scan mode (N_CHANNELS > 2):**
- Pins are specified in `ANALOG_PINS[]` array; currently `A0`–`A13` in order
- Each channel read adds per-sample latency; expect lower overall rate than DMA mode

## Build And Upload

From the project root:

```powershell
platformio run
platformio run --target upload --upload-port COM3
```

Or use the VS Code PlatformIO build/upload tasks.

## Python Requirements

Install dependencies:

```powershell
pip install pyserial numpy matplotlib
```

## Capture Usage

### Basic Example

```powershell
# Dual-channel capture (default)
python tools/serial_capture.py COM3 --max-samples 1000000 --out adc.npy

# Quad-channel capture
python tools/serial_capture.py COM3 --n-chan 4 --max-samples 500000 --out adc_4ch.npy
```

### Arguments

- `COM3`: serial port
- `--n-chan N`: number of channels to decode (default: `2`, range: `1`–`14`)
- `--max-samples N`: stop after `N` samples per channel
- `--out FILE.npy`: save stacked `(N_channels, samples_per_channel)` array

**Note:** The `--n-chan` argument must match the firmware's `N_CHANNELS` setting for correct deinterleaving.

### Execution Flow

The script will:

1. wait for the firmware prompt (e.g., "Enter to begin streaming")
2. trigger streaming
3. capture binary data in large chunks
4. stop the firmware with `q` (or `^C` to abort)
5. print Teensy-side statistics
6. plot all channels on a single figure with legend
7. save `.npy` file if `--out` is specified

## ADC Configuration

Current configuration is set in `src/main.cpp` (see `setup()` and `initDmaAdcDual()`):

- 12-bit resolution
- zero averaging (raw samples)
- conversion speed: `VERY_HIGH_SPEED` (for fast mode)
- sampling speed: `VERY_HIGH_SPEED` (for fast mode)

Example from firmware:

```cpp
adc.adc0->setAveraging(0);
adc.adc0->setResolution(12);
adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
```

For scan mode (N_CHANNELS > 2), the effective sample rate per channel decreases with channel count due to sequential `analogRead()` calls.

## Performance

### DMA Mode (N_CHANNELS ≤ 2)

Observed performance with dual-channel DMA:

- **~1.3 Msps per channel** at dual-ADC `VERY_HIGH_SPEED` settings
- Dual ISR-driven DMA with ring buffers ensures consistent throughput
- USB host latency has minimal impact due to 200-block-deep ring buffers

### Scan Mode (N_CHANNELS > 2)

Per-sample round-robin reading is slower:

- Sample rate scales inversely with channel count
- Suitable for slower sampling use-cases or when maximum channel flexibility is needed

### Factors Affecting Performance

- **ADC speed settings:** VERY_HIGH_SPEED vs. HIGH_SPEED vs. MED_SPEED
- **Host USB throughput:** local Python script is faster than remote desktop
- **Ring buffer depth:** 200 blocks × 512 bytes per channel provides good margin
- **SD logging (if enabled):** writes occur in background; USB streaming is unaffected

## Troubleshooting

### Sample values look wrong

**Sanity checks:**

- `A0` or `A1` tied to GND should read near `0x0000` (0)
- tied to `3.3V` should read near `0x0FFF` (4095)
- tied to mid-rail (1.65V) should read near `0x0800` (2048)

**If values are off:**

- Verify input wiring on the configured analog pins
- Check the Python decoder matches the firmware's `N_CHANNELS` setting (use `--n-chan` arg)
- Ensure ADC resolution is set to 12-bit in firmware

### Overrun or data loss

**If firmware reports "ERROR: Ring buffer overflow":**

- The host read was not fast enough for the current ADC rate
- Reduce ADC speed (e.g., switch from `VERY_HIGH_SPEED` to `HIGH_SPEED`)
- Run Python script locally (avoid remote desktop lag)
- Close any competing serial monitor windows
- For maximum margin, enable `WRITE_LOG = true` and `WRITE_SERIAL = false` to use SD only

**Late-stage overflow at end of run:**

- Single occurrence at shutdown is expected (ISR fires one more time after main loop stops)
- Repeated overflow during capture indicates sustained host lag; reduce ADC rate

### Deinterleaving errors (garbled data)

- Ensure `--n-chan` argument matches firmware `N_CHANNELS` setting
- For DMA mode (N ≤ 2): use `--n-chan 1` or `--n-chan 2`
- For scan mode (N > 2): use `--n-chan N` with the exact channel count

### Serial monitor reconnect/reset issues

This project disables monitor `DTR` and `RTS` in `platformio.ini` to avoid unnecessary Teensy resets when attaching host tools.

## SD Logging

SD card support is available via the `WRITE_LOG` switch in `src/main.cpp`:

```cpp
const bool WRITE_LOG = false;
```

### Using SD Logging

1. Set `WRITE_LOG = true` in `src/main.cpp`
2. Install a microSD card in the Teensy SD card socket
3. Build and upload firmware
4. Run capture (with or without `WRITE_SERIAL` enabled)
5. File `adc_capture.bin` is written to the SD root directory
6. Copy the file to your host PC for offline analysis

### Combined USB + SD Logging

Both `WRITE_SERIAL` and `WRITE_LOG` can be enabled simultaneously. In this mode:

- USB streaming provides real-time feedback and live plotting via Python script
- SD file provides a redundant on-device backup at full rate
- Performance depends on both ADC rate and host USB throughput

### Offline Analysis

After capturing to SD, load the `.bin` file with Python:

```python
import numpy as np

# Load raw binary as uint16 array
raw = np.fromfile('adc_capture.bin', dtype=np.uint16)

# Reshape for N_CHANNELS (example: 2 channels)
n_channels = 2
samples_per_ch = len(raw) // n_channels
data = raw.reshape(n_channels, samples_per_ch)
```

## Reference: Backup Implementations

`backup/main_sd_logger.cpp` keeps the older SD-only logging approach for reference. It has been updated with stability fixes but is not the active build target.

## License

See `LICENSE`.
