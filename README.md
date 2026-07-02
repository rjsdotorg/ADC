# ADC

Dual-channel Teensy 4.0 ADC capture project with high-rate USB streaming and Python-based capture/plot tools.

This project samples:

- `A0` on `adc0`
- `A1` on `adc1`

and streams interleaved `uint16` samples over native USB Serial to a host PC.

The current firmware is designed for:

- high sustained capture rate
- minimal ISR work
- DMA-backed acquisition
- buffered host streaming
- Python decoding into NumPy arrays

## Overview

The active firmware is in `src/main.cpp`.

It uses:

- one DMA channel per ADC module
- one ring buffer per channel
- interleaved USB output format: `A0[0], A1[0], A0[1], A1[1], ...`
- a short warmup discard so captures start after ADC startup transient

The host-side script is `tools/serial_capture.py`.

It:

- waits for the Teensy prompt
- starts capture
- reads the binary stream in large chunks
- decodes interleaved samples into separate `A0` and `A1` arrays
- prints capture statistics
- optionally saves data to `.npy`
- plots both channels on the same axes

## Hardware

- Teensy 4.0
- Analog inputs on `A0` and `A1`
- USB connection to host PC

Notes:

- Teensy USB CDC does not use the configured monitor baud as a physical UART rate.
- The `115200` setting in `platformio.ini` is only for host-tool consistency.

## Stream Format

Firmware emits raw little-endian `uint16` values over USB in this order:

```text
A0[0], A1[0], A0[1], A1[1], A0[2], A1[2], ...
```

Each ADC sample is 12-bit data stored in a 16-bit word.

The Python script splits the stream as:

- `A0 = block[0::2]`
- `A1 = block[1::2]`

Saved `.npy` output is a stacked array with shape `(2, N)`:

- row `0`: `A0`
- row `1`: `A1`

## Project Layout

- `src/main.cpp`: active dual-channel USB streaming firmware
- `tools/serial_capture.py`: host capture and plotting script
- `backup/main_sd_logger.cpp`: SD logging variant retained for reference
- `platformio.ini`: PlatformIO environment and monitor configuration

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

Example:

```powershell
python tools/serial_capture.py COM3 --max-samples 1000000 --out adc.npy
```

Arguments:

- `COM3`: serial port
- `--max-samples N`: stop after `N` samples per channel
- `--out FILE.npy`: save stacked `(2, N)` array

The script will:

1. wait for the firmware prompt
2. trigger streaming
3. capture binary data
4. stop the firmware with `q`
5. print Teensy-side statistics
6. plot `A0` and `A1`

## ADC Configuration

Current configuration is set in `src/main.cpp`:

- 12-bit resolution
- averaging configurable in firmware
- conversion speed configurable in firmware
- sampling speed configurable in firmware

Example high-speed settings used during testing:

```cpp
adc.adc0->setAveraging(0);
adc.adc0->setResolution(12);
adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

adc.adc1->setAveraging(0);
adc.adc1->setResolution(12);
adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
```

## Performance Notes

Observed operation has reached roughly `1.3 Msps/channel` in the current dual-channel USB streaming configuration, depending on:

- ADC speed settings
- host USB/serial performance
- Python read overhead
- ring buffer size

Important practical limits:

- Higher ADC rates increase risk of ring-buffer overflow.
- Host-side delays at startup or shutdown can cause overrun even if the main capture is otherwise valid.
- The USB streaming path is convenient, but SD logging may be better for maximum sustained capture margin.

## Startup Behavior

The first ADC samples after enabling continuous conversion may show a short transient.

To avoid that in the exported capture, firmware discards a small number of startup blocks before streaming begins.

## Troubleshooting

### Overrun ERROR: Ring buffer overflow

This means the host did not drain incoming data fast enough.

Ways to improve margin:

- reduce ADC speed slightly
- increase ring buffer size in firmware
- keep Python running locally without extra serial monitor tools attached
- avoid delays between start trigger and binary reads

### Sample values look wrong

Check:

- input wiring on `A0` and `A1`
- expected voltage range for 12-bit conversion
- whether the stream decoder matches the current interleaved wire format

Sanity checks:

- `A0` or `A1` tied to GND should read near `0`
- tied to `3.3V` should read near `4095`
- near mid-rail should read near `2048`

### Serial monitor reconnect/reset issues

This project disables monitor `DTR` and `RTS` in `platformio.ini` to avoid unnecessary Teensy resets when attaching host tools.

## SD Logging Variant

`backup/main_sd_logger.cpp` keeps the older SD card logging approach for reference. It has been updated with the same DMA/buffer stability fixes where practical, but it is not the active build target.

## License

See `LICENSE`.
