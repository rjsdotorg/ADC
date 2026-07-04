"""
serial_capture.py  –  Capture Teensy dual-ADC DMA USB streamer data.

Firmware stream format is interleaved uint16 words:
    A0[0], A1[0], A0[1], A1[1], ...

This script:
    1. Opens the port and sends Enter to start streaming.
    2. Discards text preamble lines sent before binary data begins.
    3. Reads interleaved binary blocks and splits into A0/A1 arrays.
    4. Stops on Ctrl-C and reports per-channel stats.

Usage:
        python serial_capture.py [PORT] [--max-samples N] [--out FILE.npy]

    --max-samples N  stops after N samples per channel (not total across all channels)

    Channel count is auto-detected from the firmware prompt.

Requires:  pip install pyserial numpy
"""

import argparse
import re
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial


CH_BLOCK_BYTES = 512                        # per-channel block size in firmware
BINARY_SENTINEL = "---BEGIN BINARY---"
STATS_SENTINEL_BYTES = b"---STATS---"


def parse_channels_from_prompt(line: str) -> "int | None":
    """Extract channel count from firmware prompt lines that mention '<N> channels'."""
    match = re.search(r'(\d+)\s+channels', line, re.IGNORECASE)
    return int(match.group(1)) if match else None


def scan_to_binary_start(ser: "serial.Serial") -> None:
    """Read and print text lines until the binary-start sentinel is received."""
    while True:
        line = ser.readline().decode(errors="replace").rstrip()
        if not line:
            raise RuntimeError("Timed out waiting for ---BEGIN BINARY--- sentinel from firmware")
        print("[preamble]", line)
        if line == BINARY_SENTINEL:
            return


def capture(port: str, max_samples: "int | None", out_path: "str | None") -> list[np.ndarray]:
    """Capture interleaved stream from Teensy, auto-detecting channel count from the firmware prompt."""
    n_chan_detected = None

    with serial.Serial(port, baudrate=115200, timeout=10.0) as ser:
        # Disable DTR/RTS to avoid spurious resets.
        ser.dtr = False
        ser.rts = False

        ser.reset_input_buffer()

        # Wait for the Teensy's "Enter to begin streaming N channels" prompt.
        ser.timeout = 5.0
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                break  # timed out
            print("[boot]", line)

            # Try to extract channel count from firmware prompt.
            if "Enter to" in line:
                detected = parse_channels_from_prompt(line)
                if detected is not None:
                    n_chan_detected = detected
                    print(f"[auto] Detected N_CHANNELS={n_chan_detected} from firmware prompt")
                break

        if n_chan_detected is None:
            n_chan_detected = 2  # fallback default
            print(f"[auto] Could not parse channel count; using default {n_chan_detected}")

        block_bytes = n_chan_detected * CH_BLOCK_BYTES
        read_chunk = 8 * block_bytes

        channel_chunks: list[list[np.ndarray]] = [[] for _ in range(n_chan_detected)]
        total_per_channel = 0

        # Send Enter to trigger the Teensy streaming start.
        # Do not delay here: at >1 Msps, 100 ms can overflow the MCU ring buffer
        # before the host starts draining binary data.
        ser.write(b"\r\n")
        ser.flush()
        ser.timeout = 10.0

        scan_to_binary_start(ser)

        print(f"Capturing from {port}  (Ctrl-C to stop) …")

        partial = bytearray()  # buffer for partial blocks
        timeout_count = 0
        reached_limit = False
        firmware_stopped = False

        try:
            while True:
                # Read all bytes already in the OS buffer to prevent backpressure.
                avail = ser.in_waiting
                raw = ser.read(avail if avail > read_chunk else read_chunk)
                if len(raw) == 0:
                    timeout_count += 1
                    if timeout_count >= 3:
                        print(f"\nNo data for {timeout_count} read attempts – stopping.")
                        break
                    time.sleep(0.01)
                    continue

                timeout_count = 0  # reset on any data
                partial.extend(raw)

                # Firmware prints text stats when streaming stops (normal stop or overrun).
                # Stop binary parsing at that boundary to avoid treating ASCII as ADC samples.
                stats_idx = partial.find(STATS_SENTINEL_BYTES)
                if stats_idx != -1:
                    stream_bytes = bytes(partial[:stats_idx])
                    partial.clear()
                    partial.extend(stream_bytes)
                    firmware_stopped = True

                # Extract complete blocks from partial buffer.
                while len(partial) >= block_bytes:
                    block = np.frombuffer(bytes(partial[:block_bytes]), dtype=np.uint16)
                    for ch_idx in range(n_chan_detected):
                        channel_chunks[ch_idx].append(block[ch_idx::n_chan_detected])
                    total_per_channel += len(block) // n_chan_detected
                    del partial[:block_bytes]

                    # Progress tick every 256 blocks.
                    if (len(channel_chunks[0]) % 128) == 0:
                        print(f"  {total_per_channel:,} samples/channel collected …", end="\r", flush=True)

                    if max_samples and total_per_channel >= max_samples:
                        print(f"\nReached {max_samples:,} samples/channel limit.")
                        reached_limit = True
                        break

                if reached_limit or firmware_stopped:
                    if firmware_stopped:
                        print("\n[info] Firmware ended stream; stopping binary capture.")
                    break

        except KeyboardInterrupt:
            print(f"\nStopped by user.")
        finally:
            # Send 'q' to tell the Teensy to stop streaming.
            ser.write(b"q")
            ser.flush()
            # Discard any trailing binary data until the stats sentinel.
            ser.timeout = 2.0
            saw_stats = False
            while True:
                line = ser.readline()
                if not line:
                    break
                decoded = line.decode(errors="replace").rstrip()
                if decoded == "---STATS---":
                    saw_stats = True
                    break  # found sentinel, stats follow
            if saw_stats:
                # Read only the current stats block. Stop at the next prompt to avoid
                # hanging when firmware repeats "Enter to stream again" indefinitely.
                while True:
                    line = ser.readline()
                    if not line:
                        break
                    decoded = line.decode(errors="replace").rstrip()
                    if decoded.startswith("Enter to stream again"):
                        break
                    print("[teensy]", decoded)

    if not channel_chunks[0]:
        print("No data received.")
        return [np.empty(0, dtype=np.uint16) for _ in range(n_chan_detected)]

    channel_samples = [np.concatenate(ch_chunks) for ch_chunks in channel_chunks]

    total_samples = sum(len(s) for s in channel_samples)
    print(f"Captured {len(channel_samples[0]):,} samples/channel  "
          f"({total_samples * 2 / 1024:.1f} kB total)")
    for ch_idx, samples in enumerate(channel_samples):
        print(f"  A{ch_idx}: min={samples.min()}  max={samples.max()}  "
              f"mean={samples.mean():.1f}  std={samples.std():.1f}")

    if out_path:
        np.save(out_path, np.vstack(channel_samples))
        print(f"Saved -> {out_path}")

    return channel_samples


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture Teensy ADC stream to numpy array.")
    parser.add_argument("port",             nargs="?", default="COM3",
                        help="Serial port (default: COM3)")
    parser.add_argument("--max-samples",    type=int,  default=None,
                        metavar="SAMPLES",  help="Stop after N samples per channel")
    parser.add_argument("--out",            type=str,  default=None,
                        metavar="FILE.npy", help="Save array to .npy file")
    args = parser.parse_args()

    channel_samples = capture(args.port, args.max_samples, args.out)

    if len(channel_samples[0]):
        for ch_idx, samples in enumerate(channel_samples):
            print(f"A{ch_idx} shape: {samples.shape}, dtype: {samples.dtype}")

        volts = [samples / 2.0**12 * 3.3 for samples in channel_samples]

        fig, ax = plt.subplots(figsize=(12, 4))
        for ch_idx, ch_volts in enumerate(volts):
            ax.plot(ch_volts, linewidth=0.5, label=f"A{ch_idx}")
        ax.set_xlabel("Sample index")
        ax.set_ylabel("Voltage (V)")
        ax.set_ylim(0, 3.5)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_title(f"ADC capture  –  {len(channel_samples[0]):,} samples/channel")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
