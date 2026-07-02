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

Requires:  pip install pyserial numpy
"""

import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial


CH_BLOCK_BYTES = 512                        # per-channel block size in firmware
BLOCK_BYTES    = 2 * CH_BLOCK_BYTES         # interleaved A0/A1 payload bytes per transfer block
READ_CHUNK     = 8 * BLOCK_BYTES            # larger host read to reduce Python/USB overhead
SAMPLES_BLOCK  = BLOCK_BYTES // 2           # uint16 words per interleaved block
PREAMBLE_LINES = 2                          # "Streaming…" + "Press 'q'…"


def flush_preamble(ser: "serial.Serial") -> None:
    """Read and discard the text lines the firmware sends before binary data."""
    for _ in range(PREAMBLE_LINES):
        line = ser.readline()
        print("[preamble]", line.decode(errors="replace").rstrip())


def capture(port: str, max_samples: "int | None", out_path: "str | None") -> tuple[np.ndarray, np.ndarray]:
    chunks_a0: list[np.ndarray] = []
    chunks_a1: list[np.ndarray] = []
    total_per_channel = 0

    with serial.Serial(port, baudrate=115200, timeout=10.0) as ser:
        # Disable DTR/RTS to avoid spurious resets.
        ser.dtr = False
        ser.rts = False

        # Drain anything already in the OS buffer.
        ser.reset_input_buffer()

        # Wait for the Teensy's "Enter to" prompt before triggering, so we
        # don't send Enter before the Teensy is ready (e.g. after a reset).
        ser.timeout = 5.0
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                break  # timed out
            print("[boot]", line)
            if "Enter to" in line:
                break

        # Send Enter to trigger the Teensy streaming start.
        # Do not delay here: at >1 Msps, 100 ms can overflow the MCU ring buffer
        # before the host starts draining binary data.
        ser.write(b"\r\n")
        ser.flush()
        ser.timeout = 10.0

        flush_preamble(ser)

        print(f"Capturing from {port}  (Ctrl-C to stop) …")

        partial = bytearray()  # buffer for partial blocks
        timeout_count = 0
        reached_limit = False

        try:
            while True:
                raw = ser.read(READ_CHUNK)
                if len(raw) == 0:
                    timeout_count += 1
                    if timeout_count >= 3:
                        print(f"\nNo data for {timeout_count} read attempts – stopping.")
                        break
                    time.sleep(0.01)
                    continue

                timeout_count = 0  # reset on any data
                partial.extend(raw)

                # Extract complete blocks from partial buffer.
                while len(partial) >= BLOCK_BYTES:
                    block = np.frombuffer(bytes(partial[:BLOCK_BYTES]), dtype=np.uint16)
                    chunks_a0.append(block[0::2])
                    chunks_a1.append(block[1::2])
                    total_per_channel += len(block) // 2
                    del partial[:BLOCK_BYTES]

                    # Progress tick every 256 blocks.
                    if (len(chunks_a0) % 128) == 0:
                        print(f"  {total_per_channel:,} samples/channel collected …", end="\r", flush=True)

                    if max_samples and total_per_channel >= max_samples:
                        print(f"\nReached {max_samples:,} samples/channel limit.")
                        reached_limit = True
                        break

                if reached_limit:
                    break

        except KeyboardInterrupt:
            print(f"\nStopped by user.")
        finally:
            # Send 'q' to tell the Teensy to stop streaming.
            ser.write(b"q")
            ser.flush()
            # Discard any trailing binary data until the stats sentinel.
            ser.timeout = 2.0
            while True:
                line = ser.readline()
                if not line:
                    break
                decoded = line.decode(errors="replace").rstrip()
                if decoded == "---STATS---":
                    break  # found sentinel, stats follow
            while True:
                line = ser.readline()
                if not line:
                    break
                print("[teensy]", line.decode(errors="replace").rstrip())

    if not chunks_a0:
        print("No data received.")
        return np.empty(0, dtype=np.uint16), np.empty(0, dtype=np.uint16)

    samples_a0 = np.concatenate(chunks_a0)
    samples_a1 = np.concatenate(chunks_a1)

    print(f"Captured {len(samples_a0):,} samples/channel  "
          f"({(len(samples_a0) + len(samples_a1)) * 2 / 1024:.1f} kB total)")
    print(f"  A0: min={samples_a0.min()}  max={samples_a0.max()}  "
          f"mean={samples_a0.mean():.1f}  std={samples_a0.std():.1f}")
    print(f"  A1: min={samples_a1.min()}  max={samples_a1.max()}  "
          f"mean={samples_a1.mean():.1f}  std={samples_a1.std():.1f}")

    if out_path:
        np.save(out_path, np.vstack((samples_a0, samples_a1)))
        print(f"Saved -> {out_path}")

    return samples_a0, samples_a1


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture Teensy ADC stream to numpy array.")
    parser.add_argument("port",             nargs="?", default="COM3",
                        help="Serial port (default: COM3)")
    parser.add_argument("--max-samples",    type=int,  default=None,
                        metavar="N",        help="Stop after N samples")
    parser.add_argument("--out",            type=str,  default=None,
                        metavar="FILE.npy", help="Save array to .npy file")
    args = parser.parse_args()

    samples_a0, samples_a1 = capture(args.port, args.max_samples, args.out)

    if len(samples_a0):
        print(f"\nA0 shape: {samples_a0.shape}, dtype: {samples_a0.dtype}")
        print(f"A1 shape: {samples_a1.shape}, dtype: {samples_a1.dtype}")

        volts_a0 = samples_a0 / 2.0**12 * 3.3
        volts_a1 = samples_a1 / 2.0**12 * 3.3

        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(volts_a0, linewidth=0.5, label="A0")
        ax.plot(volts_a1, linewidth=0.5, label="A1")
        ax.set_xlabel("Sample index")
        ax.set_ylabel("Voltage (V)")
        ax.set_ylim(0, 3.5)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_title(f"ADC capture  –  {len(samples_a0):,} samples/channel")

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
