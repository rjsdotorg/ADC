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
PREAMBLE_LINES = 2                          # "Streaming…" + "Press 'q'…"


def flush_preamble(ser: "serial.Serial") -> None:
    """Read and discard the text lines the firmware sends before binary data."""
    for _ in range(PREAMBLE_LINES):
        line = ser.readline()
        print("[preamble]", line.decode(errors="replace").rstrip())


def capture(port: str, n_chan: int, max_samples: "int | None", out_path: "str | None") -> list[np.ndarray]:
    block_bytes = n_chan * CH_BLOCK_BYTES
    read_chunk = 8 * block_bytes

    channel_chunks: list[list[np.ndarray]] = [[] for _ in range(n_chan)]
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
                raw = ser.read(read_chunk)
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
                while len(partial) >= block_bytes:
                    block = np.frombuffer(bytes(partial[:block_bytes]), dtype=np.uint16)
                    for ch_idx in range(n_chan):
                        channel_chunks[ch_idx].append(block[ch_idx::n_chan])
                    total_per_channel += len(block) // n_chan
                    del partial[:block_bytes]

                    # Progress tick every 256 blocks.
                    if (len(channel_chunks[0]) % 128) == 0:
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

    if not channel_chunks[0]:
        print("No data received.")
        return [np.empty(0, dtype=np.uint16) for _ in range(n_chan)]

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
                        metavar="N",        help="Stop after N samples")
    parser.add_argument("--n-chan",         type=int,  default=2,
                        metavar="N",        help="Number of interleaved channels in stream (default: 2)")
    parser.add_argument("--out",            type=str,  default=None,
                        metavar="FILE.npy", help="Save array to .npy file")
    args = parser.parse_args()

    if args.n_chan < 1:
        raise ValueError("--n-chan must be >= 1")

    channel_samples = capture(args.port, args.n_chan, args.max_samples, args.out)

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
