/*
 * @file main.cpp
 * @brief Teensy ADC capture with selectable USB streaming and SD logging outputs.
 *
 * Modes:
 * - N_CHANNELS <= 2: high-speed DMA capture (A0 on adc0, A1 on adc1 when enabled)
 * - N_CHANNELS > 2: round-robin scan capture across selected analog pins
 */

#include "ADC.h"
#include "DMAChannel.h"
#include "RingBuf.h"
#include "SdFat.h"
#include "version.h"

#include <string.h>

#if defined(__IMXRT1062__)
  #define SOURCE0_SADDR ADC1_R0
  #define SOURCE0_EVENT DMAMUX_SOURCE_ADC1
  #define SOURCE1_SADDR ADC2_R0
  #define SOURCE1_EVENT DMAMUX_SOURCE_ADC2
#else
  #error "This project targets Teensy 4.x (__IMXRT1062__)."
#endif

#define SD_CONFIG SdioConfig(FIFO_SDIO)

const bool WRITE_LOG = false;
const bool WRITE_SERIAL = true;
const size_t N_CHANNELS = 2;
const size_t CLOCK_SPEED_MHZ = 600; // up to 916;

const size_t BUF_BLOCK_SIZE = 512; // Must be multiple of 32 for DMA and cache alignment.  Each channel has its own buffer.
const size_t RING_BUF_SIZE = 480 * BUF_BLOCK_SIZE; // Per-channel ring buffer size.  Placed in OCRAM; sized for ~200 ms of USB stall headroom.
const size_t WARMUP_BLOCKS = 1;                    // Discard startup transient (256 samples/block)
const size_t MAX_CHANNELS = 14;
const uint32_t LOG_ONLY_RUN_US = 10U * 1000000U;

static_assert(N_CHANNELS >= 1 && N_CHANNELS <= MAX_CHANNELS, "N_CHANNELS must be in [1, 14]");

const size_t ACTIVE_DMA_CHANNELS = (N_CHANNELS >= 2) ? 2 : 1;
const size_t SAMPLES_PER_CH_BLOCK = BUF_BLOCK_SIZE / sizeof(uint16_t);
const size_t TX_BLOCK_BYTES = ACTIVE_DMA_CHANNELS * BUF_BLOCK_SIZE; // A* interleaved uint16 stream.

const uint8_t ANALOG_PINS[MAX_CHANNELS] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13
};

volatile uint32_t dmaCount0;
volatile uint32_t dmaCount1;
volatile bool overrun;
volatile size_t maxBytesUsed0;
volatile size_t maxBytesUsed1;

ADC adc;
DMAChannel dma0(true);
DMAChannel dma1(true);
DMAMEM static uint16_t __attribute__((aligned(32))) dmaBuf0[SAMPLES_PER_CH_BLOCK];
DMAMEM static uint16_t __attribute__((aligned(32))) dmaBuf1[SAMPLES_PER_CH_BLOCK];
DMAMEM static RingBuf<char, RING_BUF_SIZE> rb0; // OCRAM: keeps large buffers out of DTCM
DMAMEM static RingBuf<char, RING_BUF_SIZE> rb1;

SdFs sd;
FsFile logFile;

static bool openLogFile() {
  if (!WRITE_LOG) {
    return false;
  }
  if (!sd.begin(SD_CONFIG)) {
    if (WRITE_SERIAL) {
      Serial.println("sd.begin failed, continuing without log");
    }
    return false;
  }
  if (!logFile.open("adc_capture.bin", O_CREAT | O_TRUNC | O_RDWR)) {
    if (WRITE_SERIAL) {
      Serial.println("logFile.open failed, continuing without log");
    }
    return false;
  }
  return true;
}

static void closeLogFile(bool enabled) {
  if (!enabled) {
    return;
  }
  logFile.sync();
  logFile.close();
}

static void isr0() {
  if (!overrun) {
    arm_dcache_delete((void*)dmaBuf0, BUF_BLOCK_SIZE);
    rb0.beginISR();
    if (rb0.write(dmaBuf0, BUF_BLOCK_SIZE) == BUF_BLOCK_SIZE) {
      dmaCount0++;
      size_t used = rb0.bytesUsed();
      if (used > maxBytesUsed0) {
        maxBytesUsed0 = used;
      }
    } else {
      overrun = true;
    }
    rb0.endISR();
  }

  dma0.clearComplete();
  dma0.clearInterrupt();
#if defined(__IMXRT1062__)
  asm("DSB");
#endif
}

static void isr1() {
  if (ACTIVE_DMA_CHANNELS < 2) {
    dma1.clearComplete();
    dma1.clearInterrupt();
#if defined(__IMXRT1062__)
    asm("DSB");
#endif
    return;
  }

  if (!overrun) {
    arm_dcache_delete((void*)dmaBuf1, BUF_BLOCK_SIZE);
    rb1.beginISR();
    if (rb1.write(dmaBuf1, BUF_BLOCK_SIZE) == BUF_BLOCK_SIZE) {
      dmaCount1++;
      size_t used = rb1.bytesUsed();
      if (used > maxBytesUsed1) {
        maxBytesUsed1 = used;
      }
    } else {
      overrun = true;
    }
    rb1.endISR();
  }

  dma1.clearComplete();
  dma1.clearInterrupt();
#if defined(__IMXRT1062__)
  asm("DSB");
#endif
}

static void initDmaAdcDual(uint8_t pin0, uint8_t pin1) {
  dma0.begin();
  dma0.attachInterrupt(isr0);
  dma0.source((volatile const signed short&)SOURCE0_SADDR);
  dma0.destinationBuffer((volatile uint16_t*)dmaBuf0, sizeof(dmaBuf0));
  dma0.interruptAtCompletion();
  dma0.triggerAtHardwareEvent(SOURCE0_EVENT);
  dma0.enable();

  if (ACTIVE_DMA_CHANNELS >= 2) {
    dma1.begin();
    dma1.attachInterrupt(isr1);
    dma1.source((volatile const signed short&)SOURCE1_SADDR);
    dma1.destinationBuffer((volatile uint16_t*)dmaBuf1, sizeof(dmaBuf1));
    dma1.interruptAtCompletion();
    dma1.triggerAtHardwareEvent(SOURCE1_EVENT);
    dma1.enable();
  }

  adc.adc0->enableDMA();
  adc.adc0->startContinuous(pin0);
  if (ACTIVE_DMA_CHANNELS >= 2) {
    adc.adc1->enableDMA();
    adc.adc1->startContinuous(pin1);
  }
}

static void stopDma() {
  adc.adc0->disableDMA();
  dma0.disable();
  if (ACTIVE_DMA_CHANNELS >= 2) {
    adc.adc1->disableDMA();
    dma1.disable();
  }
}

static void waitSerial(const char* msg, bool repeat = false) {
  while (Serial.read() >= 0) {
    delay(1);
  }
  Serial.println(msg);
  uint32_t lastPrintMs = millis();
  while (!Serial.available()) {
    yield();
    delay(1);
    if (repeat && (millis() - lastPrintMs) >= 500) {
      Serial.println(msg);
      lastPrintMs = millis();
    }
  }
  while (Serial.read() >= 0) {
    delay(1);
  }
}

static void runDmaMode(uint8_t pin0, uint8_t pin1) {
  dmaCount0 = 0;
  dmaCount1 = 0;
  maxBytesUsed0 = 0;
  maxBytesUsed1 = 0;
  overrun = false;

  rb0.begin(nullptr);
  if (ACTIVE_DMA_CHANNELS >= 2) {
    rb1.begin(nullptr);
  }

  bool logEnabled = openLogFile();

  if (WRITE_SERIAL) {
    if (ACTIVE_DMA_CHANNELS >= 2) {
      Serial.println("Streaming interleaved binary uint16_t samples: A0,A1,... over USB Serial...");
    } else {
      Serial.println("Streaming binary uint16_t samples: A0 over USB Serial...");
    }
    Serial.println("Press 'q' to stop.");
  }

  while (Serial.read() >= 0) {}

  initDmaAdcDual(pin0, pin1);

  uint8_t discardBuf[BUF_BLOCK_SIZE];
  size_t discarded = 0;
  while (!overrun && discarded < WARMUP_BLOCKS) {
    const bool ready = (ACTIVE_DMA_CHANNELS >= 2)
      ? (rb0.bytesUsed() >= BUF_BLOCK_SIZE && rb1.bytesUsed() >= BUF_BLOCK_SIZE)
      : (rb0.bytesUsed() >= BUF_BLOCK_SIZE);

    if (ready) {
      if (rb0.read(discardBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        overrun = true;
        break;
      }
      if (ACTIVE_DMA_CHANNELS >= 2 && rb1.read(discardBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        overrun = true;
        break;
      }
      discarded++;
    } else {
      yield();
    }
  }

  uint16_t ch0Buf[SAMPLES_PER_CH_BLOCK];
  uint16_t ch1Buf[SAMPLES_PER_CH_BLOCK];
  uint16_t txSamples[2 * SAMPLES_PER_CH_BLOCK];
  uint8_t txBuf[BUF_BLOCK_SIZE];

  if (WRITE_SERIAL) {
    Serial.println("---BEGIN BINARY---");
  }

  const uint32_t startUs = micros();
  bool stopRequested = false;

  while (!overrun) {
    if (WRITE_SERIAL) {
      int ch;
      while ((ch = Serial.read()) >= 0) {
        if (ch == 'q') {
          stopRequested = true;
          goto stream_done;
        }
      }
    }

    const bool ready = (ACTIVE_DMA_CHANNELS >= 2)
      ? (rb0.bytesUsed() >= BUF_BLOCK_SIZE && rb1.bytesUsed() >= BUF_BLOCK_SIZE)
      : (rb0.bytesUsed() >= BUF_BLOCK_SIZE);

    if (!ready) {
      yield();
      continue;
    }

    if (rb0.read(ch0Buf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
      overrun = true;
      break;
    }
    if (ACTIVE_DMA_CHANNELS >= 2 && rb1.read(ch1Buf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
      overrun = true;
      break;
    }

    const uint8_t* txPtr = nullptr;
    if (ACTIVE_DMA_CHANNELS >= 2) {
      for (size_t i = 0; i < SAMPLES_PER_CH_BLOCK; i++) {
        txSamples[2 * i] = ch0Buf[i];
        txSamples[2 * i + 1] = ch1Buf[i];
      }
      txPtr = reinterpret_cast<const uint8_t*>(txSamples);
    } else {
      memcpy(txBuf, ch0Buf, BUF_BLOCK_SIZE);
      txPtr = txBuf;
    }

    if (WRITE_SERIAL) {
      size_t sent = 0;
      while (sent < TX_BLOCK_BYTES) {
        int ctrl;
        while ((ctrl = Serial.read()) >= 0) {
          if (ctrl == 'q') {
            stopRequested = true;
            break;
          }
        }
        if (stopRequested) {
          break;
        }

        const size_t w = Serial.write(txPtr + sent, TX_BLOCK_BYTES - sent);
        if (w == 0) {
          yield();
          continue;
        }
        sent += w;
      }
      if (stopRequested) {
        goto stream_done;
      }
    }

    if (logEnabled) {
      if (logFile.write(txPtr, TX_BLOCK_BYTES) != TX_BLOCK_BYTES) {
        overrun = true;
        break;
      }
    }

    if (!WRITE_SERIAL && (micros() - startUs) >= LOG_ONLY_RUN_US) {
      stopRequested = true;
      goto stream_done;
    }
  }

stream_done:
  stopDma();
  const uint32_t elapsedUs = micros() - startUs;
  closeLogFile(logEnabled);

  while (Serial.read() >= 0) {
    delay(1);
  }

  if (WRITE_SERIAL) {
    Serial.println("\n---STATS---");
    if (overrun) {
      Serial.println("Overrun ERROR: Ring buffer overflow.");
    }

    const uint32_t blocks = (ACTIVE_DMA_CHANNELS >= 2)
      ? ((dmaCount0 < dmaCount1) ? dmaCount0 : dmaCount1)
      : dmaCount0;
    const float samplesPerChannel = blocks * SAMPLES_PER_CH_BLOCK;
    const float timeSeconds = 1e-6f * elapsedUs;
    const float chRate = (timeSeconds > 0.0f) ? (samplesPerChannel / timeSeconds) : 0.0f;

    Serial.printf("dmaCount0: %u\n", (unsigned)dmaCount0);
    if (ACTIVE_DMA_CHANNELS >= 2) {
      Serial.printf("dmaCount1: %u\n", (unsigned)dmaCount1);
    }
    Serial.printf("maxBytesUsed0: %u\n", (unsigned)maxBytesUsed0);
    if (ACTIVE_DMA_CHANNELS >= 2) {
      Serial.printf("maxBytesUsed1: %u\n", (unsigned)maxBytesUsed1);
    }
    Serial.printf("elapsed: %.3f s\n", 1e-6f * elapsedUs);
    Serial.printf("avg sample rate/ch: %.1f ksamp/sec\n", chRate / 1000.0f);
    Serial.println();
  }
}

static void runScanMode() {
  overrun = false;
  bool logEnabled = openLogFile();

  if (WRITE_SERIAL) {
    Serial.printf("Streaming %u-channel round-robin uint16 scan over USB Serial...\n", (unsigned)N_CHANNELS);
    Serial.println("Frame format: ch0,ch1,...,chN-1 repeated");
    Serial.println("Press 'q' to stop.");
  }

  while (Serial.read() >= 0) {}

  uint16_t frame[MAX_CHANNELS];
  const size_t frameBytes = N_CHANNELS * sizeof(uint16_t);

  // Warmup one frame per active channel.
  for (size_t k = 0; k < WARMUP_BLOCKS; k++) {
    for (size_t i = 0; i < N_CHANNELS; i++) {
      (void)analogRead(ANALOG_PINS[i]);
    }
  }

  if (WRITE_SERIAL) {
    Serial.println("---BEGIN BINARY---");
  }

  const uint32_t startUs = micros();
  uint32_t frameCount = 0;
  bool stopRequested = false;

  while (!overrun && !stopRequested) {
    if (WRITE_SERIAL) {
      int ch;
      while ((ch = Serial.read()) >= 0) {
        if (ch == 'q') {
          stopRequested = true;
          break;
        }
      }
    }

    if (stopRequested) {
      break;
    }

    for (size_t i = 0; i < N_CHANNELS; i++) {
      frame[i] = (uint16_t)analogRead(ANALOG_PINS[i]);
    }

    const uint8_t* framePtr = reinterpret_cast<const uint8_t*>(frame);

    if (WRITE_SERIAL) {
      size_t sent = 0;
      while (sent < frameBytes) {
        const size_t w = Serial.write(framePtr + sent, frameBytes - sent);
        if (w == 0) {
          yield();
          continue;
        }
        sent += w;
      }
    }

    if (logEnabled) {
      if (logFile.write(framePtr, frameBytes) != frameBytes) {
        overrun = true;
        break;
      }
    }

    frameCount++;

    if (!WRITE_SERIAL && (micros() - startUs) >= LOG_ONLY_RUN_US) {
      break;
    }
  }

  closeLogFile(logEnabled);
  const uint32_t elapsedUs = micros() - startUs;

  if (WRITE_SERIAL) {
    Serial.println("\n---STATS---");
    if (overrun) {
      Serial.println("Overrun ERROR: output path stalled.");
    }
    const float timeSeconds = 1e-6f * elapsedUs;
    const float chRate = (timeSeconds > 0.0f) ? (frameCount / timeSeconds) : 0.0f;
    Serial.printf("frames: %u\n", (unsigned)frameCount);
    Serial.printf("elapsed: %.3f s\n", 1e-6f * elapsedUs);
    Serial.printf("avg sample rate/ch: %.1f ksamp/sec\n", chRate / 1000.0f);
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 3000) {
    yield();
  }

  Serial.println("ADC DMA USB Streamer v" VERSION_STRING);
  Serial.printf("CPU clock: %u MHz\n", (unsigned)CLOCK_SPEED_MHZ);

  adc.adc0->setAveraging(0);
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

  if (ACTIVE_DMA_CHANNELS >= 2) {
    adc.adc1->setAveraging(0);
    adc.adc1->setResolution(12);
    adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  }

  if (WRITE_SERIAL) {
    if (N_CHANNELS > 2) {
      Serial.println("N_CHANNELS > 2 uses round-robin scan mode (lower per-channel rate).");
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "Enter to begin streaming %u channels", (unsigned)N_CHANNELS);
    waitSerial(msg, /*repeat=*/true);
  }
}

void loop() {
  if (N_CHANNELS <= 2) {
    runDmaMode(ANALOG_PINS[0], ANALOG_PINS[1]);
  } else {
    runScanMode();
  }

  if (WRITE_SERIAL) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Enter to stream again (%u channels)", (unsigned)N_CHANNELS);
    waitSerial(msg, /*repeat=*/true);
  }
}
