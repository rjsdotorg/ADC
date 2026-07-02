/*
 * @file main.cpp
 * @brief Teensy ADC DMA streamer to USB Serial (no SD card writes).
 *
 * Captures ADC samples via DMA into a ping-pong buffer, stores in a large RingBuf,
 * and drains to USB Serial from the main loop.
 */
#include "ADC.h"
#include "DMAChannel.h"
#include "RingBuf.h"
#include "version.h"
#include <string.h>

#define ADC0_PIN A0
#define ADC1_PIN A1

#if defined(__IMXRT1062__)
  #define SOURCE0_SADDR ADC1_R0
  #define SOURCE0_EVENT DMAMUX_SOURCE_ADC1
  #define SOURCE1_SADDR ADC2_R0
  #define SOURCE1_EVENT DMAMUX_SOURCE_ADC2
#else
  #error "This dual-ADC streamer is implemented for Teensy 4.x (__IMXRT1062__)"
#endif

const size_t BUF_BLOCK_SIZE = 512;
const size_t RING_BUF_SIZE = 200 * BUF_BLOCK_SIZE;  // Per-channel ring buffer size.
const size_t WARMUP_BLOCKS = 1;                     // Discard startup transient (256 samples/block)
const size_t SAMPLES_PER_CH_BLOCK = BUF_BLOCK_SIZE / sizeof(uint16_t);
const size_t TX_BLOCK_BYTES = 2 * BUF_BLOCK_SIZE;   // A0/A1 interleaved uint16 stream.

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
RingBuf<char, RING_BUF_SIZE> rb0;
RingBuf<char, RING_BUF_SIZE> rb1;

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

static void init_dma_adc_dual(uint8_t pin0, uint8_t pin1) {
  dma0.begin();
  dma0.attachInterrupt(isr0);
  dma0.source((volatile const signed short&)SOURCE0_SADDR);
  dma0.destinationBuffer((volatile uint16_t*)dmaBuf0, sizeof(dmaBuf0));
  dma0.interruptAtCompletion();
  dma0.triggerAtHardwareEvent(SOURCE0_EVENT);
  dma0.enable();

  dma1.begin();
  dma1.attachInterrupt(isr1);
  dma1.source((volatile const signed short&)SOURCE1_SADDR);
  dma1.destinationBuffer((volatile uint16_t*)dmaBuf1, sizeof(dmaBuf1));
  dma1.interruptAtCompletion();
  dma1.triggerAtHardwareEvent(SOURCE1_EVENT);
  dma1.enable();

  adc.adc0->enableDMA();
  adc.adc1->enableDMA();
  adc.adc0->startContinuous(pin0);
  adc.adc1->startContinuous(pin1);
}

static void stopDma() {
  adc.adc0->disableDMA();
  adc.adc1->disableDMA();
  dma0.disable();
  dma1.disable();
}

static void waitSerial(const char* msg) {
  while (Serial.read() >= 0) {
    delay(1);
  }
  Serial.println(msg);
  while (!Serial.available()) {
    yield();
    delay(1);
  }
  while (Serial.read() >= 0) {
    delay(1);
  }
}

static void runStream(uint8_t pin0, uint8_t pin1) {
  dmaCount0 = 0;
  dmaCount1 = 0;
  maxBytesUsed0 = 0;
  maxBytesUsed1 = 0;
  overrun = false;

  rb0.begin(nullptr);  // Initialize ring buffer without file backing
  rb1.begin(nullptr);

  Serial.println("Streaming interleaved binary uint16_t samples: A0,A1,... over USB Serial...");
  Serial.println("Press 'q' to stop.");

  // Drain any residual bytes before starting.
  while (Serial.read() >= 0) {}

  // Start ADC+DMA only after preamble/housekeeping so high-rate captures
  // don't overrun before the host starts reading binary data.
  init_dma_adc_dual(pin0, pin1);

  // Drop initial ADC startup transient so capture starts near steady-state.
  uint8_t discardBuf[BUF_BLOCK_SIZE];
  size_t discarded = 0;
  while (!overrun && discarded < WARMUP_BLOCKS) {
    if (rb0.bytesUsed() >= BUF_BLOCK_SIZE && rb1.bytesUsed() >= BUF_BLOCK_SIZE) {
      if (rb0.read(discardBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("warmup discard A0 failed");
        overrun = true;
        break;
      }
      if (rb1.read(discardBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("warmup discard A1 failed");
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
  uint32_t startUs = micros();
  bool stopRequested = false;

  while (!overrun) {
    // Drain non-'q' bytes; only stop on explicit 'q'.
    int ch;
    while ((ch = Serial.read()) >= 0) {
      if (ch == 'q') {
        stopRequested = true;
        goto stream_done;
      }
    }

    if (rb0.bytesUsed() >= BUF_BLOCK_SIZE && rb1.bytesUsed() >= BUF_BLOCK_SIZE) {
      if (rb0.read(ch0Buf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("rb0.read failed");
        break;
      }
      if (rb1.read(ch1Buf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("rb1.read failed");
        break;
      }

      // Interleave channel data on wire: [A0[0],A1[0], A0[1],A1[1], ...].
      for (size_t i = 0; i < SAMPLES_PER_CH_BLOCK; i++) {
        txSamples[2 * i] = ch0Buf[i];
        txSamples[2 * i + 1] = ch1Buf[i];
      }

      const uint8_t* txPtr = reinterpret_cast<const uint8_t*>(txSamples);
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

        size_t w = Serial.write(txPtr + sent, TX_BLOCK_BYTES - sent);
        if (w == 0) {
          yield();
          continue;
        }
        sent += w;
      }
      if (stopRequested) {
        goto stream_done;
      }

    } else {
      yield();
    }
  }
  Serial.println("[Stream ended: overrun triggered]");
  stream_done:
  stopDma();
  uint32_t elapsedUs = micros() - startUs;

  while (Serial.read() >= 0) {
    delay(1);
  }

  // Sentinel line: Python discards binary tail up to this.
  Serial.println("\n---STATS---");
  if (overrun) {
    Serial.println("Overrun ERROR: Ring buffer overflow.");
  }

  uint32_t blocks = dmaCount0 < dmaCount1 ? dmaCount0 : dmaCount1;
  float samplesPerChannel = blocks * SAMPLES_PER_CH_BLOCK;
  float timeSeconds = 1e-6f * elapsedUs;
  float chRate = samplesPerChannel / timeSeconds;

  Serial.printf("dmaCount0: %u\n", (unsigned)dmaCount0);
  Serial.printf("dmaCount1: %u\n", (unsigned)dmaCount1);
  Serial.printf("maxBytesUsed0: %u\n", (unsigned)maxBytesUsed0);
  Serial.printf("maxBytesUsed1: %u\n", (unsigned)maxBytesUsed1);
  Serial.printf("elapsed: %.3f s\n", 1e-6f * elapsedUs);
  Serial.printf("avg sample rate/ch: %.1f ksamp/sec\n", chRate / 1000.0f);

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 3000) {
    yield();
  }

  Serial.println("ADC DMA USB Streamer v" VERSION_STRING);

  adc.adc0->setAveraging(0);
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

  adc.adc1->setAveraging(0);
  adc.adc1->setResolution(12);
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

  waitSerial("Enter to begin streaming");
}

void loop() {
  runStream(ADC0_PIN, ADC1_PIN);
  waitSerial("Enter to stream again");
}
