#/***************************************************************
 * @file main.cpp
 * @brief Teensy exFAT DMA ADC logger example with RingBuf.
 *
 * This sketch demonstrates using ADC + DMA to capture samples into
 * a single DMA buffer and persist 512-byte-multiple sectors to an exFAT file
 * via SdFs using a `RingBuf` as the bridge between ISR and main loop.
 *
 * Key behaviours:
 * - Minimal ISR: cache maintenance, safe calls to `rb.write()` within
 *   `rb.beginISR()`/`rb.endISR()`.
 * - Ping-pong DMA buffer in DMAMEM, 32-byte aligned.
 * - Pre-allocated file space to avoid runtime fragmentation.
 *
 * Hardware: Teensy 3.6 / 4.0 / 4.1 (handles __IMXRT1062__ differences).
 *
 * Usage: for Teensy USB Serial, baud is host-side metadata (CDC) and not the
 * physical link speed; monitor is configured at 115200 for tool consistency.
 * captures interactively.
 *
 * Copilot docstrings: function-level Doxygen blocks below assist with
 * code-navigation and automated documentation tools.
 ***************************************************************/
#ifndef DISABLE_FS_H_WARNING
  #define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.
#endif
#include "ADC.h"
#include "DMAChannel.h"
#include "SdFat.h"
#include "FreeStack.h"
#include "RingBuf.h"
#include "TimeLib.h"
#include "version.h"

//******************************************************************************
// global definitions
//******************************************************************************
// Pin must be on first ADC.
#define ADC_PIN A0
// Use FIFO SDIO.
#define SD_CONFIG SdioConfig(FIFO_SDIO)
//------------------------------------------------------------------------------
#if defined(__IMXRT1062__)  // Teensy 4.x
  #define SOURCE_SADDR ADC1_R0
  #define SOURCE_EVENT DMAMUX_SOURCE_ADC1
#else
  #define SOURCE_SADDR ADC0_RA
  #define SOURCE_EVENT DMAMUX_SOURCE_ADC0
#endif

//******************************************************************************
// global variables
//******************************************************************************
// desired block size, multiple of 512
const size_t BUF_BLOCK_SIZE = 512;
// 400 sector RingBuf - could be larger on Teensy 4.1.
const size_t RING_BUF_SIZE = 400 * BUF_BLOCK_SIZE;
const size_t WARMUP_BLOCKS = 1;  // Discard startup transient block(s).
// Count of DMA interrupts.
volatile size_t dmaCount;
// Overrun error for write to RingBuf.
volatile bool overrun;

//******************************************************************************
// create globally available objects
//******************************************************************************
// Preallocate .5GiB file, multiplying 8 by 2^26, resulting in a value of 536,870,912 bytes, or 512 megabytes (MB)
const uint64_t PRE_ALLOCATE_SIZE = 8ULL << 26;
// 100 seconds acquisition time
const uint32_t AQ_TIME = 10 * 1000000;  // 10 seconds acquisition time

ADC adc;
DMAChannel dma(true);

SdFs sd;
FsFile file;

// Single contiguous DMA buffer sized as two blocks (no 2D array).
// `BUF_BLOCK_SIZE` is in bytes; allocate uint16_t elements accordingly.
DMAMEM static uint16_t __attribute__((aligned(32))) dmaBuf[BUF_BLOCK_SIZE / sizeof(uint16_t)];


// RingBuf for BUF_BLOCK_SIZE byte sectors.
RingBuf<FsFile, RING_BUF_SIZE> rb;

// Shared between ISR and background.
volatile size_t maxBytesUsed;

//------------------------------------------------------------------------------
/**
 * @brief Build a log filename using current time if set, else compile time.
 *
 * Uses TimeLib's `timeStatus()` to decide whether to use the runtime clock
 * (set via Serial/RTC) or fall back to `__DATE__`/`__TIME__` from compile time.
 * Format: YYYYMMDD_HHMMSS.bin
 */
static void buildLogFilename(char* out, size_t len) {
  if (timeStatus() == timeSet) {
    int y = year();
    int mo = month();
    int d = day();
    int h = hour();
    int mi = minute();
    int s = second();
    snprintf(out, len, "%04d%02d%02d_%02d%02d%02d.bin", y, mo, d, h, mi, s);
  } else {
    const char* dstr = __DATE__;   // e.g., "Jan 19 2026"
    const char* tstr = __TIME__;   // e.g., "12:34:56"
    char mon_str[4] = { dstr[0], dstr[1], dstr[2], 0 };
    int d = 1, y = 2000;
    (void)sscanf(dstr + 4, "%d %d", &d, &y);
    int h = 0, mi = 0, s = 0;
    (void)sscanf(tstr, "%d:%d:%d", &h, &mi, &s);
    const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char* p = strstr(months, mon_str);
    int mo = p ? ((int)(p - months) / 3 + 1) : 0;
    snprintf(out, len, "%04d%02d%02d_%02d%02d%02d.bin", y, mo, d, h, mi, s);
  }
}

//------------------------------------------------------------------------------
/**
 * @brief DMA interrupt service routine.
 *
 * Called by the DMA channel when a transfer completes (half/full). This ISR
 * performs the minimal required work: invalidate D-cache for the DMA buffer,
 * write a BUF_BLOCK_SIZE-byte sector into the RingBuf in an ISR-safe manner, and update
 * counters. If the RingBuf write fails the `overrun` flag is set.
 *
 * @note Must be kept short and safe for ISR context. Uses `rb.beginISR()` and
 * `rb.endISR()` to enable RingBuf operations inside the ISR.
 */
static void isr() {
  if (!overrun) {
    // Completion-only mode: copy one full valid DMA block each ISR.
    arm_dcache_delete((void*)dmaBuf, BUF_BLOCK_SIZE);
    // Enable RingBuf functions to be called in ISR.
    rb.beginISR();
    if (rb.write(dmaBuf, BUF_BLOCK_SIZE) == BUF_BLOCK_SIZE) {
      dmaCount++;
      if (rb.bytesUsed() > maxBytesUsed) {
        maxBytesUsed = rb.bytesUsed();
      }
    } else {
      overrun = true;
    }
    // End use of RingBuf functions in ISR.
    rb.endISR();
  }
  dma.clearComplete();
  dma.clearInterrupt();
#if defined(__IMXRT1062__)
  // Handle clear interrupt glitch in Teensy 4.x!
  asm("DSB");
#endif  // defined(__IMXRT1062__)
}

//------------------------------------------------------------------------------
/**
 * @brief Initialize DMA and start ADC continuous conversion on `pin`.
 *
 * Configures the DMA channel for ping-pong transfers into `dmaBuf`, attaches
 * the ISR and arms the ADC to start continuous DMA-driven sampling.
 *
 * @param pin ADC input pin (e.g., `ADC_PIN`).
 */
static void init_dma_adc(uint8_t pin) {
  dma.begin();
  dma.attachInterrupt(isr);
  dma.source((volatile const signed short&)SOURCE_SADDR);
  dma.destinationBuffer((volatile uint16_t*)dmaBuf, sizeof(dmaBuf));
  // Use completion-only interrupt for deterministic whole-block copies.
  dma.interruptAtCompletion();
  dma.triggerAtHardwareEvent(SOURCE_EVENT);
  dma.enable();
  adc.adc0->enableDMA();
  adc.adc0->startContinuous(pin);
}

//------------------------------------------------------------------------------
/**
 * @brief Stop ADC DMA and disable the DMA channel.
 *
 * Safe to call from the main thread to halt sampling before file sync/close.
 */
void stopDma() {
  adc.adc0->disableDMA();
  dma.disable();
}

//------------------------------------------------------------------------------
/**
 * @brief Print a small verification readback of the captured data.
 *
 * Reads a portion of the captured data from `file` via the RingBuf and prints
 * values to the provided `Print` instance. Intended for quick verification
 * that samples were captured and written.
 *
 * @param pr Pointer to a `Print`-derived object (for example `&Serial`).
 */
void printTest(Print* pr) {
  if (file.fileSize() < 1024 * 2) {
    return;
  }
  file.rewind();
  rb.begin(&file);
  // Could readIn RING_BUF_SIZE bytes and write to a csv file in a loop.
  if (rb.readIn(2048) != 2048) {
    sd.errorHalt("rb.readIn failed");
  }
  uint16_t data;
  for (size_t i = 0; i < 1024; i += 8) {
    pr->print(i);
    for (size_t j = 0; j < 8; j++) {
      // Test read with: template <typename Type>bool read(Type* data).
      rb.read(&data);
      pr->printf(" %d,", data);
    }
    pr->println();
  }
}

//------------------------------------------------------------------------------
/**
 * @brief Run the main capture test loop.
 *
 * Prepares the file (pre-allocation), begins the ADC+DMA capture, and
 * continuously drains `rb` to disk in BUF_BLOCK_SIZE-byte sectors until stopped by
 * the user or an overrun condition. Prints statistics and runs
 * `printTest()` after capture completes.
 *
 * @param pin ADC input pin to sample (e.g., `ADC_PIN`).
 */
void runTest(uint8_t pin) {
  dmaCount = 0;
  maxBytesUsed = 0;
  overrun = false;
  // wait for Serial to start/connect
  do {
    delay(10);
  } while (Serial.read() >= 0);

  char filename[32];
  buildLogFilename(filename, sizeof(filename));
  Serial.print("Opening file: ");
  Serial.println(filename);
  if (!file.open(filename, O_CREAT | O_TRUNC | O_RDWR)) {
    sd.errorHalt("file.open failed");
  }
  if (!file.preAllocate(PRE_ALLOCATE_SIZE)) {
    sd.errorHalt("file.preAllocate failed");
  }
  rb.begin(&file);
  Serial.print("Running for ");
  Serial.print(AQ_TIME / 1000000);
  Serial.println(" seconds");


  // start the ADC in freerun
  init_dma_adc(pin);

  // Drop initial ADC startup transient so the logged file starts at steady-state.
  uint8_t discardBuf[BUF_BLOCK_SIZE];
  size_t discarded = 0;
  while (!overrun && discarded < WARMUP_BLOCKS) {
    if (rb.bytesUsed() >= BUF_BLOCK_SIZE) {
      if (rb.read(discardBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("warmup discard failed");
        file.close();
        return;
      }
      discarded++;
    }
  }

  uint8_t sectorBuf[BUF_BLOCK_SIZE];
  uint32_t samplingTime = micros();
  //while (!overrun && !Serial.available()) {
  while (!overrun && (micros() - samplingTime) < AQ_TIME) {
    size_t n = rb.bytesUsed();
    if ((n + file.curPosition()) >= (PRE_ALLOCATE_SIZE - BUF_BLOCK_SIZE)) {
      Serial.println("File full - stopping");
      break;
    }
    if (n >= BUF_BLOCK_SIZE) {
      if (rb.read(sectorBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("rb.read() failed");
        file.close();
        return;
      }
      if (file.write(sectorBuf, BUF_BLOCK_SIZE) != BUF_BLOCK_SIZE) {
        Serial.println("file.write() failed");
        file.close();
        return;
      }
      /*
      const uint16_t* samples = reinterpret_cast<const uint16_t*>(sectorBuf);
      const size_t sampleCount = BUF_BLOCK_SIZE / sizeof(uint16_t);
      for (size_t i = 0; i < sampleCount; i = i + 4) {
        Serial.print(micros() - samplingTime);
        Serial.print("\t");
        Serial.print(samples[i]);
        for (size_t j = 0; j < samples[i] / 400; j++) {
          Serial.print("\t");
        }
        Serial.println("*");
      }
        */
    }
  }
  stopDma();
  samplingTime = micros() - samplingTime;
  if (!rb.sync()) {
    Serial.println("sync() failed");
    file.close();
    return;
  }
  if (!file.truncate()) {
    sd.errorHalt("truncate failed");
  }
  if (overrun) {
    Serial.println("Overrun ERROR!!");
  }
  Serial.printf("FreeStack: %d\n", FreeStack());
  Serial.printf("dmaCount %d\n", dmaCount);
  Serial.printf("RingBufSize %d\n", RING_BUF_SIZE);
  Serial.printf("maxBytesUsed %d\n", maxBytesUsed);
  Serial.printf("fileSize %.3fkB\n", .001 * file.fileSize());
  Serial.printf("%.4f seconds\n", 0.000001 * samplingTime);
  Serial.printf("%.6f ksamp/sec\n\n", .5 * 1000 * file.fileSize() / samplingTime);

  //printTest(&Serial);
  file.close();
}

//------------------------------------------------------------------------------
/**
 * @brief Wait for a serial keypress with a prompt message.
 *
 * Blocks until a character is available on `Serial`. Prints `msg` before
 * waiting so the user knows what to do.
 *
 * @param msg Null-terminated prompt string to print.
 */
void waitSerial(const char* msg) {
  do {
    delay(10);
  } while (Serial.read() >= 0);
  Serial.println(msg);
  while (!Serial.available()) {
    // Keep USB/serial background tasks responsive while waiting for input.
    yield();
    delay(1);
  }
  Serial.println();
}

//------------------------------------------------------------------------------
/**
 * @brief Arduino `setup()` entry point.
 *
 * Initializes `Serial`, the SD card, and configures ADC parameters such as
 * averaging, resolution and conversion/sampling speeds used by the test.
 */
 void setup() {
  // On Teensy USB CDC, this value is not the physical wire speed.
  // Keep a conventional value for host tools and compatibility.
  Serial.begin(115200);
  const uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 3000) {
    yield();
  }
  Serial.println("ADC DMA Logger v" VERSION_STRING);
  Serial.println();
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  waitSerial("Enter to begin");

  // set adc speed etc
  adc.adc0->setAveraging(4);  // set number of averages
  adc.adc0->setResolution(12);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
}

//------------------------------------------------------------------------------
/**
 * @brief Arduino `loop()` entry point.
 *
 * Runs `runTest()` and then waits for user input to repeat the test.
 */
void loop() {
  runTest(ADC_PIN);
  waitSerial("Enter to run test again");
}
