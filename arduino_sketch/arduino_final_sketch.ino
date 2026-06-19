// ==========================================================================================
// LAB 9
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

// IMPORTS -----------------------------------------------------------------------------------
#include <ArduinoLowPower.h>
#include <Weights.h>
#include <ECGAnalysis.h>
#include <ADCHelper.h>

// INPUT MODE ---------------------------------------------------------------------------------
// Set to 0 for AD3-style playback for all 998 signals (W1 -> A2/D16, W2 -> D11)
// Set to 1 for the live electrode board: HR_Signal -> A4, LEADS_ON_PROT -> D2.
#ifndef LAB9_USE_ELECTRODES
#define LAB9_USE_ELECTRODES 1
#endif
#ifndef LAB9_ENABLE_BLE
#define LAB9_ENABLE_BLE LAB9_USE_ELECTRODES
#endif

// Lots of definitions for Bluetooth output, primarily to allow for selective deactivation
// for debugging.  Often, Bluetooth is a pretty time-consuming step and it can be easier to
// analyze pure waveforms.
#if LAB9_ENABLE_BLE
#include <BLEOutput.h>
#define statusBegin() ble_output_begin()
#define statusPoll() ble_output_poll()
#define statusConnected() ble_output_connected()
#define statusWriteText(text) ble_output_write_text(text)
#define statusWriteTextf(...) ble_output_write_textf(__VA_ARGS__)
#define statusWriteAfibPrediction(afib_detected, valid) \
  ble_output_write_afib_prediction((afib_detected), (valid))
#define statusWriteHeartRate(heart_rate_bpm) ble_output_write_heart_rate(heart_rate_bpm)
#define statusDelay(delay_ms) ble_output_delay(delay_ms)
#else
#define statusBegin() ((void)0)
#define statusPoll() ((void)0)
#define statusConnected() false
#define statusWriteText(text) ((void)0)
#define statusWriteTextf(...) ((void)0)
#define statusWriteAfibPrediction(afib_detected, valid) ((void)0)
#define statusWriteHeartRate(heart_rate_bpm) ((void)0)
#define statusDelay(delay_ms) ((void)0)
#endif

constexpr uint8_t kUseElectrodes = LAB9_USE_ELECTRODES;
static_assert(kUseElectrodes == 0 || kUseElectrodes == 1, "kUseElectrodes must be 0 or 1");

// DEFINE OUR STATES -------------------------------------------------------------------------
// Keep the same simple state IDs used by the earlier labs - no major changes here.
enum : byte {
  WAITING = 0,
  ACTIVE = 1,
  LOW_POWER = 2,
  POWER_OFF = 3
};

// FSM TIMING --------------------------------------------------------------------------------
// These are the user-visible timing rules from the low-power labs.
// I keep them together because they define the external behavior of the board: how long it
// waits awake, how long it captures, and how it reports progress.
constexpr uint32_t kLowPowerTimeoutMs = 30000;        // 30s
constexpr uint32_t kPowerOffTimeoutMs = 180000;       // 180s
constexpr uint32_t kLowPowerSleepSliceMs = 1000;      // 1s
constexpr uint32_t kCaptureLedPulsePeriodMs = 1200;   // Slow pulse while collecting ECG
constexpr uint32_t kCaptureLedPulseOnMs = 160;        // Length the LED is actually on during a pulse
constexpr uint32_t kAdcBlockTimeoutMs = 250;          // 250 ms
constexpr uint32_t kWindowCaptureTimeoutMs = 12000;   // 12s for one 10s window
constexpr uint16_t kCaptureSettlingDiscardSamples = kUseElectrodes ? 600 : 0;
constexpr uint32_t kEnableReadStartTimeoutMs = 100;   // 100 ms
constexpr uint32_t kEnableReadEndTimeoutMs = 1000;    // 1s
constexpr uint32_t kStatusBleSettleMs = 120;          // 120 ms
constexpr uint32_t kScopeOutputHoldMs = 500;          // Same valid-pulse length as Lab 7
constexpr uint32_t kScopeOutputGapMs = 120;           // Separate class and HR pulses
constexpr uint16_t kHeartRateOutputMaxBpm = 330;      // HR pulse maps BPM = 330 * duration seconds
constexpr uint16_t kHeartRateOutputMaxMs = 1000;      // Max MS to accept HR stuff
constexpr uint8_t kEnableReadLowConfirmSamples = 8;   // reject brief LEAD ON glitches
constexpr uint32_t kEnableReadLowConfirmDelayMs = 2;  // 16 ms total low confirm
constexpr uint8_t kEnableReadHighConfirmSamples = 12; // reject stale/brief LEAD ON starts
constexpr uint32_t kEnableReadHighConfirmDelayMs = 2; // 24 ms total high confirm

// LAB 9 RECORD CLASSIFICATION ---------------------------------------------------------------
// Here, we classify THREE 3000-sample (10s) windows, then use a margin to pick a winning classification
// for all three. Note that there are only two capture slots, that's for SRAM - we save memory this way.
constexpr size_t kLab9CaptureSlotCount = 2;
constexpr uint32_t kRecordCaptureTimeoutMs = kWindowCaptureTimeoutMs * kLab9RecordWindowCount;

// PINS/CONNECTIONS --------------------------------------------------------------------------
// These are wired to the live ECG PCB plus the AD3 scope channels used for latency checks.
const byte ledPin = LED_BUILTIN;
const byte enableReadPin = kUseElectrodes ? 2 : 11;
const uint32_t ecgInputPin = kUseElectrodes ? A4 : A2;
const byte scopeClassPin = 14;  // Same AD3 Scope 2+ output pin as Lab 7
const byte scopeValidPin = 17;  // Same AD3 Scope 1+ valid pin as Lab 7
const byte powerHoldPin = 3;  // PWR_HOLD on the lab PCB
const char* const readyStatusText = kUseElectrodes ? "READY FOR TOUCH" : "READY FOR SIGNAL";
const char* const captureCompleteText =
    kUseElectrodes ? "CAPTURE COMPLETE; REMOVE FINGERS" : "CAPTURE COMPLETE";

// ADC / DMA STATE ---------------------------------------------------------------------------
// The DMA interrupt never runs the classifier.  It only flips half-buffers, averages one ADC
// block, and calls adcSampleReady(). 
Adafruit_ZeroDMA ADC_DMA;
DmacDescriptor* dmac_descriptor_1;
DmacDescriptor* dmac_descriptor_2;

uint16_t adc_buffer[SAMPLE_BLOCK_LENGTH * 2];
volatile bool filling_first_half = true;
volatile uint16_t* active_adc_buffer = nullptr;
volatile bool adc_buffer_filled = false;

// FSM STATE ----------------------------------------------------------------------------------
// This is the high-level state machine: wait, classify one record, sleep, or shut the board down.
static byte g_state = WAITING;
static volatile bool g_enable_read = false;
static volatile bool g_enable_read_asserted_irq = false;
static uint32_t g_waiting_started_ms = 0;
static uint32_t g_idle_elapsed_ms = 0;
static bool g_active_entry_signal_pending = false;
static bool g_waiting_for_enable_read_release = false;

// CAPTURE QUEUE ------------------------------------------------------------------------------
// This is a two "slot"/window system.
// One slot is written by the ISR while the other can be processed by the main loop.
// Slot 0 points into ECGAnalysis's shared classifier window. Slot 1 is the local buffer below.
static int16_t g_capture_window_b[WINDOW_LENGTH] = {0};
static int16_t* g_capture_windows[kLab9CaptureSlotCount] = {nullptr, nullptr};

// I use explicit slot states because the ADC callback and the main loop share this queue.
// The main here loop moves READY -> PROCESSING -> FREE.
enum CaptureSlotState : uint8_t {
  CAPTURE_SLOT_FREE = 0,
  CAPTURE_SLOT_WRITING = 1,
  CAPTURE_SLOT_READY = 2,
  CAPTURE_SLOT_PROCESSING = 3
};

static volatile uint8_t g_capture_slot_state[kLab9CaptureSlotCount] = {
    CAPTURE_SLOT_FREE,
    CAPTURE_SLOT_FREE
};

// A bunch of monitoring stuff for the windows
static volatile uint16_t g_capture_slot_sample_count[kLab9CaptureSlotCount] = {0, 0};
static volatile uint8_t g_capture_slot_window_index[kLab9CaptureSlotCount] = {0, 0};
static volatile uint8_t g_capture_write_slot = 0;
static volatile uint16_t g_capture_write_index = 0;
static volatile uint8_t g_capture_completed_window_count = 0;
static volatile bool g_capture_stream_active = false;
static volatile bool g_capture_stream_done = false;
static volatile bool g_capture_stream_overrun = false;
static volatile uint16_t g_capture_discard_remaining_samples = 0;

static volatile uint8_t g_enable_read_low_sample_count = 0;

// LATENCY TRACKING ---------------------------------------------------------------------------
// This lets ACTIVE avoid waiting twice for the same falling edge at the end of a record.
static volatile bool g_signal_end_observed = false;

// This struct is the record-level summary built from up to three classifier windows.
// I keep per-window predictions out of SRAM and retain only the pieces needed for final HR,
// diagnostics, and the mean-plus-max margin AFIB rule.
// The accumulator is intentionally stack/local inside processCapturedSignal().
struct Lab9RecordAccumulator {
  size_t total_samples_captured;
  size_t classified_window_count;
  float margin_sum;
  float max_margin;
  uint16_t valid_heart_rates[kLab9RecordWindowCount];
  uint8_t valid_heart_rate_count;
  Lab9CaptureDiagnosticsAccumulator diagnostics;
};

// HELPERS -----------------------------------------------------------------------------------
// Further defined elsewhere, but the basic idea is that a lot of the heavier lifting is in /ENGG499/
static void enableReadISR();
static void reportSystemStatus(const char* text, bool allow_ble_settle = false);
static void clearScopePredictionOutput();
static void writeScopeCaptureProgress(uint8_t captured_window_count);
static void writeScopePredictionOutput(
    bool afib_detected,
    bool classification_valid,
    uint16_t heart_rate_bpm,
    bool heart_rate_valid
);
static void resetWaitingTimer();
static bool enableReadHighConfirmed();
static bool enableReadLowConfirmed();
static bool enterActiveIfEnableReadConfirmed(bool woke_from_low_power);
static void adcSampleReady(uint16_t sample);
static void resetCaptureQueue();
static bool processReadyCaptureWindow(Lab9RecordAccumulator& record);
static void processCapturedSignal(bool signal_already_high);

// SETUP -------------------------------------------------------------------------------------
// Bring up capture, power-hold, scope outputs, and the optional status channel.
void setup() {

  // Slot 0 (the first one) is ECGAnalysis's classifier window; slot 1 is a rolling buffer.
  g_capture_windows[0] = ecg_analysis_classifier_window();
  g_capture_windows[1] = g_capture_window_b;
  resetCaptureQueue();

  adc_set_input_pin(ecgInputPin);
  adc_set_sample_callback(adcSampleReady);

  // Start the ADC capture engine early, so we can wait and be ready for signal
  adc_init();
  dma_init();
  ADC_DMA.startJob();

  // PCB control pins and the scope outputs in idle state, ready to go
  pinMode(powerHoldPin, OUTPUT);
  digitalWrite(powerHoldPin, HIGH);

  // In electrode mode this is LEADS_ON_PROT/D2; in AD3 mode it is W2/D11.
  pinMode(enableReadPin, INPUT);
  pinMode(scopeValidPin, OUTPUT);
  analogWriteResolution(10);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  clearScopePredictionOutput();

  // Set up the wakeup interrupt once and leave it there
  detachInterrupt(digitalPinToInterrupt(enableReadPin));
  LowPower.attachInterruptWakeup(enableReadPin, enableReadISR, CHANGE);
  g_enable_read = (digitalRead(enableReadPin) == HIGH);

  // This starts BLE (or no-ops if we're jettisoning BLE during testing)
  statusBegin();

  resetWaitingTimer();
}

// LOOP --------------------------------------------------------------------------------------
// The loop only runs the FSM. All ECG sample movement happens through the ADC callback.
void loop() {

  // Optional status housekeeping has to happen even while we're mostly waiting around.
  statusPoll();

  // Always sample LEAD ON before making the next decision. 
  // The edge can land between interrupts and the top of the loop.
  if (g_state != POWER_OFF) {
    digitalWrite(powerHoldPin, HIGH);
  }
  g_enable_read = (digitalRead(enableReadPin) == HIGH);
  const byte state_at_loop_start = g_state;

  switch (g_state) {

    // FSM STATE: WAITING ---------------------------------------------------------------------
    // Stay fully awake briefly so the board feels responsive right after a completed run.
    case WAITING:
      {
      const uint32_t waiting_elapsed_ms = g_idle_elapsed_ms + (millis() - g_waiting_started_ms);
      digitalWrite(ledPin, LOW);

      // After a capture, I require the gate to return low before accepting a new one. This
      // prevents one long touch or one long AD3 enable pulse from creating back-to-back records.
      // (That was a common and annoying issue in early testing)
      if (g_waiting_for_enable_read_release) {
        if (enableReadLowConfirmed()) {
          g_waiting_for_enable_read_release = false;
          resetWaitingTimer();
          reportSystemStatus(readyStatusText, true);
        } else {
          break;
        }
      }

      if (enterActiveIfEnableReadConfirmed(false)) {
        break;
      }

      // WAIT is still fully awake, burning power, we expect more touches/input
      if (waiting_elapsed_ms >= kLowPowerTimeoutMs) {
        g_idle_elapsed_ms = waiting_elapsed_ms;
        g_state = LOW_POWER;
        digitalWrite(ledPin, LOW);
        reportSystemStatus("FSM WAIT->LOWPWR", true);
        reportSystemStatus("LOW POWER ENTER", true);
      }
      break;
      }

    // FSM STATE: ACTIVE ----------------------------------------------------------------------
    // One valid LEAD ON pulse means one live ECG record: capture, classify, publish -> go back to idle
    case ACTIVE:
      digitalWrite(ledPin, LOW);
      processCapturedSignal(g_active_entry_signal_pending);

      // ACTIVE handles exactly 1 ECG record. After that, WAIT owns the release check and the
      // next possible record start.
      g_active_entry_signal_pending = false;
      g_waiting_for_enable_read_release = true;

      digitalWrite(ledPin, LOW);
      resetWaitingTimer();
      g_state = WAITING;
      break;

    // FSM STATE: LOW_POWER -------------------------------------------------------------------
    // Sleep in short slices so LEAD ON can wake the SAMD21 and idle time can reach POWER_OFF
    case LOW_POWER:
      digitalWrite(powerHoldPin, HIGH);
      digitalWrite(ledPin, LOW);

      // Kinda redundant here in that I check for wake-up twice, we want the board kinda snappy
      // in waking up
      if (enterActiveIfEnableReadConfirmed(true)) {
        break;
      }

      // Kill power if we've waited around for long enough
      if (g_idle_elapsed_ms >= kPowerOffTimeoutMs) {
        g_state = POWER_OFF;
        break;
      }

      // Duplicative, but snappy
      g_enable_read = (digitalRead(enableReadPin) == HIGH);
      if (enterActiveIfEnableReadConfirmed(true)) {
        break;
      }

      // Counting stuff
      LowPower.sleep(kLowPowerSleepSliceMs);

      // Duplicative (x3), but snappy
      g_enable_read = (digitalRead(enableReadPin) == HIGH);
      if (enterActiveIfEnableReadConfirmed(true)) {
        break;
      }

      // More counting time
      g_idle_elapsed_ms += kLowPowerSleepSliceMs;
      if (g_idle_elapsed_ms >= kPowerOffTimeoutMs) {
        g_state = POWER_OFF;
      }
      break;

    // FSM STATE: POWER_OFF -------------------------------------------------------------------
    // Release PWR_HOLD and let the battery-powered board shut down.
    case POWER_OFF:
      reportSystemStatus("FSM LOWPWR->OFF", true);
      reportSystemStatus("POWERING OFF", true);
      digitalWrite(ledPin, LOW);
      clearScopePredictionOutput();

      // The external PCB power switch is latched by PWR_HOLD. Dropping it hands control back
      // to the hardware power path.
      digitalWrite(powerHoldPin, LOW);

      while (true) {
        delay(1000);
      }
      break;
  }

  // Keep awake states from spinning pointlessly. LOW_POWER sleeps explicitly, and WAIT->ACTIVE
  // should start capture without this extra delay.
  if (state_at_loop_start != LOW_POWER && !(state_at_loop_start == WAITING && g_state == ACTIVE)) {
    delay(20);
  }
}

// INTERRUPTS --------------------------------------------------------------------------------
// Keep the ISR tiny: snapshot the LEAD ON level and remember if a high edge showed up.
static void enableReadISR() {
  // The ISR does not start capture directly. It only snapshots the gate and lets loop() run the
  // confirmation checks in normal code.
  g_enable_read = (digitalRead(enableReadPin) == HIGH);
  if (g_enable_read) {
    g_enable_read_asserted_irq = true;
    g_enable_read_low_sample_count = 0;
  } else {
    g_enable_read_asserted_irq = false;
  }
}

// If Bluetooth LE is desired, this writes to the BLE log characteristic
// and optionally gives notifications time to leave before the next message.
static void reportSystemStatus(const char* text, bool allow_ble_settle) {
  statusWriteText(text);
  if (allow_ble_settle && statusConnected()) {
    statusDelay(kStatusBleSettleMs);
  }
}

// Clears the pins just in case
static void clearScopePredictionOutput() {
  digitalWrite(scopeValidPin, LOW);
  analogWrite(scopeClassPin, 0);
}

// During capture I use D14 as a simple analog progress level. The valid pin stays low so the
// AD3 scripts do not treat progress as a final prediction pulse.
// I suspect I could jettison this because it's a function of debugging, but I'm keeping it here for sanity.
static void writeScopeCaptureProgress(uint8_t captured_window_count) {
  digitalWrite(scopeValidPin, LOW);
  if (captured_window_count >= kLab9RecordWindowCount) {
    analogWrite(scopeClassPin, 768);
  } else if (captured_window_count == 2) {
    analogWrite(scopeClassPin, 512);
  } else if (captured_window_count == 1) {
    analogWrite(scopeClassPin, 256);
  } else {
    analogWrite(scopeClassPin, 128);
  }
}

// This does TWO things when I'm running AD3 tests:
// 1. Give a clean valid edge for classifier latency/accuracy
// 2. Encodes HR without needing serial or BLE -> less drama
// This is done in two pulses
// The FIRST pulse is a fixed-width valid prediction pulse with analog level 0 or 1023 for classification
// The SECOND pulse has a width proportional to BPM
static void writeScopePredictionOutput(
    bool afib_detected,
    bool classification_valid,
    uint16_t heart_rate_bpm,
    bool heart_rate_valid
) {
  if (!classification_valid) {

    // If the record was incomplete, I keep the scope output quiet instead of presenting a
    // misleading class or HR pulse -> avoid bad outputs.
    clearScopePredictionOutput();
    return;
  }

  uint16_t heart_rate_pulse_ms = 0;
  if (heart_rate_valid) {
    if (heart_rate_bpm > kHeartRateOutputMaxBpm) {
      heart_rate_bpm = kHeartRateOutputMaxBpm;
    }
    heart_rate_pulse_ms = static_cast<uint16_t>(
        (static_cast<uint32_t>(heart_rate_bpm) * kHeartRateOutputMaxMs +
         (kHeartRateOutputMaxBpm / 2u)) /
        kHeartRateOutputMaxBpm
    );
  }

  // FIRST PULSE ------------------------------------------------------
  // D14 carries class, matching the Lab 7 scope convention
  analogWrite(scopeClassPin, afib_detected ? 1023 : 0);
  digitalWrite(scopeValidPin, HIGH);
  delay(kScopeOutputHoldMs);
  digitalWrite(scopeValidPin, LOW);
  analogWrite(scopeClassPin, 0);
  delay(kScopeOutputGapMs);

  // SECOND PULSE ------------------------------------------------------
  // Width encodes HR, BPM = 330 * s
  if (heart_rate_pulse_ms > 0) {
    analogWrite(scopeClassPin, 1023);
    digitalWrite(scopeValidPin, HIGH);
    delay(heart_rate_pulse_ms);
    digitalWrite(scopeValidPin, LOW);
    analogWrite(scopeClassPin, 0);
  }
}

// FSM HELPERS -------------------------------------------------------------------------------
// Reset the idle timer whenever ACTIVE finishes and hands control back to WAIT.
static void resetWaitingTimer() {
  g_waiting_started_ms = millis();
  g_idle_elapsed_ms = 0;
}

// LEAD ON can glitch around transitions, which is annoying as hell during tests
// To kinda-sorta fix this, a start is treated only as real after repeatedly being high
static bool enableReadHighConfirmed() {

  // Basically, this rejects variability/one-sample enable glitches.  We demand some degree of consistency.
  for (uint8_t sample_index = 0; sample_index < kEnableReadHighConfirmSamples; ++sample_index) {
    if (digitalRead(enableReadPin) != HIGH) {
      g_enable_read = false;
      return false;
    }
    delay(kEnableReadHighConfirmDelayMs);
  }
  g_enable_read = true;
  return true;
}

// I use a separate release debounce because removing fingers gets weird and bounce-y.
// AD3 release pulses can also have brief edge artifacts, not 100% sure why (pure interference?)
static bool enableReadLowConfirmed() {

  // Same as above, we just demand consistency
  for (uint8_t sample_index = 0; sample_index < kEnableReadLowConfirmSamples; ++sample_index) {
    if (digitalRead(enableReadPin) != LOW) {
      g_enable_read = true;
      return false;
    }
    delay(kEnableReadLowConfirmDelayMs);
  }

  g_enable_read = false;
  g_enable_read_asserted_irq = false;
  g_enable_read_low_sample_count = kEnableReadLowConfirmSamples;
  return true;
}

// This function is the only place WAIT/LOW_POWER can promote into ACTIVE. Keeping the state
// transition here makes it clear that capture only starts after the gate is confirmed high.
static bool enterActiveIfEnableReadConfirmed(bool woke_from_low_power) {

  if (g_waiting_for_enable_read_release) {
    return false;
  }

  if (!(g_enable_read || g_enable_read_asserted_irq)) {
    return false;
  }

  if (!enableReadHighConfirmed()) {
    g_enable_read_asserted_irq = false;
    return false;
  }

  g_enable_read_asserted_irq = false;
  g_active_entry_signal_pending = true;
  g_state = ACTIVE;
  reportSystemStatus(woke_from_low_power ? "FSM LOWPWR->ACTIVE" : "FSM WAIT->ACTIVE", true);
  reportSystemStatus("SIGNAL RECEIVED", true);
  return true;
}

// ADC + CAPTURE QUEUE -----------------------------------------------------------------------
// Put the queue back into a known empty state before a new LEAD ON pulse starts capture.
static void resetCaptureQueue() {

  // I reset ALL fields with interrupts off so the ADC callback cannot observe a half-reset queue.
  noInterrupts();
  for (size_t i = 0; i < kLab9CaptureSlotCount; ++i) {
    g_capture_slot_state[i] = CAPTURE_SLOT_FREE;
    g_capture_slot_sample_count[i] = 0;
    g_capture_slot_window_index[i] = 0;
  }
  g_capture_write_slot = 0;
  g_capture_write_index = 0;
  g_capture_completed_window_count = 0;
  g_capture_stream_active = false;
  g_capture_stream_done = false;
  g_capture_stream_overrun = false;
  g_capture_discard_remaining_samples = 0;

  g_enable_read_low_sample_count = 0;
  interrupts();
}

// This is the one shared ISR-side way to say "the record is no longer accepting samples."
// The main loop polls g_capture_stream_done and then drains any READY windows.
static void stopCaptureStreamFromInterrupt() {
  g_capture_stream_active = false;
  g_capture_stream_done = true;
}

// The callback keeps capture moving without blocking. 
// Computationally intensive stuff like ECGAnalysis stays in loop() after 
// we get a window tagged READY
static void adcSampleReady(uint16_t sample) {

  // Ignore samples unless a record is active.
  if (!g_capture_stream_active) {
    return;
  }

  // Check for a confirmed falling gate so the record can end cleanly.
  if (digitalRead(enableReadPin) != HIGH) {
    // A low gate means the person removed fingers or AD3 dropped W2. I require a few consecutive
    // lows so a short glitch does not prematurely close the final window.
    if (g_enable_read_low_sample_count < kEnableReadLowConfirmSamples) {
      g_enable_read_low_sample_count++;
    }
    if (g_enable_read_low_sample_count >= kEnableReadLowConfirmSamples) {
      g_signal_end_observed = true;

      // Close out a partially filled final window if we captured enough samples to pad it.
      const uint8_t slot = g_capture_write_slot;
      const bool final_window_can_pad =
          g_capture_completed_window_count == (kLab9RecordWindowCount - 1) &&
          g_capture_write_index >= kLab9MinPaddedWindowSamples;

      // If this is the third window and it is long enough, I keep it and let ECGAnalysis pad it.
      // Earlier short windows are thrown away because they would not represent the intended
      // 10-second classifier input.
      if (g_capture_slot_state[slot] == CAPTURE_SLOT_WRITING && final_window_can_pad) {
        g_capture_slot_sample_count[slot] = g_capture_write_index;
        g_capture_slot_window_index[slot] = g_capture_completed_window_count;
        g_capture_slot_state[slot] = CAPTURE_SLOT_READY;
        g_capture_completed_window_count++;
      } else if (g_capture_slot_state[slot] == CAPTURE_SLOT_WRITING) {
        g_capture_slot_state[slot] = CAPTURE_SLOT_FREE;
      }

      g_capture_write_index = 0;
      stopCaptureStreamFromInterrupt();
    }
    return;
  }
  g_enable_read_low_sample_count = 0;

  // Right after touch, the analog front-end can still be settling. I discard those electrode
  // samples before filling the record window. AD3 mode sets this count to zero.
  if (g_capture_discard_remaining_samples > 0) {
    g_capture_discard_remaining_samples--;
    return;
  }

  // Append one sample to the current slot.
  int16_t* window = g_capture_windows[g_capture_write_slot];
  if (g_capture_slot_state[g_capture_write_slot] != CAPTURE_SLOT_WRITING) {
    g_capture_stream_overrun = true;
    stopCaptureStreamFromInterrupt();
    return;
  }
  window[g_capture_write_index] = static_cast<int16_t>(sample);
  g_capture_write_index++;


  // Close out a full 3000-sample window and arm the next slot if the record still needs it.
  // The main loop will see this slot as READY and classify it while the ISR starts filling the
  // next FREE slot.
  if (g_capture_write_index >= WINDOW_LENGTH) {
    const uint8_t slot = g_capture_write_slot;
    g_capture_slot_sample_count[slot] = WINDOW_LENGTH;
    g_capture_slot_window_index[slot] = g_capture_completed_window_count;
    g_capture_slot_state[slot] = CAPTURE_SLOT_READY;
    g_capture_completed_window_count++;
    g_capture_write_index = 0;

    if (g_capture_completed_window_count >= kLab9RecordWindowCount) {
      stopCaptureStreamFromInterrupt();
      return;
    }

    int8_t next_slot = -1;
    for (uint8_t candidate_slot = 0; candidate_slot < kLab9CaptureSlotCount; ++candidate_slot) {
      if (g_capture_slot_state[candidate_slot] == CAPTURE_SLOT_FREE) {
        next_slot = static_cast<int8_t>(candidate_slot);
        break;
      }
    }

    // If loop() cannot free a slot quickly enough, I stop capture instead of overwriting data.
    if (next_slot < 0) {
      g_capture_stream_overrun = true;
      stopCaptureStreamFromInterrupt();
      return;
    }

    g_capture_write_slot = static_cast<uint8_t>(next_slot);
    g_capture_slot_state[g_capture_write_slot] = CAPTURE_SLOT_WRITING;
  }
}

// Function to cause us to actually process a given window sample using the ECG
// Note that this is ONE window, not all three
static bool processReadyCaptureWindow(Lab9RecordAccumulator& record) {

  uint8_t capture_slot = 0;
  uint8_t window_index = 0;
  uint16_t samples_captured = 0;
  bool found = false;

  // Pull the oldest ready window out of the queue
  noInterrupts();
  for (uint8_t slot = 0; slot < kLab9CaptureSlotCount; ++slot) {
    if (g_capture_slot_state[slot] != CAPTURE_SLOT_READY) {
      continue;
    }

    if (!found || g_capture_slot_window_index[slot] < window_index) {
      found = true;
      capture_slot = slot;
      window_index = g_capture_slot_window_index[slot];
      samples_captured = g_capture_slot_sample_count[slot];
    }
  }
  
  // Mark PROCESSING if found while interrupts are still off so the ISR will not reuse this slot.
  if (found) {
    g_capture_slot_state[capture_slot] = CAPTURE_SLOT_PROCESSING;
  }
  interrupts();
  if (!found) {
    return false;
  }

  // Define the window
  const bool full_window = samples_captured == WINDOW_LENGTH;
  const bool padded_final_window =
      window_index == (kLab9RecordWindowCount - 1) &&
      samples_captured >= kLab9MinPaddedWindowSamples;
  const bool usable_window =
      window_index < kLab9RecordWindowCount &&
      (full_window || padded_final_window);

  // A short non-final window is not useful for the classifier, so throw a fit
  // if we don't have a usable window
  if (!usable_window) {
    noInterrupts();
    g_capture_slot_state[capture_slot] = CAPTURE_SLOT_FREE;
    interrupts();
    return true;
  }

  int16_t* window = g_capture_windows[capture_slot];
  record.total_samples_captured += samples_captured;

  // Pump out diagnostics
  ecg_analysis_update_capture_diagnostics(record.diagnostics, window, samples_captured);

  // Estimate HR per classifier window (these get combined later)
  const Lab9HeartRateResult window_heart_rate = ecg_analysis_estimate_heart_rate(
      window,
      samples_captured
  );
  if (window_heart_rate.valid && record.valid_heart_rate_count < kLab9RecordWindowCount) {
    record.valid_heart_rates[record.valid_heart_rate_count] = window_heart_rate.heart_rate_bpm;
    record.valid_heart_rate_count++;
  }

  // Classification mutates the window during preprocessing, so all diagnostics and HR work
  // that need raw-ish samples happen BEFORE THIS CALL - not after(!)
  const ECGClassifierModel classifier_model = ecg_classifier_model_from_weights();
  const ECGAnalysisResult window_result = ecg_analysis_classify_mutable_window(
      window,
      samples_captured,
      &classifier_model
  );

  // I accumulate just the sum and maximum margin here so no per-window scores have to be retained, 
  // that's a huge burden on the SRAM
  if (window_result.classification_valid) {
    const float margin = window_result.afib_score - window_result.normal_score;
    record.margin_sum += margin;
    if (record.classified_window_count == 0 || margin > record.max_margin) {
      record.max_margin = margin;
    }
    record.classified_window_count++;
  }

  noInterrupts();
  g_capture_slot_state[capture_slot] = CAPTURE_SLOT_FREE;
  interrupts();
  return true;
}

// ECG PROCESSING ----------------------------------------------------------------------
// Capture one live ECG record, run ECGAnalysis, and publish BLE/scope results.
// ACTIVE enters here once per record. Everything in this function is record-scoped: start
// capture, drain/classify windows, publish one final result, then return to WAIT.
// This is a painfully long function because it basically handles the whole data path, tip to tail
static void processCapturedSignal(bool signal_already_high) {

  g_signal_end_observed = false;
  clearScopePredictionOutput();

  // If the FSM already saw LEAD ON high, do not re-block forever waiting for the same pulse.
  if (!signal_already_high) {

    // Normally WAIT already confirmed the high gate. But here it double-checks for sanity
    const uint32_t signal_wait_started_ms = millis();
    while (true) {
      if (digitalRead(enableReadPin) == HIGH && enableReadHighConfirmed()) {
        break;
      }

      if (millis() - signal_wait_started_ms >= kEnableReadStartTimeoutMs) {
        g_enable_read = false;
        return;
      }
      delay(1);
    }
  }

  // Capture one 30-second Lab 9 record. The ISR fills one slot while the main loop
  // classifies another, so only two full ECG windows have to stay in SRAM -> savings!
  Lab9RecordAccumulator record = {};
  record.diagnostics = ecg_analysis_make_capture_diagnostics_accumulator();
  Lab9HeartRateResult heart_rate_result = {0, false};

  // Restart ADC/DMA and drop the first block so the averaged samples line up
  resetCaptureQueue();

  // Restart the engine for every record so the first captured window has a predictable cadence
  // Low-power sleep can leave ADC/DMA in a stale state, which gets wonky at this part
  ADC_DMA.abort();
  adc_buffer_filled = false;
  filling_first_half = true;
  active_adc_buffer = &adc_buffer[0];
  adc_init();
  ADC_DMA.startJob();

  const uint32_t adc_wait_started_ms = millis();
  while (!adc_buffer_filled && millis() - adc_wait_started_ms < kAdcBlockTimeoutMs) {
  }

  // If filled, we kick off here
  // The first post-restart DMA block is discarded. 
  // The next decimated sample starts the record.
  if (adc_buffer_filled) {
    
    adc_buffer_filled = false;
    noInterrupts();
    g_capture_slot_state[0] = CAPTURE_SLOT_WRITING;
    g_capture_write_slot = 0;
    g_capture_write_index = 0;
    g_capture_discard_remaining_samples = kCaptureSettlingDiscardSamples;
    g_capture_stream_active = true;
    g_capture_stream_done = false;
    g_capture_stream_overrun = false;
    interrupts();

    const uint32_t capture_started_ms = millis();
    uint8_t reported_window_count = 0;
    bool full_record_captured = false;
    writeScopeCaptureProgress(reported_window_count);

    // Keep classifying ready windows while the ISR fills the next one. 
    // The LED can get a bit weird here, which is more or less OK, but future improvements
    // could probably do something else so it's not quite as jittery.  Maybe just leaving the light on
    // and not pulsing it?
    while (true) {

      // Handle LED
      while (true) {
        digitalWrite(ledPin, LOW);
        if (!processReadyCaptureWindow(record)) {
          break;
        }
        statusPoll();
      }

      // Snapshot the volatile capture flags together so this loop makes one consistent decision
      noInterrupts();
      const bool capture_done = g_capture_stream_done;
      const bool queue_overrun = g_capture_stream_overrun;
      const uint8_t completed_window_snapshot = g_capture_completed_window_count;
      interrupts();

      // Valid stays low until the final class/HR output.
      if (completed_window_snapshot != reported_window_count) {
        reported_window_count = completed_window_snapshot;
        writeScopeCaptureProgress(reported_window_count);
      }

      // Throw a fit if our queue overruns (this doesn't seem to happen in testing anymore,
      // but I keep this here for sanity)
      if (queue_overrun) {
        reportSystemStatus("CAPTURE OVERRUN", true);
        break;
      }

      // Done can mean three full windows, a padded final window after release, timeout, or
      // overrun. Here, we don't ascertain validity, just doneness.
      if (capture_done) {
        full_record_captured = completed_window_snapshot >= kLab9RecordWindowCount;
        break;
      }

      // I never want a stuck gate to capture beyond one record. Thirty seconds is the target;
      // this timeout gives each 10-second window a little margin.
      if (millis() - capture_started_ms >= kRecordCaptureTimeoutMs) {
        noInterrupts();
        g_capture_stream_active = false;
        g_capture_stream_done = true;
        interrupts();
        break;
      }

      const uint32_t capture_phase_ms =
          (millis() - capture_started_ms) % kCaptureLedPulsePeriodMs;

      // The LED slowly pulses only while samples are actively being collected.
      // That said, as noted above, it gets a bit screwy when the processor is true and fully busy
      // Might change it at some point
      digitalWrite(ledPin, capture_phase_ms < kCaptureLedPulseOnMs ? HIGH : LOW);
      statusPoll();
      delay(1);
    }
    digitalWrite(ledPin, LOW);

    // Capture may have ended with one last window in READY state, so drain it just in case
    while (true) {
      digitalWrite(ledPin, LOW);
      if (!processReadyCaptureWindow(record)) {
        break;
      }
      statusPoll();
    }

    // Report if a full record is captured
    if (full_record_captured) {
      reportSystemStatus(captureCompleteText, true);
    }

    // Sanity check diagnostics, this burns power/time/latency but is helpful especially with the electrodes
    const Lab9CaptureDiagnostics capture_diagnostics =
        ecg_analysis_finish_capture_diagnostics(record.diagnostics);
    if (record.total_samples_captured > 0) {
      statusWriteTextf(
          "CAPTURE Q p2p=%u min=%u z=%u.%u",
          capture_diagnostics.peak_to_peak_counts,
          capture_diagnostics.min_window_peak_to_peak_counts,
          capture_diagnostics.zero_cross_rate_hz_x10 / 10u,
          capture_diagnostics.zero_cross_rate_hz_x10 % 10u
      );
      statusDelay(kStatusBleSettleMs);
    }

    // Publish the HR result
    heart_rate_result = ecg_analysis_summarize_heart_rates(
        record.valid_heart_rates,
        record.valid_heart_rate_count
    );
  }

  // If the ISR did not observe the falling edge, wait a bit so the caller does not
  // return to WAIT while the gate is still high.
  if (!g_signal_end_observed) {
    const uint32_t signal_end_started_ms = millis();
    bool low_confirmed = false;
    g_enable_read = (digitalRead(enableReadPin) == HIGH);
    while (true) {
      low_confirmed = enableReadLowConfirmed();
      if (low_confirmed || millis() - signal_end_started_ms >= kEnableReadEndTimeoutMs) {
        break;
      }
    }
    if (low_confirmed) {
      g_signal_end_observed = true;
    }
  }

  // If capture failed, throw a fit
  // (This often happens if you just tap the electrodes randomly, as you might expect)
  if (record.total_samples_captured == 0) {
    statusWriteAfibPrediction(false, false);
    statusWriteHeartRate(static_cast<uint16_t>(0));
    statusWriteText("RESULT HR=0 BPM AFIB=UNKNOWN");
    writeScopePredictionOutput(false, false, 0, false);
    return;
  }

  // Validity definitions based on record length, classification, afib
  const bool record_length_valid =
      record.total_samples_captured >= kLab9MinRecordSamples &&
      record.total_samples_captured <= kLab9RequiredRecordSamples;
  const bool classification_valid =
      record.classified_window_count == kLab9RecordWindowCount &&
      record_length_valid;
  const bool afib_detected = classification_valid
      ? ecg_analysis_predict_lab9_record(record.margin_sum, record.max_margin)
      : false;

  // BLE receives HR/AFIB/text, we can also push via the scope pins (in AD3 testing especially)
  statusWriteAfibPrediction(
      afib_detected,
      classification_valid
  );
  statusWriteHeartRate(
      heart_rate_result.valid ? heart_rate_result.heart_rate_bpm : static_cast<uint16_t>(0)
  );
  statusWriteTextf(
      "RESULT HR=%u BPM AFIB=%s",
      heart_rate_result.valid ? heart_rate_result.heart_rate_bpm : static_cast<uint16_t>(0),
      classification_valid ? (afib_detected ? "AFIB" : "NOT AFIB") : "UNKNOWN"
  );
  writeScopePredictionOutput(
      afib_detected,
      classification_valid,
      heart_rate_result.heart_rate_bpm,
      heart_rate_result.valid
  );
}
