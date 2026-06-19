// ==========================================================================================
// ECG ANALYSIS
// AFIB classification + Heart Rate Processing Functions
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#ifndef ECGANALYSIS_H
#define ECGANALYSIS_H

// IMPORTS -----------------------------------------------------------------------------
#include "Arduino.h"
#include "SignalCondition.h"

// ECG ANALYSIS CONSTANTS --------------------------------------------------------------
// These lengths come from the SignalCondition pipeline for one 3000-sample ECG window.
constexpr size_t ECG_ANALYSIS_FEATURE1_LENGTH = kFeature1Length;
constexpr size_t ECG_ANALYSIS_FEATURE2_LENGTH = kFeature2Length;

// ECG CLASSIFIER MODEL ----------------------------------------------------------------
// Pointer table for the trained GRU model. Weights.h owns the actual generated arrays for 8x8x8
struct ECGClassifierModel {

  // Model Topology
  int hidden_size1;
  int hidden_size2;
  int hidden_size3;
  int input_size1;
  int input_size2;
  int input_size3;
  int input_size_fc;
  int output_size_fc;

  // Pointers to Weights.h stuff, saves memory
  const float* Wfc;
  const float* Wh1;
  const float* Wh2;
  const float* Wh3;
  const float* Wr1;
  const float* Wr2;
  const float* Wr3;
  const float* Wz1;
  const float* Wz2;
  const float* Wz3;
  const float* bfc;
  const float* bh1;
  const float* bh2;
  const float* bh3;
  const float* br1;
  const float* br2;
  const float* br3;
  const float* bz1;
  const float* bz2;
  const float* bz3;
  const float* mu;
  const float* sg;
};

// ECG ANALYSIS RESULT -----------------------------------------------------------------
// Keep only what Lab 9's record-level voting needs from each 10-second window.
struct ECGAnalysisResult {

  // classification_valid separates "the model produced a score" from "the caller should trust
  // this record"
  bool classification_valid;
  
  // Raw class scores for margin approach
  float afib_score;
  float normal_score;
};

// LAB 9 RECORD HELPERS ----------------------------------------------------------------

// I capture three 10-second windows and combines their per-window classifier scores
constexpr size_t kLab9RecordWindowCount = 3;
constexpr size_t kLab9RequiredRecordSamples = kLab9RecordWindowCount * WINDOW_LENGTH;

// If the final window ends early but has at least half of a normal window, we pad it.
constexpr uint16_t kLab9MinPaddedWindowSamples = 1500;
constexpr size_t kLab9MinRecordSamples =
    ((kLab9RecordWindowCount - 1) * WINDOW_LENGTH) + kLab9MinPaddedWindowSamples;

// The result struct lets the sketch publish HR=0 when a noisy or short window cannot support an estimate.
// That is different from an actual zero BPM estimate.
struct Lab9HeartRateResult {
  uint16_t heart_rate_bpm;
  bool valid;
};

// Log-only diagnostics, helpful mostly for issues with the electrodes
struct Lab9CaptureDiagnostics {
  uint16_t peak_to_peak_counts;
  uint16_t min_window_peak_to_peak_counts;
  uint16_t zero_cross_rate_hz_x10;
};

// Updated once per processed window so it never stores the whole 30-second
// record in SRAM.
struct Lab9CaptureDiagnosticsAccumulator {
  bool have_samples;
  uint16_t global_min_counts;
  uint16_t global_max_counts;
  uint16_t min_window_peak_to_peak_counts;
  uint32_t sample_count_total;
  uint32_t zero_crossing_count_total;
};

bool ecg_analysis_predict_lab9_record(float margin_sum, float max_margin);

Lab9CaptureDiagnosticsAccumulator ecg_analysis_make_capture_diagnostics_accumulator();

// Updates capture diagnostics
void ecg_analysis_update_capture_diagnostics(
    Lab9CaptureDiagnosticsAccumulator& accumulator,
    const int16_t* window,
    uint16_t sample_count
);

// Finishes same capture diagnostics
Lab9CaptureDiagnostics ecg_analysis_finish_capture_diagnostics(
    const Lab9CaptureDiagnosticsAccumulator& accumulator
);

// HR result stuff
Lab9HeartRateResult ecg_analysis_estimate_heart_rate(
    const int16_t* samples,
    uint16_t sample_count
);
Lab9HeartRateResult ecg_analysis_summarize_heart_rates(
    uint16_t valid_rates[kLab9RecordWindowCount],
    uint8_t valid_rate_count
);

// RAM-SAVING CLASSIFIER WINDOW API ---------------------------------------------------
// Exposes ECGAnalysis's shared 3000-sample classifier window, that way the .ino can
// "reach in" and access it without copying into ram
int16_t* ecg_analysis_classifier_window();

// Process a mutable caller-owned window
ECGAnalysisResult ecg_analysis_classify_mutable_window(
    int16_t* signal_window,
    size_t sample_count,
    const ECGClassifierModel* classifier_model
);

#ifdef WEIGHTS_H
// Build a model pointer table from the generated globals in Weights.h
inline ECGClassifierModel ecg_classifier_model_from_weights() {
  ECGClassifierModel model = {};
  model.hidden_size1 = hidden_size1;
  model.hidden_size2 = hidden_size2;
  model.hidden_size3 = hidden_size3;
  model.input_size1 = input_size1;
  model.input_size2 = input_size2;
  model.input_size3 = input_size3;
  model.input_size_fc = input_size4;
  model.output_size_fc = output_size4;
  model.Wfc = W4;
  model.bfc = b4;
  model.Wh1 = Wh1;
  model.Wh2 = Wh2;
  model.Wh3 = Wh3;
  model.Wr1 = Wr1;
  model.Wr2 = Wr2;
  model.Wr3 = Wr3;
  model.Wz1 = Wz1;
  model.Wz2 = Wz2;
  model.Wz3 = Wz3;
  model.bh1 = bh1;
  model.bh2 = bh2;
  model.bh3 = bh3;
  model.br1 = br1;
  model.br2 = br2;
  model.br3 = br3;
  model.bz1 = bz1;
  model.bz2 = bz2;
  model.bz3 = bz3;
  model.mu = mu;
  model.sg = sg;
  return model;
}
#endif

#endif
