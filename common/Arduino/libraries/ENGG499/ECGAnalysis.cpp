// ==========================================================================================
// ECG ANALYSIS
// AFIB classification + Heart Rate Processing Functions
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#include "ECGAnalysis.h"

#include "ANN.h"
#include "SignalCondition.h"

#include <math.h>

// CLASSIFIER DIMENSIONS ---------------------------------------------------------------------
// These mirror the trained 8x8x8 model in Weights.h.
static constexpr size_t kClassifierInput1Length = 3;
static constexpr size_t kClassifierHidden1Length = 8;
static constexpr size_t kClassifierHidden2Length = 8;
static constexpr size_t kClassifierHidden3Length = 8;
static constexpr size_t kClassifierOutputLength = 2;

// SHARED WORKING STORAGE --------------------------------------------------------------------
// Reuse the classifier window across calls so Lab 9 can capture directly into it.
// Note that this saves a TON of SRAM - slot 0 of the live capture points directly here!
static int16_t g_classifier_window[WINDOW_LENGTH] = {0};

// HELPERS -----------------------------------------------------------------------------------
// Short final windows get padded to a full 3000-sample classifier input, just in case
static void pad_classifier_window(int16_t* classifier_window, size_t sample_count);

// Run the trained model against one window
static ECGAnalysisResult run_classifier(
    const ECGClassifierModel* classifier_model,
    int16_t* classifier_window
);
static ECGAnalysisResult empty_analysis_result();
static float extract_classifier_feature1_at(const int16_t* classifier_window, size_t feature_index);

// The classifier always expects 3000 samples. If we don't get that, pad.
static void pad_classifier_window(int16_t* classifier_window, size_t sample_count) {
  const size_t copy_count = (sample_count < WINDOW_LENGTH) ? sample_count : WINDOW_LENGTH;
  const int16_t pad_value = (copy_count > 0) ? classifier_window[copy_count - 1] : 0;

  for (size_t i = copy_count; i < WINDOW_LENGTH; ++i) {
    classifier_window[i] = pad_value;
  }
}

// Feature 1 is ZCR over 100-sample windows. 
// To save some ram and processing, this uses a small ring.
// The shape is designed to match the training feature geometry
static float extract_classifier_feature1_at(
    const int16_t* classifier_window,
    size_t feature_index
) {

  constexpr size_t kZcrHopSize = kZcrWindowSize - kZcrOverlap;
  const size_t start = feature_index * kZcrHopSize;
  return kZcrFeatureScaleHz * static_cast<float>(
      zero_cross_rate_hyst(classifier_window + start, kZcrWindowSize));
}

// Empty result = nothing
static ECGAnalysisResult empty_analysis_result() {
  ECGAnalysisResult result = {};
  return result;
}

// SHARED WINDOW API -------------------------------------------------------------------------
// Expose the shared classifier window so RAM-constrained sketches can capture directly into it.
int16_t* ecg_analysis_classifier_window() {
  return g_classifier_window;
}

// Window kept for preprocessing plus classification in place
ECGAnalysisResult ecg_analysis_classify_mutable_window(
    int16_t* signal_window,
    size_t sample_count,
    const ECGClassifierModel* classifier_model
) {
  const size_t clipped_sample_count = (sample_count < WINDOW_LENGTH) ? sample_count : WINDOW_LENGTH;
  if (signal_window == nullptr || clipped_sample_count == 0 || classifier_model == nullptr) {
    return empty_analysis_result();
  }

  pad_classifier_window(signal_window, clipped_sample_count);
  return run_classifier(classifier_model, signal_window);
}

// AFIB CLASSIFICATION -----------------------------------------------------------------------
// Run feature extraction plus the three-layer 8x8x8 GRU model for one 3000-sample window
// This is the ONLY place where one 10-second ECG window becomes an AFIB/N score pair.
// The ordering here avoids storing feature1[264], feature2[257], and feature3[257] in SRAM -> savings!
static ECGAnalysisResult run_classifier(
    const ECGClassifierModel* classifier_model,
    int16_t* classifier_window
) {

  ECGAnalysisResult result = {};
  result.classification_valid = true;
  result.afib_score = 0.0f;
  result.normal_score = 0.0f;

  // Mean-center the classification window in place.
  // This mutates the input window, which is why diagnostics/HR have to come first
  offset_removal(classifier_window, classifier_window);

  // Local GRU state stays tiny: three 8-wide hidden states and a two-class output.
  float x_t[kClassifierInput1Length] = {1.0f, 1.0f, 1.0f};
  float h_t_minus_1[kClassifierHidden1Length] = {0.0f};
  float h_t1[kClassifierHidden1Length] = {0.0f};
  float h_t_minus_2[kClassifierHidden2Length] = {0.0f};
  float h_t2[kClassifierHidden2Length] = {0.0f};
  float h_t_minus_3[kClassifierHidden3Length] = {0.0f};
  float h_t3[kClassifierHidden3Length] = {0.0f};
  float output[kClassifierOutputLength] = {1.0f, 0.0f};
  float feature1_ring[kKurtosisWindowSize] = {0.0f};

  // Keep only the eight ZCR values needed for the current kurtosis frame.
  // This replaces the old full g_feature1 array
  for (size_t feature_index = 0; feature_index < kKurtosisWindowSize; ++feature_index) {
    feature1_ring[feature_index] = extract_classifier_feature1_at(
        classifier_window,
        feature_index
    );
  }

  // Walk across the feature time series one frame at a time
  for (size_t i = 0; i < ECG_ANALYSIS_FEATURE2_LENGTH; ++i) {
    const size_t feature1_ring_index = i % kKurtosisWindowSize;
    const float feature1_value = feature1_ring[feature1_ring_index];

    // Feature 2 is kurtosis over the eight ZCR values referenced above. 
    // The log1p transform matches the training pipeline and keeps peaky windows constrained somewhat
    float feature2 = kurtosis(feature1_ring, kKurtosisWindowSize);
    if (kKurtosisFeatureUsesLog1p) {
      if (feature2 < 0.0f) {
        feature2 = 0.0f;
      }
      feature2 = log1pf(feature2);
    }

    // Feature 3 is spectral entropy from the original centered sample window
    // Same as above, this is on-demand so we avoid too much storage
    const float feature3 = extract_feature2_spectral_entropy_at(
        classifier_window,
        WINDOW_LENGTH,
        static_cast<int>(i)
    );

    // Normalize the three features using the training-set mu/sigma values
    x_t[0] = (feature1_value - classifier_model->mu[0]) / classifier_model->sg[0];
    x_t[1] = (feature2 - classifier_model->mu[1]) / classifier_model->sg[1];
    x_t[2] = (feature3 - classifier_model->mu[2]) / classifier_model->sg[2];

    // GRU Layer 1 ----------------------------------------------------------------------
    gru_forward_step(
        x_t,
        h_t_minus_1,
        h_t1,
        classifier_model->Wz1,
        classifier_model->Wr1,
        classifier_model->Wh1,
        classifier_model->bz1,
        classifier_model->br1,
        classifier_model->bh1,
        classifier_model->input_size1,
        classifier_model->hidden_size1
    );

    for (size_t k = 0; k < static_cast<size_t>(classifier_model->hidden_size1); ++k) {
      h_t_minus_1[k] = h_t1[k];
    }

    // GRU Layer 2 ----------------------------------------------------------------------
    gru_forward_step(
        h_t1,
        h_t_minus_2,
        h_t2,
        classifier_model->Wz2,
        classifier_model->Wr2,
        classifier_model->Wh2,
        classifier_model->bz2,
        classifier_model->br2,
        classifier_model->bh2,
        classifier_model->input_size2,
        classifier_model->hidden_size2
    );

    for (size_t k = 0; k < static_cast<size_t>(classifier_model->hidden_size2); ++k) {
      h_t_minus_2[k] = h_t2[k];
    }

    // GRU Layer 3 ----------------------------------------------------------------------
    gru_forward_step(
        h_t2,
        h_t_minus_3,
        h_t3,
        classifier_model->Wz3,
        classifier_model->Wr3,
        classifier_model->Wh3,
        classifier_model->bz3,
        classifier_model->br3,
        classifier_model->bh3,
        classifier_model->input_size3,
        classifier_model->hidden_size3
    );

    for (size_t k = 0; k < static_cast<size_t>(classifier_model->hidden_size3); ++k) {
      h_t_minus_3[k] = h_t3[k];
    }

    // Fully-Connected Layer  -----------------------------------------------------------
    fully_connected_forward(
        h_t3,
        classifier_model->Wfc,
        classifier_model->bfc,
        output,
        classifier_model->input_size_fc,
        classifier_model->output_size_fc
    );

    // This frees up the ring slot for future ZCR/kurtosis
    const size_t next_feature_index = i + kKurtosisWindowSize;
    if (next_feature_index < ECG_ANALYSIS_FEATURE1_LENGTH) {
      feature1_ring[feature1_ring_index] = extract_classifier_feature1_at(
          classifier_window,
          next_feature_index
      );
    }
  }

  // Push our AFIB (A) and NORMAL (N) raw scores back to the caller. Lab 9 does
  // record-level voting from the score margins instead of trusting one window label.
  result.afib_score = output[0];
  result.normal_score = output[1];

  return result;
}

// RECORD HELPERS -------------------------------------------------------------------------
// These helpers keep record-level ECG postprocessing out of the sketch state machine.
static constexpr float kMeanPlusMaxMarginThreshold = 0.308795f;
static constexpr uint16_t kHeartRateSampleRateHz = 300;
static constexpr uint8_t kHeartRateSmoothSpan = 15;
static constexpr uint8_t kHeartRateThresholdPercent = 15;
static constexpr uint16_t kHeartRateMinPeakSpacingSamples = 100;
static constexpr uint16_t kHeartRateMinBpm = 35;
static constexpr uint16_t kHeartRateMaxBpm = 220;

// This is a causal moving average over derivative energy. It is integer-only
// to keep the computational cost simple.
static uint32_t push_smoothed_derivative_energy(
    uint32_t energy_ring[kHeartRateSmoothSpan],
    uint8_t& ring_index,
    uint8_t& ring_count,
    uint32_t& running_sum,
    uint32_t energy
) {
  if (ring_count < kHeartRateSmoothSpan) {
    ring_count++;
  } else {
    running_sum -= energy_ring[ring_index];
  }

  energy_ring[ring_index] = energy;
  running_sum += energy;
  ring_index = (ring_index + 1u) % kHeartRateSmoothSpan;
  return running_sum / ring_count;
}

// Resets heart rate, just in case
static void reset_heart_rate_energy_state(
    uint32_t energy_ring[kHeartRateSmoothSpan],
    uint8_t& ring_index,
    uint8_t& ring_count,
    uint32_t& running_sum
) {
  for (uint8_t i = 0; i < kHeartRateSmoothSpan; ++i) {
    energy_ring[i] = 0;
  }
  ring_index = 0;
  ring_count = 0;
  running_sum = 0;
}

// This is how I implement, per a sweep of various tests, a theoretically idyllic
// mean margin plus maximum margin approach as the record-level AFIB rule.
// margin = AFIB score - normal score. A positive margin means that window leaned AFIB
bool ecg_analysis_predict_lab9_record(float margin_sum, float max_margin) {

  const float mean_margin = margin_sum / static_cast<float>(kLab9RecordWindowCount);
  return (mean_margin + max_margin) > kMeanPlusMaxMarginThreshold;
}

// Diagnostics accumulator, mostly for testing (obviously...)
Lab9CaptureDiagnosticsAccumulator ecg_analysis_make_capture_diagnostics_accumulator() {
  Lab9CaptureDiagnosticsAccumulator accumulator = {};
  accumulator.global_min_counts = 4095;
  accumulator.min_window_peak_to_peak_counts = 4095;
  return accumulator;
}

// This helps me capture diagnostics during evaluation
void ecg_analysis_update_capture_diagnostics(
    Lab9CaptureDiagnosticsAccumulator& accumulator,
    const int16_t* window,
    uint16_t sample_count
) {
  if (sample_count == 0) {
    return;
  }

  // Lazy man's diagnostics: global p2p, weakest-window p2p, and a rough
  // zero-crossing rate.  Very valuable for identifying noise
  uint16_t window_min = 4095;
  uint16_t window_max = 0;
  uint32_t window_sum = 0;
  for (uint16_t sample_index = 0; sample_index < sample_count; ++sample_index) {
    int32_t sample = static_cast<int32_t>(window[sample_index]);
    if (sample < 0) {
      sample = 0;
    } else if (sample > 4095) {
      sample = 4095;
    }

    const uint16_t clipped_sample = static_cast<uint16_t>(sample);
    if (clipped_sample < window_min) {
      window_min = clipped_sample;
    }
    if (clipped_sample > window_max) {
      window_max = clipped_sample;
    }
    if (clipped_sample < accumulator.global_min_counts) {
      accumulator.global_min_counts = clipped_sample;
    }
    if (clipped_sample > accumulator.global_max_counts) {
      accumulator.global_max_counts = clipped_sample;
    }
    window_sum += clipped_sample;
  }

  accumulator.have_samples = true;
  accumulator.sample_count_total += sample_count;
  const uint16_t window_peak_to_peak = window_max - window_min;
  if (window_peak_to_peak < accumulator.min_window_peak_to_peak_counts) {
    accumulator.min_window_peak_to_peak_counts = window_peak_to_peak;
  }

  const int32_t window_mean = static_cast<int32_t>(window_sum / sample_count);
  int8_t crossing_state = (static_cast<int32_t>(window[0]) - window_mean >= 0) ? 1 : -1;
  for (uint16_t sample_index = 1; sample_index < sample_count; ++sample_index) {
    const int32_t centered_sample = static_cast<int32_t>(window[sample_index]) - window_mean;
    if (crossing_state == -1 && centered_sample >= kZcrThresholdHigh) {
      accumulator.zero_crossing_count_total++;
      crossing_state = 1;
    } else if (crossing_state == 1 && centered_sample <= -kZcrThresholdLow) {
      accumulator.zero_crossing_count_total++;
      crossing_state = -1;
    }
  }
}

// Finishes capture of diagnostics, a lot of this is a holdover from some tests during Lab9
Lab9CaptureDiagnostics ecg_analysis_finish_capture_diagnostics(
    const Lab9CaptureDiagnosticsAccumulator& accumulator
) {
  Lab9CaptureDiagnostics result = {};
  if (!accumulator.have_samples || accumulator.sample_count_total == 0) {
    return result;
  }

  result.peak_to_peak_counts = accumulator.global_max_counts - accumulator.global_min_counts;
  result.min_window_peak_to_peak_counts = accumulator.min_window_peak_to_peak_counts;
  result.zero_cross_rate_hz_x10 = static_cast<uint16_t>(
      (accumulator.zero_crossing_count_total *
       static_cast<uint32_t>(kHeartRateSampleRateHz) * 10u +
       (accumulator.sample_count_total / 2u)) /
      accumulator.sample_count_total
  );
  return result;
}

// New and shiny version of the heart rate analysis process, way faster and WAY more accurate
Lab9HeartRateResult ecg_analysis_estimate_heart_rate(
    const int16_t* samples,
    uint16_t sample_count
) {
  Lab9HeartRateResult result = {0, false};
  if (sample_count < (2u * kHeartRateMinPeakSpacingSamples)) {
    return result;
  }

  // Find the adaptive derivative-energy threshold for this window.
  uint32_t energy_ring[kHeartRateSmoothSpan];
  uint8_t ring_index = 0;
  uint8_t ring_count = 0;
  uint32_t running_sum = 0;
  reset_heart_rate_energy_state(energy_ring, ring_index, ring_count, running_sum);

  int16_t previous_sample = samples[0];
  uint32_t min_energy = 0xFFFFFFFFu;
  uint32_t max_energy = 0;

  // Short moving average suppresses single-sample spikes
  for (uint16_t sample_index = 1; sample_index < sample_count; ++sample_index) {
    const int16_t current_sample = samples[sample_index];
    const int32_t delta = static_cast<int32_t>(current_sample) -
        static_cast<int32_t>(previous_sample);
    previous_sample = current_sample;

    // Squaring the sample-to-sample delta makes QRS-like fast changes easier to see
    const uint32_t energy = static_cast<uint32_t>(delta * delta);
    const uint32_t smoothed_energy = push_smoothed_derivative_energy(
        energy_ring,
        ring_index,
        ring_count,
        running_sum,
        energy
    );
    if (smoothed_energy < min_energy) {
      min_energy = smoothed_energy;
    }
    if (smoothed_energy > max_energy) {
      max_energy = smoothed_energy;
    }
  }

  if (max_energy <= min_energy) {
    return result;
  }

  const uint32_t threshold = min_energy +
      ((max_energy - min_energy) * kHeartRateThresholdPercent) / 100u;

  // Count local energy peaks with spacing suppression so one broad beat does not
  // appear as several beats.
  reset_heart_rate_energy_state(energy_ring, ring_index, ring_count, running_sum);
  previous_sample = samples[0];

  // Here, this defines an active peak - it holds the strongest peak
  bool have_prev1 = false;
  bool have_prev2 = false;
  uint32_t prev1_energy = 0;
  uint32_t prev2_energy = 0;
  size_t prev1_index = 0;
  bool have_active_peak = false;
  size_t active_peak_index = 0;
  uint32_t active_peak_energy = 0;
  uint16_t beat_count = 0;

  // Similar-ish loop to above, this is basically a second pass
  for (uint16_t sample_index = 1; sample_index < sample_count; ++sample_index) {
    const int16_t current_sample = samples[sample_index];
    const int32_t delta = static_cast<int32_t>(current_sample) -
        static_cast<int32_t>(previous_sample);
    previous_sample = current_sample;

    const uint32_t energy = static_cast<uint32_t>(delta * delta);
    const uint32_t current_energy = push_smoothed_derivative_energy(
        energy_ring,
        ring_index,
        ring_count,
        running_sum,
        energy
    );

    if (!have_prev1) {
      prev1_energy = current_energy;
      prev1_index = sample_index;
      have_prev1 = true;
      continue;
    }

    if (!have_prev2) {
      prev2_energy = prev1_energy;
      prev1_energy = current_energy;
      prev1_index = sample_index;
      have_prev2 = true;
      continue;
    }

    if (prev1_energy > threshold &&
        prev1_energy >= prev2_energy &&
        prev1_energy > current_energy) {
      if (!have_active_peak) {
        have_active_peak = true;
        active_peak_index = prev1_index;
        active_peak_energy = prev1_energy;
      } else if ((prev1_index - active_peak_index) >= kHeartRateMinPeakSpacingSamples) {
        beat_count++;
        active_peak_index = prev1_index;
        active_peak_energy = prev1_energy;
      } else if (prev1_energy > active_peak_energy) {
        active_peak_index = prev1_index;
        active_peak_energy = prev1_energy;
      }
    }

    prev2_energy = prev1_energy;
    prev1_energy = current_energy;
    prev1_index = sample_index;
  }

  if (have_active_peak) {
    beat_count++;
  }

  if (beat_count < 3) {
    return result;
  }

  // Convert beats counted in this sample window into BPM.
  const uint32_t numerator =
      60u * static_cast<uint32_t>(beat_count) * kHeartRateSampleRateHz;
  const uint16_t bpm = static_cast<uint16_t>(
      (numerator + (static_cast<uint32_t>(sample_count) / 2u)) /
      static_cast<uint32_t>(sample_count)
  );

  result.heart_rate_bpm = bpm;
  result.valid = bpm >= kHeartRateMinBpm && bpm <= kHeartRateMaxBpm;
  return result;
}

// Lab 9-refined heart rate result, summarizes the valid rates we calculated (see above)
Lab9HeartRateResult ecg_analysis_summarize_heart_rates(
    uint16_t valid_rates[kLab9RecordWindowCount],
    uint8_t valid_rate_count
) {

  // Jettison if junk
  Lab9HeartRateResult result = {0, false};
  if (valid_rate_count == 0) {
    return result;
  }

  // Lazy/tiny in-place sort to output valid rates
  for (uint8_t i = 0; i < valid_rate_count; ++i) {
    for (uint8_t j = i + 1; j < valid_rate_count; ++j) {
      if (valid_rates[j] < valid_rates[i]) {
        const uint16_t tmp = valid_rates[i];
        valid_rates[i] = valid_rates[j];
        valid_rates[j] = tmp;
      }
    }
  }

  // With two windows I average instead of choosing one arbitrarily.
  if (valid_rate_count == 2) {
    result.heart_rate_bpm = static_cast<uint16_t>((valid_rates[0] + valid_rates[1] + 1u) / 2u);

  // With one or three windows, the middle element after sorting is the most robust summary.
  } else {
    result.heart_rate_bpm = valid_rates[valid_rate_count / 2u];
  }
  
  // Output HR
  result.valid = true;
  return result;
}
