// ==========================================================================================
// SIGNALCONDITION
// Feature extraction, FFT-type stuff, and the like
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#ifndef SIGNALCONDITION_H
#define SIGNALCONDITION_H

// IMPORTS -----------------------------------------------------------------------------------
#include "Arduino.h"
#include <stddef.h>

// WINDOW CONSTANTS --------------------------------------------------------------------------
// The classifier sees 10 seconds of ECG at 300 Sa/s
#define WINDOW_LENGTH 3000

// FEATURE PARAMETERS ------------------------------------------------------------------------
// Largely hammered out in previous labs, but tweaked a bit
constexpr int kZcrThresholdHigh = 10;
constexpr int kZcrThresholdLow = 10;
constexpr int kZcrWindowSize = 100;
constexpr int kZcrOverlap = 89;
constexpr float kZcrFeatureScaleHz = 300.0f;
constexpr int kKurtosisWindowSize = 8;
constexpr int kKurtosisOverlap = 7;
constexpr bool kKurtosisFeatureUsesLog1p = true;
constexpr int kSpectralEntropyFftLength = 128;

// Only positive-frequency bins are used for entropy
constexpr int kSpectralEntropyBinCount = kSpectralEntropyFftLength / 2;

// FEATURE LENGTHS ---------------------------------------------------------------------------
// Calculate feature lengths at compile time
constexpr size_t signal_feature_window_count(
  size_t input_length,
  size_t window_size,
  size_t overlap
) {
  return (window_size == 0 || overlap >= window_size || input_length < window_size)
      ? 0
      : 1 + ((input_length - window_size) / (window_size - overlap));
}

constexpr size_t kFeature1Length =
    signal_feature_window_count(WINDOW_LENGTH, kZcrWindowSize, kZcrOverlap);
constexpr size_t kFeature2Length =
    signal_feature_window_count(kFeature1Length, kKurtosisWindowSize, kKurtosisOverlap);

// BASIC HELPERS -----------------------------------------------------------------------------
// Lab 9 uses these through ECGAnalysis.
void offset_removal(const int16_t* x, int16_t* y);
double zero_cross_rate_hyst(const int16_t* x, int winSize);
float kurtosis(const float* x, int winSize);

// FEATURE EXTRACTION ------------------------------------------------------------------------
// Lab 9 computes this one frame at a time so it does not keep full feature arrays in SRAM.
float extract_feature2_spectral_entropy_at(
  const int16_t* x,
  int arraySize,
  int frameIndex
);

#endif
