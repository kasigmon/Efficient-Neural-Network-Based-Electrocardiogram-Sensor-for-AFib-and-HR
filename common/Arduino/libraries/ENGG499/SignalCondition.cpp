// ==========================================================================================
// SIGNALCONDITION
// Feature extraction, FFT-type stuff, and the like
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#include "SignalCondition.h"

#ifndef ARM_MATH_CM0PLUS
#define ARM_MATH_CM0PLUS
#endif
#include <arm_math.h>

#include <math.h>

namespace {

// SPECTRAL ENTROPY STATE --------------------------------------------------------------------
// Hamming window is precomputed in Q15 so the FFT path does not spend time doing computationally
// expensive trig. Thankfully, this can sit in flash (the FFT buffer doesn't)
static const q15_t kHammingWindowQ15[kSpectralEntropyFftLength] = {
  2621, 2640, 2695, 2787, 2916, 3080, 3281, 3516,
  3787, 4091, 4429, 4799, 5201, 5633, 6095, 6585,
  7102, 7645, 8213, 8804, 9417, 10050, 10702, 11371,
  12055, 12754, 13464, 14185, 14914, 15650, 16391, 17135,
  17881, 18626, 19369, 20107, 20840, 21565, 22281, 22985,
  23677, 24354, 25014, 25657, 26280, 26882, 27462, 28018,
  28548, 29052, 29528, 29975, 30393, 30779, 31133, 31454,
  31741, 31994, 32212, 32395, 32542, 32652, 32726, 32762,
  32762, 32726, 32652, 32542, 32395, 32212, 31994, 31741,
  31454, 31133, 30779, 30393, 29975, 29528, 29052, 28548,
  28018, 27462, 26882, 26280, 25657, 25014, 24354, 23677,
  22985, 22281, 21565, 20840, 20107, 19369, 18626, 17881,
  17135, 16391, 15650, 14914, 14185, 13464, 12754, 12055,
  11371, 10702, 10050, 9417, 8804, 8213, 7645, 7102,
  6585, 6095, 5633, 5201, 4799, 4429, 4091, 3787,
  3516, 3281, 3080, 2916, 2787, 2695, 2640, 2621
};

static arm_cfft_radix2_instance_q15 g_spectral_fft_instance;
static bool g_spectral_fft_initialized = false;
static q15_t g_spectral_fft_buffer[2 * kSpectralEntropyFftLength] = {0};

// Initialize CMSIS's FFT state the 1st time spectral entropy is asked for
bool ensure_spectral_fft_initialized() {
  if (g_spectral_fft_initialized) {
    return true;
  }

  g_spectral_fft_initialized =
      (arm_cfft_radix2_init_q15(
          &g_spectral_fft_instance,
          kSpectralEntropyFftLength,
          0,
          1) == ARM_MATH_SUCCESS);
  return g_spectral_fft_initialized;
}

// Quick way to get absolute value
int32_t abs_i32(int32_t value) {
  return (value < 0) ? -value : value;
}

// Convert one ECG sample into a windowed Q15 FFT input sample.
q15_t spectral_sample_to_q15(int16_t sample, int32_t max_abs, size_t window_index) {
  // Normalize the current entropy frame by its largest absolute sample, then apply the Hamming
  // window. This keeps the fixed-point FFT from clipping on high-amplitude ECG captures.
  if (max_abs <= 0) {
    return 0;
  }

  int32_t scaled = (static_cast<int32_t>(sample) * 32767L) / max_abs;
  if (scaled > 32767L) {
    scaled = 32767L;
  } else if (scaled < -32768L) {
    scaled = -32768L;
  }

  const int32_t windowed = (scaled * kHammingWindowQ15[window_index]) >> 15;
  if (windowed > 32767L) {
    return 32767;
  }
  if (windowed < -32768L) {
    return -32768;
  }
  return static_cast<q15_t>(windowed);
}

// FEATURE HELPERS ---------------------------------------------------------------------------
// Mean-center one full classifier window.
template <typename SampleType>
void offset_removal_impl(const SampleType* x, SampleType* y) {

  // Don't handle null stuff
  if (x == nullptr || y == nullptr) {
    return;
  }

  long signal_sum = 0;
  for (int i = 0; i < WINDOW_LENGTH; ++i) {
    signal_sum += x[i];
  }

  const long offset = signal_sum / WINDOW_LENGTH;
  for (int i = 0; i < WINDOW_LENGTH; ++i) {
    y[i] = static_cast<SampleType>(x[i] - offset);
  }
}

// Zero-crossing rate with hysteresis
template <typename SampleType>
double zero_cross_rate_hyst_impl(
  const SampleType* x,
  int winSize
) {

  // Hysteresis helps account for some noise
  if (x == nullptr || winSize <= 1) {
    return 0;
  }

  int zeroCrossings = 0;
  const int upperThresh = abs(kZcrThresholdHigh);
  const int lowerThresh = -abs(kZcrThresholdLow);

  int state = (x[0] >= 0) ? 1 : -1;

  // Calculate the ZCR manually
  for (int i = 1; i < winSize; ++i) {
    if (state == -1 && x[i] >= upperThresh) {
      zeroCrossings++;
      state = 1;
    } else if (state == 1 && x[i] <= lowerThresh) {
      zeroCrossings++;
      state = -1;
    }
  }

  return static_cast<double>(zeroCrossings) / static_cast<double>(winSize);
}

} 

// BASIC HELPERS -----------------------------------------------------------------------------
// Mean-center an int16_t window
void offset_removal(
  const int16_t* x,
  int16_t* y
){
  offset_removal_impl(x, y);
}

// Zero-crossing rate wrapper for captured ECG window(s)
double zero_cross_rate_hyst(
  const int16_t* x,
  int winSize
){
  return zero_cross_rate_hyst_impl(x, winSize);
}

// Kurtosis of one FLOAT window
// This is feature 2 - tells classifier whether ZCR is flat or has sharpness
// We later treat this as log1p(kurtosis) to keep it somewhat bounded
float kurtosis(
  const float* x,
  int winSize
) {
  if (x == nullptr || winSize <= 0) {
    return 0.0f;
  }

  float mean = 0.0f;
  for (int i = 0; i < winSize; ++i) {
    mean += x[i];
  }
  mean /= static_cast<float>(winSize);

  float var = 0.0f;
  float fourth_moment = 0.0f;
  for (int i = 0; i < winSize; ++i) {
    const float centered = x[i] - mean;
    const float centered2 = centered * centered;
    var += centered2;
    fourth_moment += centered2 * centered2;
  }
  var /= static_cast<float>(winSize);
  fourth_moment /= static_cast<float>(winSize);
  
  // If variance collapsed to zero, return a neutral-ish bounded value.
  if (var > 0) {
    return fourth_moment / (var * var);
  }

  return 2.0f;
}

// FEATURE 3: SPECTRAL ENTROPY ---------------------------------------------------------------
// Compute one spectral entropy frame on demand using the Q15 FFT state above.
float extract_feature2_spectral_entropy_at(
  const int16_t* x,
  int arraySize,
  int frameIndex
) {
  if (x == nullptr || arraySize <= 0 || frameIndex < 0) {
    return 0.0f;
  }

  if (!ensure_spectral_fft_initialized()) {
    return 0.0f;
  }

  // Each frame advances by the ZCR, then looks backward over a 128-sample FFT window.
  constexpr int kHopSize = kZcrWindowSize - kZcrOverlap;
  constexpr float kInvLogBinCount = 1.0f / 4.1588830833596715f;

  // Early frames do not have 128 samples of history yet, so they are left-padded with zeros.
  const int raw_end = (kZcrWindowSize - 1) + frameIndex * kHopSize;
  const int raw_start = raw_end - kSpectralEntropyFftLength + 1;

  int32_t max_abs = 0;
  for (int n = 0; n < kSpectralEntropyFftLength; ++n) {
    const int source_index = raw_start + n;
    if (source_index >= 0 && source_index < arraySize) {
      const int32_t sample_abs = abs_i32(static_cast<int32_t>(x[source_index]));
      if (sample_abs > max_abs) {
        max_abs = sample_abs;
      }
    }
  }

  // Fill the interleaved real/imaginary FFT buffer. Out-of-range samples are zero padding at the
  // start of early frames
  for (int n = 0; n < kSpectralEntropyFftLength; ++n) {
    const int source_index = raw_start + n;
    const int16_t sample =
        (source_index >= 0 && source_index < arraySize) ? x[source_index] : 0;
    g_spectral_fft_buffer[2 * n] = spectral_sample_to_q15(sample, max_abs, n);
    g_spectral_fft_buffer[2 * n + 1] = 0;
  }

  arm_cfft_radix2_q15(&g_spectral_fft_instance, g_spectral_fft_buffer);

  // Ignore DC and normalize
  float power_sum = 0.0f;
  for (int bin = 1; bin <= kSpectralEntropyBinCount; ++bin) {
    const float real_value = static_cast<float>(g_spectral_fft_buffer[2 * bin]);
    const float imag_value = static_cast<float>(g_spectral_fft_buffer[2 * bin + 1]);
    power_sum += real_value * real_value + imag_value * imag_value;
  }

  if (power_sum <= 0.0f) {
    return 0.0f;
  }

  float entropy = 0.0f;
  for (int bin = 1; bin <= kSpectralEntropyBinCount; ++bin) {
    const float real_value = static_cast<float>(g_spectral_fft_buffer[2 * bin]);
    const float imag_value = static_cast<float>(g_spectral_fft_buffer[2 * bin + 1]);
    const float power = real_value * real_value + imag_value * imag_value;
    if (power > 0.0f) {
      const float probability = power / power_sum;
      entropy -= probability * logf(probability);
    }
  }

  return entropy * kInvLogBinCount;
}
