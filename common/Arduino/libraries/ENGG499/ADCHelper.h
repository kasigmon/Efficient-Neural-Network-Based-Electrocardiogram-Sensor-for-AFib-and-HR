// ==========================================================================================
// ADC HELPER
// Handles a lot of signal receipt, processing, etc.
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#ifndef ADCHelper_H
#define ADCHelper_H

// IMPORTS -----------------------------------------------------------------------------------
#include <Adafruit_ZeroDMA.h>

// ADC / DMA CONSTANTS -----------------------------------------------------------------------
// The ADC runs around 21 kSa/s; averaging 70 raw samples gives the ECG pipeline 300 Sa/s.
// The classifier, HR estimator, and MATLAB/AD3 test scripts all assume the decimated ECG stream
// is 300 samples/second.
#define DECIMATION_FACTOR 70

#define ADC_DEFAULT_PIN A2
#define SAMPLE_BLOCK_LENGTH DECIMATION_FACTOR

// CALLBACKS ---------------------------------------------------------------------------------
// Gets one averaged sample whenever a DMA half-buffer finishes
typedef void (*ADCSampleCallback)(uint16_t sample);

// API ---------------------------------------------------------------------------------------
// The helper owns the ADC/DMA configuration, leaving other stuff to the Arduino sketch
void adc_set_input_pin(uint32_t pin);
void adc_init();
void dma_init();
void adc_set_sample_callback(ADCSampleCallback callback);
void dma_callback(Adafruit_ZeroDMA* dma);

#endif
