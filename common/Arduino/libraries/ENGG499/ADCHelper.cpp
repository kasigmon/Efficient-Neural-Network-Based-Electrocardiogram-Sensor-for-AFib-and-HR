// ==========================================================================================
// ADC HELPER
// Handles a lot of signal receipt, processing, etc.
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#include "ADCHelper.h"

// DMA STATE --------------------------------------------------------------------
extern Adafruit_ZeroDMA ADC_DMA;
extern DmacDescriptor* dmac_descriptor_1;
extern DmacDescriptor* dmac_descriptor_2;

extern uint16_t adc_buffer[SAMPLE_BLOCK_LENGTH * 2];
extern volatile bool filling_first_half;
extern volatile uint16_t* active_adc_buffer;
extern volatile bool adc_buffer_filled;

static void adc_sync();

namespace {

// SAMPLE CALLBACK ---------------------------------------------------------------------------
// Lab 9 consumes ONE averaged sample per DMA half-buffer, which gets us to roughly 300 Sa/s.
ADCSampleCallback g_adc_sample_callback = nullptr;
uint32_t g_adc_input_pin = ADC_DEFAULT_PIN;

// DMA gives us a small burst of fast raw ADC samples. I average the whole burst here so the
// sketch only sees the 300 Sa/s stream that the ECG pipeline expects.
uint16_t average_dma_block(const volatile uint16_t* data) {
  uint32_t sum = 0;
  for (int i = 0; i < SAMPLE_BLOCK_LENGTH; ++i) {
    sum += data[i];
  }
  return static_cast<uint16_t>(sum / SAMPLE_BLOCK_LENGTH);
}

}

// ADC SETUP ---------------------------------------------------------------------------------
// Configure the SAMD21 ADC for 12-bit reads on the ECG input pin

// The Arduino sketch chooses A2 for AD3 mode or A4 for electrode mode
void adc_set_input_pin(uint32_t pin) {
  g_adc_input_pin = pin;
}

// Initialize the ADC
void adc_init() {

  // analogRead() lets the Arduino core map the selected analog pin into the ADC peripheral
  // before I take over the lower-level registers -> electrode mode or AD3 mode switch works.
  analogRead(g_adc_input_pin);
  ADC->CTRLA.bit.ENABLE = 0;
  adc_sync();

  // Half-gain VCC reference used so 12-bit conversion range matches front-end.
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1;
  adc_sync();

  // The active analog channel is the only pin-specific register setting
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[g_adc_input_pin].ulADCChannelNumber;
  adc_sync();

  // No hardware averaging
  ADC->AVGCTRL.reg = 0;
  ADC->SAMPCTRL.reg = 4;
  adc_sync();

  // Free-run mode keeps the ADC producing samples
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_12BIT;
  adc_sync();
  ADC->CTRLA.bit.ENABLE = 1;
  adc_sync();
}

// DMA SETUP ---------------------------------------------------------------------------------
// Ping-pong two half-buffers so the ADC can keep running while the sketch handles the last
// finished block -> less RAM needed to do a lot of this

void dma_init() {

  // Each descriptor copies ADC->RESULT into one half of the sketch-owned adc_buffer. When one
  // half finishes, the DMA interrupt fires and the next descriptor continues filling the other.
  ADC_DMA.allocate();
  ADC_DMA.setTrigger(ADC_DMAC_ID_RESRDY);
  ADC_DMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  dmac_descriptor_1 = ADC_DMA.addDescriptor(
    (void*)(&ADC->RESULT.reg),
    adc_buffer,
    SAMPLE_BLOCK_LENGTH,
    DMA_BEAT_SIZE_HWORD,
    false,
    true);
  dmac_descriptor_1->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

  dmac_descriptor_2 = ADC_DMA.addDescriptor(
    (void*)(&ADC->RESULT.reg),
    adc_buffer + SAMPLE_BLOCK_LENGTH,
    SAMPLE_BLOCK_LENGTH,
    DMA_BEAT_SIZE_HWORD,
    false,
    true);
  dmac_descriptor_2->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;

  ADC_DMA.loop(true);
  ADC_DMA.setCallback(dma_callback);
}

// Let the sketch subscribe to one decimated sample per completed DMA block
void adc_set_sample_callback(ADCSampleCallback callback) {
  g_adc_sample_callback = callback;
}

// ADC registers synchronized through peripheral bus, gotta wait
static void adc_sync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1) {
  }
}

// DMA CALLBACK -------------------------------------------------------------------------------
// Flip to the half-buffer that just finished, then hand the avg sample to the Arduino sketch.
void dma_callback(Adafruit_ZeroDMA* dma) {
  (void)dma;

  // If we're filling the first half...
  if (filling_first_half) {

    // Second half is safe to read.
    active_adc_buffer = &adc_buffer[SAMPLE_BLOCK_LENGTH];
    filling_first_half = false;
  
  // Otherwise, DMA is filling the second half, so the first half OK to read.
  } else {
    
    active_adc_buffer = &adc_buffer[0];
    filling_first_half = true;
  }
  adc_buffer_filled = true;

  if (g_adc_sample_callback != nullptr) {
    g_adc_sample_callback(average_dma_block(active_adc_buffer));
  }
}
