// ==========================================================================================
// ANN
// Implements the GRU, such as the GRU forward step
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#ifndef ANN_H
#define ANN_H

// IMPORTS -----------------------------------------------------------------------------------
#include "Arduino.h"

// GRU / DENSE HELPERS -----------------------------------------------------------------------
// Small forward-pass helpers.
// To save SRAM, the helpers write into caller-provided arrays so they do not own any global SRAM.
// Ugly, but effective
void gru_forward_step(
    float* x_t,
    float* h_t_minus_1,
    float* h_t,
    const float* Wz,
    const float* Wr,
    const float* Wh,
    const float* bz,
    const float* br,
    const float* bh,
    int input_size, 
    int hidden_size
);

// Final affine layer used after the GRU stack.
void fully_connected_forward(
    float* input,
    const float* weights,
    const float* bias,
    float* output,
    int input_size, 
    int output_size
);

#endif
