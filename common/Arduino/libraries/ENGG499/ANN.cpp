// ==========================================================================================
// ANN
// Implements the GRU, such as the GRU forward step
// Kirk A. Sigmon - kirk.a.sigmon.th@dartmouth.edu
// ==========================================================================================

#include "ANN.h"

// GRU STEP ----------------------------------------------------------------------------------
// One recurrent step. The variable names intentionally match the
// MATLAB equations, which makes debugging the export much less annoying.
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
) {

    // Define sigmoid locally because it is only used inside one GRU step
    auto sigmoid = [](float x) -> float {
        return 1.0f / (1.0f + __builtin_expf(-x));
    };

    // One timestep of GRU state. Stack allocation is small and avoids permanent SRAM.
    float r_t[hidden_size];
    float z_t[hidden_size];
    float h_hat_t[hidden_size];

    // Each exported GRU matrix stores input weights followed by recurrent weights in each row.
    const int row_size = input_size + hidden_size;

    // FIRST TWO SUB-EQUATIONS -------------------------------------------------------
    // Compute the reset gate r_t and update gate z_t.
    // r_t decides how much previous hidden-state information is allowed into the candidate
    // hidden state. z_t decides how much of the previous hidden state survives into the new
    // hidden state. Both are sigmoid gates, so every element is between 0 and 1.
    for (int i = 0; i < hidden_size; ++i) {

        // Keep the MATLAB names split out so the exported equations can be checked by eye.
        float Wrx_t = 0.0f;
        float Urh_t_minus_1 = 0.0f;
        float Wzx_t = 0.0f;
        float Uzh_t_minus_1 = 0.0f;

        // First half of the row multiplies the current input feature vector.
        for (int j = 0; j < input_size; ++j) {
            Wrx_t += Wr[i * row_size + j] * x_t[j];
            Wzx_t += Wz[i * row_size + j] * x_t[j];
        }

        // Second half of the row multiplies the previous hidden state.
        for (int j = 0; j < hidden_size; ++j) {
            Urh_t_minus_1 += Wr[i * row_size + input_size + j] * h_t_minus_1[j];
            Uzh_t_minus_1 += Wz[i * row_size + input_size + j] * h_t_minus_1[j];
        }

        // Bias is applied last, then sigmoid squashes each gate into [0, 1].
        r_t[i] = sigmoid(Wrx_t + Urh_t_minus_1 + br[i]);
        z_t[i] = sigmoid(Wzx_t + Uzh_t_minus_1 + bz[i]);
    }

    // THIRD SUB-EQUATION FOR h_hat_t -------------------------------------------------
    // Build the candidate hidden state
    for (int i = 0; i < hidden_size; ++i) {

        float Whx_t = 0.0f;
        float Uhh_t_minus_1 = 0.0f;

        for (int j = 0; j < input_size; ++j) {
            Whx_t += Wh[i * row_size + j] * x_t[j];
        }

        // Build U_h*h_t_minus_1, then apply the reset gate after the recurrent
        // matrix multiplication to match MATLAB's after-multiplication mode.
        for (int j = 0; j < hidden_size; ++j) {
            Uhh_t_minus_1 += Wh[i * row_size + input_size + j] * h_t_minus_1[j];
        }

        // Finally, calculate h_hat_t using the helper equations above
        h_hat_t[i] = __builtin_tanhf(Whx_t + r_t[i] * Uhh_t_minus_1 + bh[i]);
    }

    // FOURTH (AND FINAL) EQUATION FOR h_t --------------------------------------------
    // Blend the previous hidden state and candidate hidden state using the update gate.
    // If z_t is close to 1, memory carries forward. If z_t is close to 0, the new candidate
    // replaces it.
    for (int i = 0; i < hidden_size; ++i) {

        h_t[i] = z_t[i] * h_t_minus_1[i] + (1.0f - z_t[i]) * h_hat_t[i];
    }
}

// FULLY CONNECTED STEP ----------------------------------------------------------------------
// Small final affine layer after the GRU stack.
void fully_connected_forward(
    float* input,
    const float* weights,
    const float* bias,
    float* output,
    int input_size, 
    int output_size
) {
    // This is a dense layer WITHOUT activation. 
    // After all, we now handle margin later
    for (int i = 0; i < output_size; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < input_size; ++j) {
            sum += weights[i * input_size + j] * input[j];
        }
        sum += bias[i]; 
        output[i] = sum;
    }
}
