/*
 * Copyright (C) 2013 Ron Wright.
 *
 * This file is part of Ron's Multiband Compressor.
 *
 * Ron's Multiband Compressor is free software: you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either version
 * 2.1 of the License, or (at your option) any later version.
 *
 * Ron's Multiband Compressor is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Ron's Multiband Compressor.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "linkwitzriley.h"
#include <string.h>
#include <math.h>

/* Converts the specified band index into a registry index */
#define BAND_TO_REG_IDX( i ) ( ( i ) + REG_IDX_LPF_1 )

/* Computes filter coefficients and stores them in the context, given the split
 * index and a boolean value that determines whether the filter is to be set in
 * 24 dB mode or 12 dB mode */
static void ComputeFilterCoeffs( four_band_lr_context_t * p_ctx, int i_split,
                                 bool b_24dB_mode )
{
    float f_a0, f_a1, f_a2, f_b0l, f_b1l, f_b2l, f_b0h, f_b1h, f_b2h;
    float f_fc = p_ctx->info.f_split_frequencies[i_split];
    float f_fN = 0.5f * p_ctx->info.i_rate;
    float f_w0 = f_fc < f_fN ? (float) M_PI * f_fc / f_fN : (float) M_PI;
    float f_hw0 = 0.5f * f_w0;
    float f_cosw0 = cosf( f_w0 );
    float f_sinhw0 = sinf( f_hw0 );
    float f_coshw0 = cosf( f_hw0 );

    /* Numerator coefficients for LPF and HPF */
    f_b1l = f_sinhw0 * f_sinhw0;
    f_b1h = f_coshw0 * f_coshw0;
    if( b_24dB_mode )
    {
        f_b1l *= 2.0f;
        f_b1h *= -2.0f;
    }

    /* Numerator coefficients for LPF */
    f_b0l = 0.5f * f_b1l;
    f_b2l = f_b0l;

    /* Numerator coefficients for HPF */
    f_b0h = -0.5f * f_b1h;
    f_b2h = f_b0h;

    if( b_24dB_mode )
    {
        /* Denominator coefficients for LPF and HPF */
        float f_alpha = sinf( f_w0 ) * (float) M_SQRT1_2;
        f_a0 = 1.0f + f_alpha;
        f_a1 = -2.0f * f_cosw0;
        f_a2 = 1.0f - f_alpha;

        /* Normalize them */
        f_a1 /= f_a0;
        f_a2 /= f_a0;

        /* We only need an all pass filter for the first and third splits */
        if( ( i_split & 1 ) == 0 )
        {
            /* Use second order all pass filter */

            int i_index = i_split >> 1;

            /* Compute the normalized APF coefficients */

            p_ctx->apf[i_index].f_a0 = f_a1;
            p_ctx->apf[i_index].f_a1 = f_a2;
            p_ctx->apf[i_index].f_b0 = f_a2;
            p_ctx->apf[i_index].f_b1 = f_a1;
            p_ctx->apf[i_index].f_b2 = 1.0f;
        }
    }
    else
    {
        float f_hw0pqp = f_hw0 + (float) M_PI_4;
        float f_sinhw0pqp = sinf( f_hw0pqp );
        float f_coshw0pqp = cosf( f_hw0pqp );

        /* Denominator coefficients for LPF and HPF */
        f_a0 = f_sinhw0pqp * f_sinhw0pqp;
        f_a1 = -f_cosw0;
        f_a2 = f_coshw0pqp * f_coshw0pqp;

        /* Normalize them */
        f_a1 /= f_a0;
        f_a2 /= f_a0;

        /* We only need an all pass filter for the first and third splits */
        if( ( i_split & 1 ) == 0 )
        {
            /* Use first order all pass filter */

            int i_index = i_split >> 1;

            /* Compute the normalized APF coefficients */

            float f_a0a = -f_coshw0pqp / f_sinhw0pqp;
            p_ctx->apf[i_index].f_a0 = f_a0a;
            p_ctx->apf[i_index].f_b0 = f_a0a;
            p_ctx->apf[i_index].f_b1 = 1.0f;
        }
    }

    /* Compute the normalized LPF and HPF coefficients */

    p_ctx->lpf[i_split].f_a0 = f_a1;
    p_ctx->lpf[i_split].f_a1 = f_a2;
    p_ctx->lpf[i_split].f_b0 = f_b0l / f_a0;
    p_ctx->lpf[i_split].f_b1 = f_b1l / f_a0;
    p_ctx->lpf[i_split].f_b2 = f_b2l / f_a0;

    p_ctx->hpf[i_split].f_a0 = f_a1;
    p_ctx->hpf[i_split].f_a1 = f_a2;
    p_ctx->hpf[i_split].f_b0 = f_b0h / f_a0;
    p_ctx->hpf[i_split].f_b1 = f_b1h / f_a0;
    p_ctx->hpf[i_split].f_b2 = f_b2h / f_a0;
}

/* Copies the sample buffer into the history registry at the specified registry
 * index and the history pointer (which should be one past the current history
 * pointer value) */
static void StoreInput( four_band_lr_context_t * p_ctx, const float * pf_input,
                        int i_reg_idx, int i_ptr_p1 )
{
    memcpy( &p_ctx->history.f_x[i_reg_idx][i_ptr_p1][0], pf_input,
            p_ctx->info.i_channels * sizeof( *pf_input ) );
}

/* Processes an order 1 filter, given the filter coefficients, the input and
 * output history registry indices, and the needed history pointers, and stores
 * the newly computed sample in the context's history */
static void ProcessOrder1Filter( four_band_lr_context_t * p_ctx,
                                 second_order_filter_coeff_t * p_coeffs,
                                 int i_input_reg_idx, int i_output_reg_idx,
                                 int i_ptr_p0, int i_ptr_p1 )
{
    for( int i_channel = 0; i_channel < p_ctx->info.i_channels; i_channel++ )
    {
        float f_x0 = p_ctx->history.f_x[i_input_reg_idx][i_ptr_p1][i_channel];
        float f_x1 = p_ctx->history.f_x[i_input_reg_idx][i_ptr_p0][i_channel];
        float f_y1 = p_ctx->history.f_x[i_output_reg_idx][i_ptr_p0][i_channel];
        p_ctx->history.f_x[i_output_reg_idx][i_ptr_p1][i_channel] =
            p_coeffs->f_b0 * f_x0 + p_coeffs->f_b1 * f_x1
                                  - p_coeffs->f_a0 * f_y1;
    }
}

/* Processes an order 2 filter, given the filter coefficients, the input and
 * output history registry indices, and the needed history pointers, and stores
 * the newly computed sample in the context's history */
static void ProcessOrder2Filter( four_band_lr_context_t * p_ctx,
                                 second_order_filter_coeff_t * p_coeffs,
                                 int i_input_reg_idx, int i_output_reg_idx,
                                 int i_ptr_p00, int i_ptr_p10, int i_ptr_p20,
                                 int i_ptr_p11 )
{
    four_band_lr_reg_t * p_hist = &p_ctx->history;
    for( int i_channel = 0; i_channel < p_ctx->info.i_channels; i_channel++ )
    {
        float f_x0 = p_hist->f_x[i_input_reg_idx][i_ptr_p10][i_channel];
        float f_x1 = p_hist->f_x[i_input_reg_idx][i_ptr_p00][i_channel];
        float f_x2 = p_hist->f_x[i_input_reg_idx][i_ptr_p20][i_channel];
        float f_y1 = p_hist->f_x[i_output_reg_idx][i_ptr_p00][i_channel];
        float f_y2 = p_hist->f_x[i_output_reg_idx][i_ptr_p20][i_channel];
        p_hist->f_x[i_output_reg_idx][i_ptr_p11][i_channel] =
        p_hist->f_x[i_output_reg_idx][i_ptr_p10][i_channel] =
            p_coeffs->f_b0 * f_x0 + p_coeffs->f_b1 * f_x1
                                  + p_coeffs->f_b2 * f_x2
                                  - p_coeffs->f_a0 * f_y1
                                  - p_coeffs->f_a1 * f_y2;
    }
}

/* Processes an order 4 filter, given the filter coefficients, the input and
 * output history registry indices, and the needed history pointers, and stores
 * the newly computed sample in the context's history */
static void ProcessOrder4Filter( four_band_lr_context_t * p_ctx,
                                 second_order_filter_coeff_t * p_coeffs,
                                 int i_input_reg_idx, int i_output_reg_idx,
                                 int i_ptr_p00, int i_ptr_p10, int i_ptr_p20,
                                 int i_ptr_p01, int i_ptr_p11, int i_ptr_p21 )
{
    four_band_lr_reg_t * p_hist = &p_ctx->history;
    for( int i_channel = 0; i_channel < p_ctx->info.i_channels; i_channel++ )
    {
        float f_x0 = p_hist->f_x[i_input_reg_idx][i_ptr_p10][i_channel];
        float f_x1 = p_hist->f_x[i_input_reg_idx][i_ptr_p00][i_channel];
        float f_x2 = p_hist->f_x[i_input_reg_idx][i_ptr_p20][i_channel];
        float f_y1 = p_hist->f_x[i_output_reg_idx][i_ptr_p01][i_channel];
        float f_y2 = p_hist->f_x[i_output_reg_idx][i_ptr_p21][i_channel];
        float f_y0 = p_coeffs->f_b0 * f_x0 + p_coeffs->f_b1 * f_x1
                                           + p_coeffs->f_b2 * f_x2
                                           - p_coeffs->f_a0 * f_y1
                                           - p_coeffs->f_a1 * f_y2;
        float f_z1 = p_hist->f_x[i_output_reg_idx][i_ptr_p00][i_channel];
        float f_z2 = p_hist->f_x[i_output_reg_idx][i_ptr_p20][i_channel];
        float f_z0 = p_coeffs->f_b0 * f_y0 + p_coeffs->f_b1 * f_y1
                                           + p_coeffs->f_b2 * f_y2
                                           - p_coeffs->f_a0 * f_z1
                                           - p_coeffs->f_a1 * f_z2;
        p_hist->f_x[i_output_reg_idx][i_ptr_p11][i_channel] = f_y0;
        p_hist->f_x[i_output_reg_idx][i_ptr_p10][i_channel] = f_z0;
    }
}

/* Processes the given input sample, assuming the filter is set to 12 dB
 * mode */
static void Process12DbMode( four_band_lr_context_t * p_ctx,
                             const float * pf_input )
{
    int i_ptr_p00 = p_ctx->i_history_ptr;
    int i_ptr_p10 = ( i_ptr_p00 + 1 ) % MAX_HISTORY_COUNT;
    int i_ptr_p20 = ( i_ptr_p00 + 2 ) % MAX_HISTORY_COUNT;
    int i_ptr_p11 = MAX_HISTORY_COUNT + i_ptr_p10;
    p_ctx->i_history_ptr = i_ptr_p10;

    StoreInput( p_ctx, pf_input, REG_IDX_INPUT, i_ptr_p10 );
    ProcessOrder1Filter( p_ctx, &p_ctx->apf[1], REG_IDX_INPUT, REG_IDX_APF_3,
                         i_ptr_p00, i_ptr_p10 );
    ProcessOrder1Filter( p_ctx, &p_ctx->apf[0], REG_IDX_INPUT, REG_IDX_APF_1,
                         i_ptr_p00, i_ptr_p10 );
    ProcessOrder2Filter( p_ctx, &p_ctx->lpf[1], REG_IDX_APF_3, REG_IDX_LPF_2,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->hpf[1], REG_IDX_APF_1, REG_IDX_HPF_2,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->lpf[0], REG_IDX_LPF_2, REG_IDX_LPF_1,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->hpf[0], REG_IDX_LPF_2, REG_IDX_HPF_1,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->lpf[2], REG_IDX_HPF_2, REG_IDX_LPF_3,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->hpf[2], REG_IDX_HPF_2, REG_IDX_HPF_3,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
}

/* Processes the given input sample, assuming the filter is set to 24 dB
 * mode */
static void Process24DbMode( four_band_lr_context_t * p_ctx,
                             const float * pf_input )
{
    int i_ptr_p00 = p_ctx->i_history_ptr;
    int i_ptr_p10 = ( i_ptr_p00 + 1 ) % MAX_HISTORY_COUNT;
    int i_ptr_p20 = ( i_ptr_p00 + 2 ) % MAX_HISTORY_COUNT;
    int i_ptr_p01 = MAX_HISTORY_COUNT + i_ptr_p00;
    int i_ptr_p11 = MAX_HISTORY_COUNT + i_ptr_p10;
    int i_ptr_p21 = MAX_HISTORY_COUNT + i_ptr_p20;
    p_ctx->i_history_ptr = i_ptr_p10;

    StoreInput( p_ctx, pf_input, REG_IDX_INPUT, i_ptr_p10 );
    ProcessOrder2Filter( p_ctx, &p_ctx->apf[1], REG_IDX_INPUT, REG_IDX_APF_3,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder2Filter( p_ctx, &p_ctx->apf[0], REG_IDX_INPUT, REG_IDX_APF_1,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20, i_ptr_p11 );
    ProcessOrder4Filter( p_ctx, &p_ctx->lpf[1], REG_IDX_APF_3, REG_IDX_LPF_2,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
    ProcessOrder4Filter( p_ctx, &p_ctx->hpf[1], REG_IDX_APF_1, REG_IDX_HPF_2,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
    ProcessOrder4Filter( p_ctx, &p_ctx->lpf[0], REG_IDX_LPF_2, REG_IDX_LPF_1,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
    ProcessOrder4Filter( p_ctx, &p_ctx->hpf[0], REG_IDX_LPF_2, REG_IDX_HPF_1,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
    ProcessOrder4Filter( p_ctx, &p_ctx->lpf[2], REG_IDX_HPF_2, REG_IDX_LPF_3,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
    ProcessOrder4Filter( p_ctx, &p_ctx->hpf[2], REG_IDX_HPF_2, REG_IDX_HPF_3,
                         i_ptr_p00, i_ptr_p10, i_ptr_p20,
                         i_ptr_p01, i_ptr_p11, i_ptr_p21 );
}

/* Initializes a four-band Linkwitz-Riley filter context.  The user specifies
 * the current sampling rate and the current number of channels in each sample.
 * The user also specifies each of the three split frequencies and a boolean
 * value (b_24dB_mode) that determines whether the filter is to be set in 24 dB
 * mode (true) or 12 dB mode (false) */
void InitFourBandLRContext( four_band_lr_context_t * p_ctx, int i_rate,
                            int i_channels, float f_split_frequency_1,
                            float f_split_frequency_2,
                            float f_split_frequency_3, bool b_24dB_mode )
{
    p_ctx->info.i_rate = i_rate;
    p_ctx->info.i_channels = i_channels;
    p_ctx->i_history_ptr = 0;
    p_ctx->p_process_func = b_24dB_mode ? Process24DbMode : Process12DbMode;
    memset( &p_ctx->history, 0, sizeof( p_ctx->history ) );
    SetSplitFrequency( p_ctx, 0, f_split_frequency_1 );
    SetSplitFrequency( p_ctx, 1, f_split_frequency_2 );
    SetSplitFrequency( p_ctx, 2, f_split_frequency_3 );
}

/* Sets the mode of the Linkwitz-Riley filter.  If b_24dB_mode is true, then
 * 24 dB mode is assumed, and LR4 filters are used; otherwise, 12 dB mode is
 * assumed, and LR2 filters are used */
void SetLRMode( four_band_lr_context_t * p_ctx, bool b_24dB_mode )
{
    p_ctx->p_process_func = b_24dB_mode ? Process24DbMode : Process12DbMode;
    ComputeFilterCoeffs( p_ctx, 0, b_24dB_mode );
    ComputeFilterCoeffs( p_ctx, 1, b_24dB_mode );
    ComputeFilterCoeffs( p_ctx, 2, b_24dB_mode );
}

/* Sets the split frequency at a specified split index */
void SetSplitFrequency( four_band_lr_context_t * p_ctx, int i_split_index,
                        float f_split_frequency )
{
    p_ctx->info.f_split_frequencies[i_split_index] = f_split_frequency;
    ComputeFilterCoeffs( p_ctx, i_split_index,
                         p_ctx->p_process_func == Process24DbMode );
}

/* Processes the given input sample */
void Process( four_band_lr_context_t * p_ctx, const float * pf_input )
{
    /* Calls Process12DbMode (if filter is set to 12 dB mode) or
     *       Process24DbMode (if filter is set to 24 dB mode) */
    p_ctx->p_process_func( p_ctx, pf_input );
}

/* Retrieves the result of processing a sample for a specified band */
const float * GetOutputData( four_band_lr_context_t * p_ctx, int i_band )
{
    int i_reg_idx = BAND_TO_REG_IDX( i_band );
    return &p_ctx->history.f_x[i_reg_idx][p_ctx->i_history_ptr][0];
}

/* Copies the result of processing a sample for a specified band into a
 * separate buffer */
void CopyOutputData( four_band_lr_context_t * p_ctx, float * pf_dest,
                     int i_band )
{
    memcpy( pf_dest, GetOutputData( p_ctx, i_band ),
            p_ctx->info.i_channels * sizeof(*pf_dest) );
}
