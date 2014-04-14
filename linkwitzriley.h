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

#ifndef LINKWITZRILEY_H
#define LINKWITZRILEY_H

/* I intend to place this code into VLC and replace this comment and the three
 * lines below it with an inclusion of vlc_filter.h.  These lines are there
 * only to make this code usable in LADSPA. */
#ifndef AOUT_CHAN_MAX
#define AOUT_CHAN_MAX 2
#endif

#include <stdbool.h>

/*
 * Filter flow chart:
 *
 *                                      +--> LPF(f1) -----> subband
 *                                      |
 *        +--> APF(f3) -----> LPF(f2) --+--> HPF(f1) -----> lowband
 * x(t) --+
 *        +--> APF(f1) -----> HPF(f2) --+--> LPF(f3) -----> midband
 *                                      |
 *                                      +--> HPF(f3) -----> highband
 *
 * The input buffer, output buffers, and intermediate results in between are
 * stored into a structured array known as the "history registry."
 */

/* Structure type for storing coefficients for a second order filter.  In this
 * implementation, two passes are used to give the effect of a fourth-order
 * filter */
typedef struct
{
    float f_a0;
    float f_a1;
    float f_b0;
    float f_b1;
    float f_b2;
}
second_order_filter_coeff_t;

/* Maximum number of samples needed to provide the filtering at any single
 * stage */
#define MAX_HISTORY_COUNT 3
#define HISTORY_SIZE (MAX_HISTORY_COUNT<<1)

/* Registry indices */
#define REG_IDX_INPUT 0
#define REG_IDX_APF_3 1
#define REG_IDX_APF_1 2
#define REG_IDX_LPF_2 3
#define REG_IDX_HPF_2 4
#define REG_IDX_LPF_1 5
#define REG_IDX_HPF_1 6
#define REG_IDX_LPF_3 7
#define REG_IDX_HPF_3 8
#define NUM_REG_INDICES 9

/* History registry */
typedef struct
{
    /* Format:  (regidx, histidx, channel) */
    float f_x[NUM_REG_INDICES][HISTORY_SIZE][AOUT_CHAN_MAX];
}
four_band_lr_reg_t;

/* Number of split frequencies for a four-band crossover */
#define NSPLITS 3

/* Number of all pass filters needed for the four-band crossover */
#define NAPFS 2

/* Information about the sampling rate, number of channels per sample, and the
 * user-defined center frequencies */
typedef struct
{
    int i_rate;
    int i_channels;
    float f_split_frequencies[NSPLITS];
}
four_band_info_t;

typedef struct four_band_lr_context_t four_band_lr_context_t;
typedef void ( * ProcessFunc )( four_band_lr_context_t *, const float * );

/* The context for the Linkwitz-Riley filter */
struct four_band_lr_context_t
{
    four_band_info_t info;      /* Audio/split frequency information */
    second_order_filter_coeff_t lpf[NSPLITS]; /* LPF(f1), LPF(f2), LPF(f3) */
    second_order_filter_coeff_t hpf[NSPLITS]; /* HPF(f1), HPF(f2), HPF(f3) */
    second_order_filter_coeff_t apf[NAPFS];   /* APF(f1), APF(f3) */
    int i_history_ptr;          /* History pointer */
    four_band_lr_reg_t history; /* History registry */
    ProcessFunc p_process_func; /* Pointer to a processing function */
};

/* Initializes a four-band Linkwitz-Riley filter context.  The user specifies
 * the current sampling rate and the current number of channels in each sample.
 * The user also specifies each of the three split frequencies and a boolean
 * value (b_24dB_mode) that determines whether the filter is to be set in 24 dB
 * mode (true) or 12 dB mode (false) */
void InitFourBandLRContext( four_band_lr_context_t * p_ctx, int i_rate,
                            int i_channels, float f_split_frequency_1,
                            float f_split_frequency_2,
                            float f_split_frequency_3, bool b_24dB_mode );

/* Sets the mode of the Linkwitz-Riley filter.  If b_24dB_mode is true, then
 * 24 dB mode is assumed, and LR4 filters are used; otherwise, 12 dB mode is
 * assumed, and LR2 filters are used */
void SetLRMode( four_band_lr_context_t * p_ctx, bool b_24dB_mode );

/* Sets the split frequency at a specified split index */
void SetSplitFrequency( four_band_lr_context_t * p_ctx, int i_split_index,
                        float f_split_frequency );

/* Processes the given input sample */
void Process( four_band_lr_context_t * p_ctx, const float * pf_input );

/* Retrieves the result of processing a sample for a specified band */
const float * GetOutputData( four_band_lr_context_t * p_ctx, int i_band );

/* Copies the result of processing a sample for a specified band into a
 * separate buffer */
void CopyOutputData( four_band_lr_context_t * p_ctx, float * pf_dest,
                     int i_band );

#endif
