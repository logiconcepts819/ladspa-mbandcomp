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
#include "ladspa.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/*** BEGIN PORTS ***/
#define MBC_SUB_RMS_PEAK     0
#define MBC_SUB_ATTACK       1
#define MBC_SUB_RELEASE      2
#define MBC_SUB_THRESHOLD    3
#define MBC_SUB_RATIO        4
#define MBC_SUB_KNEE         5
#define MBC_SUB_MAKEUP_GAIN  6
#define MBC_SUB_BYPASS       7
#define MBC_SUB_SOLO         8
#define MBC_SUB_LOW_SPLIT    9
#define MBC_LOW_RMS_PEAK     10
#define MBC_LOW_ATTACK       11
#define MBC_LOW_RELEASE      12
#define MBC_LOW_THRESHOLD    13
#define MBC_LOW_RATIO        14
#define MBC_LOW_KNEE         15
#define MBC_LOW_MAKEUP_GAIN  16
#define MBC_LOW_BYPASS       17
#define MBC_LOW_SOLO         18
#define MBC_LOW_MID_SPLIT    19
#define MBC_MID_RMS_PEAK     20
#define MBC_MID_ATTACK       21
#define MBC_MID_RELEASE      22
#define MBC_MID_THRESHOLD    23
#define MBC_MID_RATIO        24
#define MBC_MID_KNEE         25
#define MBC_MID_MAKEUP_GAIN  26
#define MBC_MID_BYPASS       27
#define MBC_MID_SOLO         28
#define MBC_MID_HIGH_SPLIT   29
#define MBC_HIGH_RMS_PEAK    30
#define MBC_HIGH_ATTACK      31
#define MBC_HIGH_RELEASE     32
#define MBC_HIGH_THRESHOLD   33
#define MBC_HIGH_RATIO       34
#define MBC_HIGH_KNEE        35
#define MBC_HIGH_MAKEUP_GAIN 36
#define MBC_HIGH_BYPASS      37
#define MBC_HIGH_SOLO        38
#define MBC_24DB_MODE        39
#define MBC_INPUT1           40
#define MBC_OUTPUT1          41
#define MBC_INPUT2           42
#define MBC_OUTPUT2          43
/*** END PORTS ***/

#define A_TBL (256)

#define DB_TABLE_SIZE   (1024)
#define DB_MIN          (-60.0f)
#define DB_MAX          (36.0f)
#define LIN_TABLE_SIZE  (1024)
#define LIN_MIN         (0.0000000002f)
#define LIN_MAX         (9.0f)
#define DB_DEFAULT_CUBE
#define RMS_BUF_SIZE    (960)
#define LOOKAHEAD_SIZE  ((RMS_BUF_SIZE)<<1)

#define LIN_INTERP(f,a,b) ((a) + (f) * ( (b) - (a) ))
#define LIMIT(v,l,u)      (v < l ? l : ( v > u ? u : v ))

typedef struct
{
    float        pf_buf[RMS_BUF_SIZE];
    unsigned int i_pos;
    unsigned int i_count;
    float        f_sum;

} rms_env;

typedef struct
{
    struct
    {
        float pf_vals[AOUT_CHAN_MAX];
        float f_lev_in;

    } p_buf[LOOKAHEAD_SIZE];
    unsigned int i_pos;
    unsigned int i_count;

} lookahead;

typedef union
{
    float f;
    int32_t i;

} ls_pcast32;

/* Compression parameters */
typedef struct
{
    float f_rms_peak;
    float f_attack;
    float f_release;
    float f_threshold;
    float f_ratio;
    float f_knee;
    float f_makeup_gain;
    bool b_bypass;       /* Unused for full band compression */
    bool b_solo;         /* Unused for full band compression */

} comp_params;

/* Internal compression parameters */
typedef struct
{
    float f_amp;
    unsigned int i_count;
    float f_env;
    float f_env_peak;
    float f_env_rms;
    float f_gain;
    float f_gain_out;
    rms_env rms;
    float f_sum;
    lookahead la;

} int_params;

typedef struct 
{
    float f_ga;
    float f_gr;
    float f_rs;
    float f_mug;
    float f_knee_min;
    float f_knee_max;
    float f_ef_a;
    float f_ef_ai;

} aux_params;

typedef struct filter_sys_t
{
    float f_sample_rate;
    float pf_as[A_TBL];

    int_params full_band_int_params;
    int_params sub_band_int_params;
    int_params low_band_int_params;
    int_params mid_band_int_params;
    int_params high_band_int_params;

    float pf_db_data[DB_TABLE_SIZE];
    float pf_lin_data[LIN_TABLE_SIZE];

    float sub_low_split;
    float low_mid_split;
    float mid_high_split;

    four_band_lr_context_t lr_context;

    bool b_24dB_mode;
    bool b_multiband;

} filter_sys_t;

/* We have a grand total of 44 ports */
#define NPORTS 44
typedef struct
{
    struct filter_sys_t sys;

    LADSPA_Data * m_pfSubRMSPeak;
    LADSPA_Data * m_pfSubAttack;
    LADSPA_Data * m_pfSubRelease;
    LADSPA_Data * m_pfSubThreshold;
    LADSPA_Data * m_pfSubRatio;
    LADSPA_Data * m_pfSubKnee;
    LADSPA_Data * m_pfSubMakeup;
    LADSPA_Data * m_pfSubBypass;
    LADSPA_Data * m_pfSubSolo;

    LADSPA_Data * m_pfSubLowSplit;

    LADSPA_Data * m_pfLowRMSPeak;
    LADSPA_Data * m_pfLowAttack;
    LADSPA_Data * m_pfLowRelease;
    LADSPA_Data * m_pfLowThreshold;
    LADSPA_Data * m_pfLowRatio;
    LADSPA_Data * m_pfLowKnee;
    LADSPA_Data * m_pfLowMakeup;
    LADSPA_Data * m_pfLowBypass;
    LADSPA_Data * m_pfLowSolo;

    LADSPA_Data * m_pfLowMidSplit;

    LADSPA_Data * m_pfMidRMSPeak;
    LADSPA_Data * m_pfMidAttack;
    LADSPA_Data * m_pfMidRelease;
    LADSPA_Data * m_pfMidThreshold;
    LADSPA_Data * m_pfMidRatio;
    LADSPA_Data * m_pfMidKnee;
    LADSPA_Data * m_pfMidMakeup;
    LADSPA_Data * m_pfMidBypass;
    LADSPA_Data * m_pfMidSolo;

    LADSPA_Data * m_pfMidHighSplit;

    LADSPA_Data * m_pfHighRMSPeak;
    LADSPA_Data * m_pfHighAttack;
    LADSPA_Data * m_pfHighRelease;
    LADSPA_Data * m_pfHighThreshold;
    LADSPA_Data * m_pfHighRatio;
    LADSPA_Data * m_pfHighKnee;
    LADSPA_Data * m_pfHighMakeup;
    LADSPA_Data * m_pfHighBypass;
    LADSPA_Data * m_pfHighSolo;

    LADSPA_Data * m_pf24dBMode;

    LADSPA_Data * m_pfInputBuffer1;
    LADSPA_Data * m_pfOutputBuffer1;
    LADSPA_Data * m_pfInputBuffer2;
    LADSPA_Data * m_pfOutputBuffer2;
}
MbandComp;

static void     DbInit            ( filter_sys_t * );
static float    Db2Lin            ( float, filter_sys_t * );
static float    Lin2Db            ( float, filter_sys_t * );
#ifdef DB_DEFAULT_CUBE
static float    CubeInterp        ( const float, const float, const float,
                                    const float, const float );
#endif
static void     RoundToZero       ( float * );
static void     ForceSmallToZero  ( float * );
static float    Max               ( float, float );
static float    Clamp             ( float, float, float );
static int      Round             ( float );
static float    RmsEnvProcess     ( rms_env *, const float );
static void     BufferProcess     ( float *, int, float, float, lookahead * );

static void CalculateAuxiliaryParameters( filter_sys_t *, comp_params *,
                                          aux_params  * );

static void ApplyCompression      ( filter_sys_t *, comp_params *, int_params *,
                                    aux_params *, float *, int );

static float getLowValue( float min, float max, bool b_logarithmic )
{
    return b_logarithmic ? expf( logf( min ) * 0.75 + logf( max ) * 0.25 )
                         : ( min * 0.75 + max * 0.25 );
}

static float getMidValue( float min, float max, bool b_logarithmic )
{
    return b_logarithmic ? expf( logf( min ) * 0.5 + logf( max ) * 0.5 )
                         : ( min * 0.5 + max * 0.5 );
}

static float getHighValue( float min, float max, bool b_logarithmic )
{
    return b_logarithmic ? expf( logf( min ) * 0.25 + logf( max ) * 0.75 )
                         : ( min * 0.25 + max * 0.75 );
}

static LADSPA_Handle
instantiateMbandComp( const LADSPA_Descriptor * Descriptor,
                      unsigned long SampleRate )
{
    MbandComp * pFilter;
    (void) Descriptor;
    pFilter = (MbandComp *) calloc( 1, sizeof(MbandComp) );
    if( pFilter )
    {
        float f_num;
        filter_sys_t * p_sys = &pFilter->sys;
        p_sys->f_sample_rate = (float) SampleRate;
        p_sys->b_24dB_mode = false;
        p_sys->sub_low_split = getLowValue( 10.0f, 20000.0f, true );
        p_sys->low_mid_split = getMidValue( 10.0f, 20000.0f, true );
        p_sys->mid_high_split = getHighValue( 10.0f, 20000.0f, true );
        InitFourBandLRContext( &p_sys->lr_context, (int) SampleRate, 2,
                               p_sys->sub_low_split, p_sys->low_mid_split,
                               p_sys->mid_high_split, false );
        p_sys->pf_as[0] = 1.0f;

        /* Initialize the attack lookup table */
        for( int i = 1; i < A_TBL; i++ )
        {
            p_sys->pf_as[i] =
                expf( -1.0f / ( p_sys->f_sample_rate * i / A_TBL ) );
        }

        /* Calculate the RMS and lookahead sizes from the sample rate */
        f_num = 0.01f * p_sys->f_sample_rate;

        p_sys->full_band_int_params.rms.i_count =
        p_sys->sub_band_int_params.rms.i_count  =
        p_sys->low_band_int_params.rms.i_count  =
        p_sys->mid_band_int_params.rms.i_count  =
        p_sys->high_band_int_params.rms.i_count =
                Round( Clamp( 0.5f * f_num, 1.0f, RMS_BUF_SIZE ) );

        p_sys->full_band_int_params.la.i_count =
        p_sys->sub_band_int_params.la.i_count  =
        p_sys->low_band_int_params.la.i_count  =
        p_sys->mid_band_int_params.la.i_count  =
        p_sys->high_band_int_params.la.i_count =
                Round( Clamp( f_num, 1.0f, LOOKAHEAD_SIZE ) );

        /* Initialize decibel lookup tables */
        DbInit( p_sys );

    }
    return pFilter;
}

static void connectPortToMbandComp( LADSPA_Handle Instance, unsigned long Port,
                                    LADSPA_Data * DataLocation )
{
    MbandComp * pFilter;
    pFilter = (MbandComp *) Instance;
    switch( Port )
    {
    case MBC_SUB_RMS_PEAK:
        pFilter->m_pfSubRMSPeak = DataLocation;
        break;
    case MBC_SUB_ATTACK:
        pFilter->m_pfSubAttack = DataLocation;
        break;
    case MBC_SUB_RELEASE:
        pFilter->m_pfSubRelease = DataLocation;
        break;
    case MBC_SUB_THRESHOLD:
        pFilter->m_pfSubThreshold = DataLocation;
        break;
    case MBC_SUB_RATIO:
        pFilter->m_pfSubRatio = DataLocation;
        break;
    case MBC_SUB_KNEE:
        pFilter->m_pfSubKnee = DataLocation;
        break;
    case MBC_SUB_MAKEUP_GAIN:
        pFilter->m_pfSubMakeup = DataLocation;
        break;
    case MBC_SUB_BYPASS:
        pFilter->m_pfSubBypass = DataLocation;
        break;
    case MBC_SUB_SOLO:
        pFilter->m_pfSubSolo = DataLocation;
        break;
    case MBC_SUB_LOW_SPLIT:
        pFilter->m_pfSubLowSplit = DataLocation;
        break;
    case MBC_LOW_RMS_PEAK:
        pFilter->m_pfLowRMSPeak = DataLocation;
        break;
    case MBC_LOW_ATTACK:
        pFilter->m_pfLowAttack = DataLocation;
        break;
    case MBC_LOW_RELEASE:
        pFilter->m_pfLowRelease = DataLocation;
        break;
    case MBC_LOW_THRESHOLD:
        pFilter->m_pfLowThreshold = DataLocation;
        break;
    case MBC_LOW_RATIO:
        pFilter->m_pfLowRatio = DataLocation;
        break;
    case MBC_LOW_KNEE:
        pFilter->m_pfLowKnee = DataLocation;
        break;
    case MBC_LOW_MAKEUP_GAIN:
        pFilter->m_pfLowMakeup = DataLocation;
        break;
    case MBC_LOW_BYPASS:
        pFilter->m_pfLowBypass = DataLocation;
        break;
    case MBC_LOW_SOLO:
        pFilter->m_pfLowSolo = DataLocation;
        break;
    case MBC_LOW_MID_SPLIT:
        pFilter->m_pfLowMidSplit = DataLocation;
        break;
    case MBC_MID_RMS_PEAK:
        pFilter->m_pfMidRMSPeak = DataLocation;
        break;
    case MBC_MID_ATTACK:
        pFilter->m_pfMidAttack = DataLocation;
        break;
    case MBC_MID_RELEASE:
        pFilter->m_pfMidRelease = DataLocation;
        break;
    case MBC_MID_THRESHOLD:
        pFilter->m_pfMidThreshold = DataLocation;
        break;
    case MBC_MID_RATIO:
        pFilter->m_pfMidRatio = DataLocation;
        break;
    case MBC_MID_KNEE:
        pFilter->m_pfMidKnee = DataLocation;
        break;
    case MBC_MID_MAKEUP_GAIN:
        pFilter->m_pfMidMakeup = DataLocation;
        break;
    case MBC_MID_BYPASS:
        pFilter->m_pfMidBypass = DataLocation;
        break;
    case MBC_MID_SOLO:
        pFilter->m_pfMidSolo = DataLocation;
        break;
    case MBC_MID_HIGH_SPLIT:
        pFilter->m_pfMidHighSplit = DataLocation;
        break;
    case MBC_HIGH_RMS_PEAK:
        pFilter->m_pfHighRMSPeak = DataLocation;
        break;
    case MBC_HIGH_ATTACK:
        pFilter->m_pfHighAttack = DataLocation;
        break;
    case MBC_HIGH_RELEASE:
        pFilter->m_pfHighRelease = DataLocation;
        break;
    case MBC_HIGH_THRESHOLD:
        pFilter->m_pfHighThreshold = DataLocation;
        break;
    case MBC_HIGH_RATIO:
        pFilter->m_pfHighRatio = DataLocation;
        break;
    case MBC_HIGH_KNEE:
        pFilter->m_pfHighKnee = DataLocation;
        break;
    case MBC_HIGH_MAKEUP_GAIN:
        pFilter->m_pfHighMakeup = DataLocation;
        break;
    case MBC_HIGH_BYPASS:
        pFilter->m_pfHighBypass = DataLocation;
        break;
    case MBC_HIGH_SOLO:
        pFilter->m_pfHighSolo = DataLocation;
        break;
    case MBC_24DB_MODE:
        pFilter->m_pf24dBMode = DataLocation;
        break;
    case MBC_INPUT1:
        pFilter->m_pfInputBuffer1 = DataLocation;
        break;
    case MBC_OUTPUT1:
        pFilter->m_pfOutputBuffer1 = DataLocation;
        break;
    case MBC_INPUT2:
        pFilter->m_pfInputBuffer2 = DataLocation;
        break;
    case MBC_OUTPUT2:
        pFilter->m_pfOutputBuffer2 = DataLocation;
        break;
    }
}

static void runMbandComp( LADSPA_Handle Instance, unsigned long SampleCount )
{
    MbandComp * pComp = (MbandComp *) Instance;
    filter_sys_t * p_sys = &pComp->sys;
    LADSPA_Data * pfInput1 = pComp->m_pfInputBuffer1;
    LADSPA_Data * pfInput2 = pComp->m_pfInputBuffer2;
    LADSPA_Data * pfOutput1 = pComp->m_pfOutputBuffer1;
    LADSPA_Data * pfOutput2 = pComp->m_pfOutputBuffer2;
    comp_params sub_band_params;
    comp_params low_band_params;
    comp_params mid_band_params;
    comp_params high_band_params;
    aux_params sub_band_aux_params;
    aux_params low_band_aux_params;
    aux_params mid_band_aux_params;
    aux_params high_band_aux_params;
    float sub_low_split;
    float low_mid_split;
    float mid_high_split;
    bool b_24dB_mode, b_no_solo;
    bool b_process_sub_band, b_process_low_band;
    bool b_process_mid_band, b_process_high_band;

    sub_band_params.f_rms_peak = *( pComp->m_pfSubRMSPeak );
    sub_band_params.f_attack = *( pComp->m_pfSubAttack );
    sub_band_params.f_release = *( pComp->m_pfSubRelease );
    sub_band_params.f_threshold = *( pComp->m_pfSubThreshold );
    sub_band_params.f_ratio = *( pComp->m_pfSubRatio );
    sub_band_params.f_knee = *( pComp->m_pfSubKnee );
    sub_band_params.f_makeup_gain = *( pComp->m_pfSubMakeup );
    sub_band_params.b_bypass = (bool) *( pComp->m_pfSubBypass );
    sub_band_params.b_solo = (bool) *( pComp->m_pfSubSolo );

    sub_low_split = *( pComp->m_pfSubLowSplit );
    if( sub_low_split != p_sys->sub_low_split )
    {
        p_sys->sub_low_split = sub_low_split;
        SetSplitFrequency( &p_sys->lr_context, 0, sub_low_split );
    }

    low_band_params.f_rms_peak = *( pComp->m_pfLowRMSPeak );
    low_band_params.f_attack = *( pComp->m_pfLowAttack );
    low_band_params.f_release = *( pComp->m_pfLowRelease );
    low_band_params.f_threshold = *( pComp->m_pfLowThreshold );
    low_band_params.f_ratio = *( pComp->m_pfLowRatio );
    low_band_params.f_knee = *( pComp->m_pfLowKnee );
    low_band_params.f_makeup_gain = *( pComp->m_pfLowMakeup );
    low_band_params.b_bypass = (int) *( pComp->m_pfLowBypass );
    low_band_params.b_solo = (int) *( pComp->m_pfLowSolo );

    low_mid_split = *( pComp->m_pfLowMidSplit );
    if( low_mid_split != p_sys->low_mid_split )
    {
        p_sys->low_mid_split = low_mid_split;
        SetSplitFrequency( &p_sys->lr_context, 1, low_mid_split );
    }

    mid_band_params.f_rms_peak = *( pComp->m_pfMidRMSPeak );
    mid_band_params.f_attack = *( pComp->m_pfMidAttack );
    mid_band_params.f_release = *( pComp->m_pfMidRelease );
    mid_band_params.f_threshold = *( pComp->m_pfMidThreshold );
    mid_band_params.f_ratio = *( pComp->m_pfMidRatio );
    mid_band_params.f_knee = *( pComp->m_pfMidKnee );
    mid_band_params.f_makeup_gain = *( pComp->m_pfMidMakeup );
    mid_band_params.b_bypass = (int) *( pComp->m_pfMidBypass );
    mid_band_params.b_solo = (int) *( pComp->m_pfMidSolo );

    mid_high_split = *( pComp->m_pfMidHighSplit );
    if( mid_high_split != p_sys->mid_high_split )
    {
        p_sys->mid_high_split = mid_high_split;
        SetSplitFrequency( &p_sys->lr_context, 2, mid_high_split );
    }

    high_band_params.f_rms_peak = *( pComp->m_pfHighRMSPeak );
    high_band_params.f_attack = *( pComp->m_pfHighAttack );
    high_band_params.f_release = *( pComp->m_pfHighRelease );
    high_band_params.f_threshold = *( pComp->m_pfHighThreshold );
    high_band_params.f_ratio = *( pComp->m_pfHighRatio );
    high_band_params.f_knee = *( pComp->m_pfHighKnee );
    high_band_params.f_makeup_gain = *( pComp->m_pfHighMakeup );
    high_band_params.b_bypass = (int) *( pComp->m_pfHighBypass );
    high_band_params.b_solo = (int) *( pComp->m_pfHighSolo );

    b_24dB_mode = (int) *( pComp->m_pf24dBMode );
    if( b_24dB_mode != p_sys->b_24dB_mode )
    {
        p_sys->b_24dB_mode = b_24dB_mode;
        SetLRMode( &p_sys->lr_context, b_24dB_mode );
    }

    /* Nothing is solo if no band is solo */
    b_no_solo = !sub_band_params.b_solo && !low_band_params.b_solo
                                        && !mid_band_params.b_solo
                                        && !high_band_params.b_solo;

    /* Process the band if it is solo or none of the bands are solo */
    b_process_sub_band = sub_band_params.b_solo || b_no_solo;
    b_process_low_band = low_band_params.b_solo || b_no_solo;
    b_process_mid_band = mid_band_params.b_solo || b_no_solo;
    b_process_high_band = high_band_params.b_solo || b_no_solo;

    /* Calculate auxiliary parameters */
    CalculateAuxiliaryParameters( p_sys, &sub_band_params,
                                         &sub_band_aux_params );
    CalculateAuxiliaryParameters( p_sys, &low_band_params,
                                         &low_band_aux_params );
    CalculateAuxiliaryParameters( p_sys, &mid_band_params,
                                         &mid_band_aux_params );
    CalculateAuxiliaryParameters( p_sys, &high_band_params,
                                         &high_band_aux_params );

    for( int i = 0; i < (int) SampleCount; i++ )
    {
        float buf_array[6][2];
        memset( &buf_array[0][0], 0, sizeof( buf_array ) );
        buf_array[0][0] = pfInput1[i];
        buf_array[0][1] = pfInput2[i];

        Process( &p_sys->lr_context, &buf_array[0][0] );

        /* Compress each sample */
        if( b_process_sub_band )
        {
            CopyOutputData( &p_sys->lr_context, &buf_array[1][0], 0 );
            if( !sub_band_params.b_bypass )
            {
                ApplyCompression(  p_sys, &sub_band_params,
                                  &p_sys->sub_band_int_params,
                                  &sub_band_aux_params,
                                  &buf_array[1][0], 2 );
            }
        }
        if( b_process_low_band )
        {
            CopyOutputData( &p_sys->lr_context, &buf_array[2][0], 1 );
            if( !low_band_params.b_bypass )
            {
                ApplyCompression(  p_sys, &low_band_params,
                                  &p_sys->low_band_int_params,
                                  &low_band_aux_params,
                                  &buf_array[2][0], 2 );
            }
        }
        if( b_process_mid_band )
        {
            CopyOutputData( &p_sys->lr_context, &buf_array[3][0], 2 );
            if( !mid_band_params.b_bypass )
            {
                ApplyCompression(  p_sys, &mid_band_params,
                                  &p_sys->mid_band_int_params,
                                  &mid_band_aux_params,
                                  &buf_array[3][0], 2 );
            }
        }
        if( b_process_high_band )
        {
            CopyOutputData( &p_sys->lr_context, &buf_array[4][0], 3 );
            if( !high_band_params.b_bypass )
            {
                ApplyCompression(  p_sys, &high_band_params,
                                  &p_sys->high_band_int_params,
                                  &high_band_aux_params,
                                  &buf_array[4][0], 2 );
            }
        }

        /* Finally, recombine the samples into one */
        for( int j = 0; j < 2; j++ )
        {
            buf_array[5][j] = buf_array[1][j] + buf_array[2][j]
                                              + buf_array[3][j]
                                              + buf_array[4][j];
        }
        pfOutput1[i] = buf_array[5][0];
        pfOutput2[i] = buf_array[5][1];
    }
}

static void cleanupMbandComp( LADSPA_Handle Instance )
{
    free( Instance );
}

static LADSPA_Descriptor * g_psDescriptor = NULL;

static void my_init( void )
{
    char ** pcPortNames;
    LADSPA_PortDescriptor * piPortDescriptors;
    LADSPA_PortRangeHint * psPortRangeHints;

    g_psDescriptor = (LADSPA_Descriptor *) malloc( sizeof(LADSPA_Descriptor) );
    if( g_psDescriptor )
    {
        g_psDescriptor->UniqueID = 2501;
        g_psDescriptor->Label = strdup( "mband_comp" );
        g_psDescriptor->Properties = LADSPA_PROPERTY_HARD_RT_CAPABLE;
        g_psDescriptor->Name = strdup( "Ron's Multiband Compressor" );
        g_psDescriptor->Maker = strdup( "Ron Wright" );
        g_psDescriptor->Copyright = strdup( "LGPL" );
        g_psDescriptor->PortCount = NPORTS;
        piPortDescriptors =
          (LADSPA_PortDescriptor *) calloc( NPORTS,
                                            sizeof(LADSPA_PortDescriptor) );
        g_psDescriptor->PortDescriptors =
          (const LADSPA_PortDescriptor *) piPortDescriptors;
        piPortDescriptors[MBC_SUB_RMS_PEAK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_ATTACK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_RELEASE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_THRESHOLD] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_RATIO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_KNEE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_MAKEUP_GAIN] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_BYPASS] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_SOLO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_SUB_LOW_SPLIT] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_RMS_PEAK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_ATTACK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_RELEASE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_THRESHOLD] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_RATIO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_KNEE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_MAKEUP_GAIN] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_BYPASS] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_SOLO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_LOW_MID_SPLIT] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_RMS_PEAK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_ATTACK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_RELEASE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_THRESHOLD] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_RATIO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_KNEE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_MAKEUP_GAIN] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_BYPASS] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_SOLO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_MID_HIGH_SPLIT] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_RMS_PEAK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_ATTACK] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_RELEASE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_THRESHOLD] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_RATIO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_KNEE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_MAKEUP_GAIN] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_BYPASS] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_HIGH_SOLO] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_24DB_MODE] =
          LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL;
        piPortDescriptors[MBC_INPUT1] =
          LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
        piPortDescriptors[MBC_OUTPUT1] =
          LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
        piPortDescriptors[MBC_INPUT2] =
          LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO;
        piPortDescriptors[MBC_OUTPUT2] =
          LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO;
        pcPortNames = (char **) calloc( NPORTS, sizeof(char *) );
        g_psDescriptor->PortNames = (const char **) pcPortNames;
        pcPortNames[MBC_SUB_RMS_PEAK] = strdup( "Sub RMS peak" );
        pcPortNames[MBC_SUB_ATTACK] = strdup( "Sub attack (ms)" );
        pcPortNames[MBC_SUB_RELEASE] = strdup( "Sub release (ms)" );
        pcPortNames[MBC_SUB_THRESHOLD] = strdup( "Sub threshold (dB)" );
        pcPortNames[MBC_SUB_RATIO] = strdup( "Sub ratio (n:1)" );
        pcPortNames[MBC_SUB_KNEE] = strdup( "Sub knee radius (dB)" );
        pcPortNames[MBC_SUB_MAKEUP_GAIN] = strdup( "Sub makeup gain (dB)" );
        pcPortNames[MBC_SUB_BYPASS] = strdup( "Sub bypass" );
        pcPortNames[MBC_SUB_SOLO] = strdup( "Sub solo" );
        pcPortNames[MBC_SUB_LOW_SPLIT] =
            strdup( "Sub/low split frequency (Hz)" );
        pcPortNames[MBC_LOW_RMS_PEAK] = strdup( "Low RMS peak" );
        pcPortNames[MBC_LOW_ATTACK] = strdup( "Low attack (ms)" );
        pcPortNames[MBC_LOW_RELEASE] = strdup( "Low release (ms)" );
        pcPortNames[MBC_LOW_THRESHOLD] = strdup( "Low threshold (dB)" );
        pcPortNames[MBC_LOW_RATIO] = strdup( "Low ratio (n:1)" );
        pcPortNames[MBC_LOW_KNEE] = strdup( "Low knee radius (dB)" );
        pcPortNames[MBC_LOW_MAKEUP_GAIN] = strdup( "Low makeup gain (dB)" );
        pcPortNames[MBC_LOW_BYPASS] = strdup( "Low bypass" );
        pcPortNames[MBC_LOW_SOLO] = strdup( "Low solo" );
        pcPortNames[MBC_LOW_MID_SPLIT] =
            strdup( "Low/mid split frequency (Hz)" );
        pcPortNames[MBC_MID_RMS_PEAK] = strdup( "Mid RMS peak" );
        pcPortNames[MBC_MID_ATTACK] = strdup( "Mid attack (ms)" );
        pcPortNames[MBC_MID_RELEASE] = strdup( "Mid release (ms)" );
        pcPortNames[MBC_MID_THRESHOLD] = strdup( "Mid threshold (dB)" );
        pcPortNames[MBC_MID_RATIO] = strdup( "Mid ratio (n:1)" );
        pcPortNames[MBC_MID_KNEE] = strdup( "Mid knee radius (dB)" );
        pcPortNames[MBC_MID_MAKEUP_GAIN] = strdup( "Mid makeup gain (dB)" );
        pcPortNames[MBC_MID_BYPASS] = strdup( "Mid bypass" );
        pcPortNames[MBC_MID_SOLO] = strdup( "Mid solo" );
        pcPortNames[MBC_MID_HIGH_SPLIT] =
            strdup( "Mid/high split frequency (Hz)" );
        pcPortNames[MBC_HIGH_RMS_PEAK] = strdup( "High RMS peak" );
        pcPortNames[MBC_HIGH_ATTACK] = strdup( "High attack (ms)" );
        pcPortNames[MBC_HIGH_RELEASE] = strdup( "High release (ms)" );
        pcPortNames[MBC_HIGH_THRESHOLD] = strdup( "High threshold (dB)" );
        pcPortNames[MBC_HIGH_RATIO] = strdup( "High ratio (n:1)" );
        pcPortNames[MBC_HIGH_KNEE] = strdup( "High knee radius (dB)" );
        pcPortNames[MBC_HIGH_MAKEUP_GAIN] = strdup( "High makeup gain (dB)" );
        pcPortNames[MBC_HIGH_BYPASS] = strdup( "High bypass" );
        pcPortNames[MBC_HIGH_SOLO] = strdup( "High solo" );
        pcPortNames[MBC_24DB_MODE] = strdup( "24 dB mode" );
        pcPortNames[MBC_INPUT1] = strdup( "Input (Left)" );
        pcPortNames[MBC_OUTPUT1] = strdup( "Output (Left)" );
        pcPortNames[MBC_INPUT2] = strdup( "Input (Right)" );
        pcPortNames[MBC_OUTPUT2] = strdup( "Output (Right)" );
        psPortRangeHints =
          (LADSPA_PortRangeHint *) calloc( NPORTS,
                                           sizeof(LADSPA_PortRangeHint) );
        g_psDescriptor->PortRangeHints =
          (const LADSPA_PortRangeHint *) psPortRangeHints;
        psPortRangeHints[MBC_SUB_RMS_PEAK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_SUB_RMS_PEAK].LowerBound = 0.0;
        psPortRangeHints[MBC_SUB_RMS_PEAK].UpperBound = 1.0;
        psPortRangeHints[MBC_SUB_ATTACK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_SUB_ATTACK].LowerBound = 1.5;
        psPortRangeHints[MBC_SUB_ATTACK].UpperBound = 400.0;
        psPortRangeHints[MBC_SUB_RELEASE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_SUB_RELEASE].LowerBound = 2.0;
        psPortRangeHints[MBC_SUB_RELEASE].UpperBound = 800.0;
        psPortRangeHints[MBC_SUB_THRESHOLD].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_SUB_THRESHOLD].LowerBound = -60.0;
        psPortRangeHints[MBC_SUB_THRESHOLD].UpperBound = 0.0;
        psPortRangeHints[MBC_SUB_RATIO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_SUB_RATIO].LowerBound = 1.0;
        psPortRangeHints[MBC_SUB_RATIO].UpperBound = 20.0;
        psPortRangeHints[MBC_SUB_KNEE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_SUB_KNEE].LowerBound = 1.0;
        psPortRangeHints[MBC_SUB_KNEE].UpperBound = 10.0;
        psPortRangeHints[MBC_SUB_MAKEUP_GAIN].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_SUB_MAKEUP_GAIN].LowerBound = 0.0;
        psPortRangeHints[MBC_SUB_MAKEUP_GAIN].UpperBound = 36.0;
        psPortRangeHints[MBC_SUB_BYPASS].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_SUB_BYPASS].LowerBound = 0.0;
        psPortRangeHints[MBC_SUB_BYPASS].UpperBound = 1.0;
        psPortRangeHints[MBC_SUB_SOLO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_SUB_SOLO].LowerBound = 0.0;
        psPortRangeHints[MBC_SUB_SOLO].UpperBound = 1.0;
        psPortRangeHints[MBC_SUB_LOW_SPLIT].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_LOGARITHMIC
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_SUB_LOW_SPLIT].LowerBound = 10;
        psPortRangeHints[MBC_SUB_LOW_SPLIT].UpperBound = 20000;
        psPortRangeHints[MBC_LOW_RMS_PEAK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_LOW_RMS_PEAK].LowerBound = 0.0;
        psPortRangeHints[MBC_LOW_RMS_PEAK].UpperBound = 1.0;
        psPortRangeHints[MBC_LOW_ATTACK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_LOW_ATTACK].LowerBound = 1.5;
        psPortRangeHints[MBC_LOW_ATTACK].UpperBound = 400.0;
        psPortRangeHints[MBC_LOW_RELEASE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_LOW_RELEASE].LowerBound = 2.0;
        psPortRangeHints[MBC_LOW_RELEASE].UpperBound = 800.0;
        psPortRangeHints[MBC_LOW_THRESHOLD].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_LOW_THRESHOLD].LowerBound = -60.0;
        psPortRangeHints[MBC_LOW_THRESHOLD].UpperBound = 0.0;
        psPortRangeHints[MBC_LOW_RATIO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_LOW_RATIO].LowerBound = 1.0;
        psPortRangeHints[MBC_LOW_RATIO].UpperBound = 20.0;
        psPortRangeHints[MBC_LOW_KNEE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_LOW_KNEE].LowerBound = 1.0;
        psPortRangeHints[MBC_LOW_KNEE].UpperBound = 10.0;
        psPortRangeHints[MBC_LOW_MAKEUP_GAIN].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_LOW_MAKEUP_GAIN].LowerBound = 0.0;
        psPortRangeHints[MBC_LOW_MAKEUP_GAIN].UpperBound = 36.0;
        psPortRangeHints[MBC_LOW_BYPASS].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_LOW_BYPASS].LowerBound = 0.0;
        psPortRangeHints[MBC_LOW_BYPASS].UpperBound = 1.0;
        psPortRangeHints[MBC_LOW_SOLO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_LOW_SOLO].LowerBound = 0.0;
        psPortRangeHints[MBC_LOW_SOLO].UpperBound = 1.0;
        psPortRangeHints[MBC_LOW_MID_SPLIT].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_LOGARITHMIC
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_LOW_MID_SPLIT].LowerBound = 10;
        psPortRangeHints[MBC_LOW_MID_SPLIT].UpperBound = 20000;
        psPortRangeHints[MBC_MID_RMS_PEAK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_MID_RMS_PEAK].LowerBound = 0.0;
        psPortRangeHints[MBC_MID_RMS_PEAK].UpperBound = 1.0;
        psPortRangeHints[MBC_MID_ATTACK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_MID_ATTACK].LowerBound = 1.5;
        psPortRangeHints[MBC_MID_ATTACK].UpperBound = 400.0;
        psPortRangeHints[MBC_MID_RELEASE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_MID_RELEASE].LowerBound = 2.0;
        psPortRangeHints[MBC_MID_RELEASE].UpperBound = 800.0;
        psPortRangeHints[MBC_MID_THRESHOLD].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_MID_THRESHOLD].LowerBound = -60.0;
        psPortRangeHints[MBC_MID_THRESHOLD].UpperBound = 0.0;
        psPortRangeHints[MBC_MID_RATIO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_MID_RATIO].LowerBound = 1.0;
        psPortRangeHints[MBC_MID_RATIO].UpperBound = 20.0;
        psPortRangeHints[MBC_MID_KNEE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_MID_KNEE].LowerBound = 1.0;
        psPortRangeHints[MBC_MID_KNEE].UpperBound = 10.0;
        psPortRangeHints[MBC_MID_MAKEUP_GAIN].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_MID_MAKEUP_GAIN].LowerBound = 0.0;
        psPortRangeHints[MBC_MID_MAKEUP_GAIN].UpperBound = 36.0;
        psPortRangeHints[MBC_MID_BYPASS].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_MID_BYPASS].LowerBound = 0.0;
        psPortRangeHints[MBC_MID_BYPASS].UpperBound = 1.0;
        psPortRangeHints[MBC_MID_SOLO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_MID_SOLO].LowerBound = 0.0;
        psPortRangeHints[MBC_MID_SOLO].UpperBound = 1.0;
        psPortRangeHints[MBC_MID_HIGH_SPLIT].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_LOGARITHMIC
                                    | LADSPA_HINT_DEFAULT_HIGH;
        psPortRangeHints[MBC_MID_HIGH_SPLIT].LowerBound = 10;
        psPortRangeHints[MBC_MID_HIGH_SPLIT].UpperBound = 20000;
        psPortRangeHints[MBC_HIGH_RMS_PEAK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_HIGH_RMS_PEAK].LowerBound = 0.0;
        psPortRangeHints[MBC_HIGH_RMS_PEAK].UpperBound = 1.0;
        psPortRangeHints[MBC_HIGH_ATTACK].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_HIGH_ATTACK].LowerBound = 1.5;
        psPortRangeHints[MBC_HIGH_ATTACK].UpperBound = 400.0;
        psPortRangeHints[MBC_HIGH_RELEASE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_HIGH_RELEASE].LowerBound = 2.0;
        psPortRangeHints[MBC_HIGH_RELEASE].UpperBound = 800.0;
        psPortRangeHints[MBC_HIGH_THRESHOLD].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_HIGH_THRESHOLD].LowerBound = -60.0;
        psPortRangeHints[MBC_HIGH_THRESHOLD].UpperBound = 0.0;
        psPortRangeHints[MBC_HIGH_RATIO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_MIDDLE;
        psPortRangeHints[MBC_HIGH_RATIO].LowerBound = 1.0;
        psPortRangeHints[MBC_HIGH_RATIO].UpperBound = 20.0;
        psPortRangeHints[MBC_HIGH_KNEE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_HIGH_KNEE].LowerBound = 1.0;
        psPortRangeHints[MBC_HIGH_KNEE].UpperBound = 10.0;
        psPortRangeHints[MBC_HIGH_MAKEUP_GAIN].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_DEFAULT_LOW;
        psPortRangeHints[MBC_HIGH_MAKEUP_GAIN].LowerBound = 0.0;
        psPortRangeHints[MBC_HIGH_MAKEUP_GAIN].UpperBound = 36.0;
        psPortRangeHints[MBC_HIGH_BYPASS].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_HIGH_BYPASS].LowerBound = 0.0;
        psPortRangeHints[MBC_HIGH_BYPASS].UpperBound = 1.0;
        psPortRangeHints[MBC_HIGH_SOLO].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_HIGH_SOLO].LowerBound = 0.0;
        psPortRangeHints[MBC_HIGH_SOLO].UpperBound = 1.0;
        psPortRangeHints[MBC_24DB_MODE].HintDescriptor =
          LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE
                                    | LADSPA_HINT_INTEGER
                                    | LADSPA_HINT_DEFAULT_0;
        psPortRangeHints[MBC_24DB_MODE].LowerBound = 0.0;
        psPortRangeHints[MBC_24DB_MODE].UpperBound = 1.0;
        psPortRangeHints[MBC_INPUT1].HintDescriptor = 0;
        psPortRangeHints[MBC_OUTPUT1].HintDescriptor = 0;
        psPortRangeHints[MBC_INPUT2].HintDescriptor = 0;
        psPortRangeHints[MBC_OUTPUT2].HintDescriptor = 0;
        g_psDescriptor->instantiate = instantiateMbandComp;
        g_psDescriptor->connect_port = connectPortToMbandComp;
        g_psDescriptor->activate = NULL;
        g_psDescriptor->run = runMbandComp;
        g_psDescriptor->run_adding = NULL;
        g_psDescriptor->set_run_adding_gain = NULL;
        g_psDescriptor->deactivate = NULL;
        g_psDescriptor->cleanup = cleanupMbandComp;
    }
}

static void deleteDescriptor( LADSPA_Descriptor * psDescriptor )
{
    unsigned long lIndex;
    if( psDescriptor )
    {
        free( (char *) psDescriptor->Label );
        free( (char *) psDescriptor->Name );
        free( (char *) psDescriptor->Maker );
        free( (char *) psDescriptor->Copyright );
        free( (LADSPA_Descriptor *) psDescriptor->PortDescriptors );
        for( lIndex = 0; lIndex < psDescriptor->PortCount; lIndex++ )
        {
            free( (char *) psDescriptor->PortNames[lIndex] );
        }
        free( (char **) psDescriptor->PortNames);
        free( (LADSPA_PortRangeHint *) psDescriptor->PortRangeHints );
        free( psDescriptor );
    }
}

static void my_fini( void )
{
    deleteDescriptor( g_psDescriptor );
}

LADSPA_SYMBOL_EXPORT
const LADSPA_Descriptor * ladspa_descriptor( unsigned long Index )
{
    if( Index == 0 )
    {
        return g_psDescriptor;
    }
    return NULL;
}

#ifdef WIN32
#include <windows.h>
BOOL APIENTRY DllMain( HANDLE hModule, DWORD ul_reason_for_call,
                       LPVOID lpReserved )
{
    (void) hModule, (void) lpReserved;
    switch( ul_reason_for_call )
    {
    case DLL_PROCESS_ATTACH:
        my_init();
        break;
    case DLL_THREAD_ATTACH:
        break;
    case DLL_THREAD_DETACH:
        break;
    case DLL_PROCESS_DETACH:
        my_fini();
        break;
    }
    return TRUE;
}
#else
void __attribute__((constructor)) init( void )
{
    my_init();
}
void __attribute__((destructor)) fini( void )
{
    my_fini();
}
#endif

static void CalculateAuxiliaryParameters( filter_sys_t * p_sys,
                                          comp_params * p_cparams,
                                          aux_params  * p_aparams )
{
    p_aparams->f_ga = p_cparams->f_attack < 2.0f ? 0.0f :
                      p_sys->pf_as[Round( p_cparams->f_attack  * 0.001f * ( A_TBL - 1 ) )];
    p_aparams->f_gr = p_sys->pf_as[Round( p_cparams->f_release * 0.001f * ( A_TBL - 1 ) )];
    p_aparams->f_rs = ( p_cparams->f_ratio - 1.0f ) / p_cparams->f_ratio;
    p_aparams->f_mug      = Db2Lin( p_cparams->f_makeup_gain, p_sys );
    p_aparams->f_knee_min = Db2Lin( p_cparams->f_threshold - p_cparams->f_knee, p_sys );
    p_aparams->f_knee_max = Db2Lin( p_cparams->f_threshold + p_cparams->f_knee, p_sys );
    p_aparams->f_ef_a  = p_aparams->f_ga * 0.25f;
    p_aparams->f_ef_ai = 1.0f - p_aparams->f_ef_a;
}

/* Do the actual compression */
static void ApplyCompression( filter_sys_t * p_sys, comp_params * p_cparams,
                              int_params * p_iparams, aux_params * p_aparams,
                              float * pf_buf, int i_channels )
{
    float f_lev_in_old, f_lev_in_new;

    /* Compress the audio (ported from sc4_1882 plugin with a few
     * modifications) */

    /* Fetch the old delayed buffer value */
    f_lev_in_old = p_iparams->la.p_buf[p_iparams->la.i_pos].f_lev_in;

    /* Find the peak value of current sample.  This becomes the new delayed
     * buffer value that replaces the old one in the lookahead array */
    f_lev_in_new = fabs( pf_buf[0] );
    for( int i_chan = 1; i_chan < i_channels; i_chan++ )
    {
        f_lev_in_new = Max( f_lev_in_new, fabs( pf_buf[i_chan] ) );
    }
    p_iparams->la.p_buf[p_iparams->la.i_pos].f_lev_in = f_lev_in_new;

    /* Add the square of the peak value to a running sum */
    p_iparams->f_sum += f_lev_in_new * f_lev_in_new;

    /* Update the RMS envelope */
    if( p_iparams->f_amp > p_iparams->f_env_rms )
    {
        p_iparams->f_env_rms = p_iparams->f_env_rms * p_aparams->f_ga
                             + p_iparams->f_amp * ( 1.0f - p_aparams->f_ga );
    }
    else
    {
        p_iparams->f_env_rms = p_iparams->f_env_rms * p_aparams->f_gr
                             + p_iparams->f_amp * ( 1.0f - p_aparams->f_gr );
    }
    RoundToZero( &p_iparams->f_env_rms );

    /* Update the peak envelope */
    if( f_lev_in_old > p_iparams->f_env_peak )
    {
        p_iparams->f_env_peak = p_iparams->f_env_peak * p_aparams->f_ga
                              + f_lev_in_old * ( 1.0f - p_aparams->f_ga );
    }
    else
    {
        p_iparams->f_env_peak = p_iparams->f_env_peak * p_aparams->f_gr
                              + f_lev_in_old * ( 1.0f - p_aparams->f_gr );
    }
    RoundToZero( &p_iparams->f_env_peak );

    /* Process the RMS value and update the output gain every 4 samples */
    if( ( p_iparams->i_count++ & 3 ) == 3 )
    {
        /* Process the RMS value by placing in the mean square value, and
         * reset the running sum */
        p_iparams->f_amp = RmsEnvProcess( &p_iparams->rms,
                                           p_iparams->f_sum * 0.25f );
        p_iparams->f_sum = 0.0f;
        if( isnan( p_iparams->f_env_rms ) )
        {
            /* This can happen sometimes, but I don't know why. */
            p_iparams->f_env_rms = 0.0f;
        }

        /* Find the superposition of the RMS and peak envelopes */
        p_iparams->f_env = LIN_INTERP( p_cparams->f_rms_peak,
                                       p_iparams->f_env_rms,
                                       p_iparams->f_env_peak );

        /* Update the output gain */
        if( p_iparams->f_env <= p_aparams->f_knee_min )
        {
            /* Gain below the knee (and below the threshold) */
            p_iparams->f_gain_out = 1.0f;
        }
        else if( p_iparams->f_env < p_aparams->f_knee_max )
        {
            /* Gain within the knee */
            const float f_x = -( p_cparams->f_threshold
                               - p_cparams->f_knee
                               - Lin2Db( p_iparams->f_env, p_sys ) )
                               / p_cparams->f_knee;

            p_iparams->f_gain_out = Db2Lin( -p_cparams->f_knee
                                           * p_aparams->f_rs * f_x * f_x * 0.25,
                                             p_sys );
        }
        else
        {
            /* Gain above the knee (and above the threshold) */
            p_iparams->f_gain_out = Db2Lin( ( p_cparams->f_threshold
                                    - Lin2Db( p_iparams->f_env, p_sys ) )
                                    * p_aparams->f_rs, p_sys );
        }
    }

    /* Find the total gain */
    p_iparams->f_gain = p_iparams->f_gain * p_aparams->f_ef_a
                      + p_iparams->f_gain_out * p_aparams->f_ef_ai;

    /* Write the resulting buffer to the output */
    BufferProcess( pf_buf, i_channels, p_iparams->f_gain, p_aparams->f_mug,
                   &p_iparams->la );
}

static void DbInit( filter_sys_t * p_sys )
{
    float *pf_lin_data = p_sys->pf_lin_data;
    float *pf_db_data = p_sys->pf_db_data;

    /* Fill linear lookup table */
    for( int i = 0; i < LIN_TABLE_SIZE; i++ )
    {
        pf_lin_data[i] = powf( 10.0f, ( ( DB_MAX - DB_MIN ) *
                   (float)i / LIN_TABLE_SIZE + DB_MIN ) / 20.0f );
    }

    /* Fill logarithmic lookup table */
    for( int i = 0; i < DB_TABLE_SIZE; i++ )
    {
        pf_db_data[i] = 20.0f * log10f( ( LIN_MAX - LIN_MIN ) *
                   (float)i / DB_TABLE_SIZE + LIN_MIN );
    }
}

static float Db2Lin( float f_db, filter_sys_t * p_sys )
{
    float f_scale = ( f_db - DB_MIN ) * LIN_TABLE_SIZE / ( DB_MAX - DB_MIN );
    int i_base = Round( f_scale - 0.5f );
    float f_ofs = f_scale - i_base;
    float *pf_lin_data = p_sys->pf_lin_data;

    if( i_base < 1 )
    {
        return 0.0f;
    }
    else if( i_base > LIN_TABLE_SIZE - 3 )
    {
        return pf_lin_data[LIN_TABLE_SIZE - 2];
    }

#ifdef DB_DEFAULT_CUBE
    return CubeInterp( f_ofs, pf_lin_data[i_base - 1],
                              pf_lin_data[i_base],
                              pf_lin_data[i_base + 1],
                              pf_lin_data[i_base + 2] );
#else
    return ( 1.0f - f_ofs ) * pf_lin_data[i_base]
                  + f_ofs   * pf_lin_data[i_base + 1];
#endif
}

static float Lin2Db( float f_lin, filter_sys_t * p_sys )
{
    float f_scale = ( f_lin - LIN_MIN ) * DB_TABLE_SIZE / ( LIN_MAX - LIN_MIN );
    int i_base = Round( f_scale - 0.5f );
    float f_ofs = f_scale - i_base;
    float *pf_db_data = p_sys->pf_db_data;

    if( i_base < 2 )
    {
        return pf_db_data[2] * f_scale * 0.5f - 23.0f * ( 2.0f - f_scale );
    }
    else if( i_base > DB_TABLE_SIZE - 3 )
    {
        return pf_db_data[DB_TABLE_SIZE - 2];
    }

#ifdef DB_DEFAULT_CUBE
    return CubeInterp( f_ofs, pf_db_data[i_base - 1],
                              pf_db_data[i_base],
                              pf_db_data[i_base + 1],
                              pf_db_data[i_base + 2] );
#else
    return ( 1.0f - f_ofs ) * pf_db_data[i_base]
                  + f_ofs   * pf_db_data[i_base + 1];
#endif
}

#ifdef DB_DEFAULT_CUBE
/* Cubic interpolation function */
static float CubeInterp( const float f_fr, const float f_inm1,
                                           const float f_in,
                                           const float f_inp1,
                                           const float f_inp2 )
{
    return f_in + 0.5f * f_fr * ( f_inp1 - f_inm1 +
         f_fr * ( 4.0f * f_inp1 + 2.0f * f_inm1 - 5.0f * f_in - f_inp2 +
         f_fr * ( 3.0f * ( f_in - f_inp1 ) - f_inm1 + f_inp2 ) ) );
}
#endif

/* Zero out denormals by adding and subtracting a small number, from Laurent
 * de Soras */
static void RoundToZero( float *pf_x )
{
    static const float f_anti_denormal = 1e-18;

    *pf_x += f_anti_denormal;
    *pf_x -= f_anti_denormal;
}

/* Zero out small enough numbers */
static void ForceSmallToZero( float *pf_x )
{
    if( fabs( *pf_x ) < 1.0e-6 )
    {
        *pf_x = 0.0f;
    }
}

/* A set of branchless clipping operations from Laurent de Soras */

static float Max( float f_x, float f_a )
{
    f_x -= f_a;
    f_x += fabs( f_x );
    f_x *= 0.5;
    f_x += f_a;

    return f_x;
}

static float Clamp( float f_x, float f_a, float f_b )
{
    const float f_x1 = fabs( f_x - f_a );
    const float f_x2 = fabs( f_x - f_b );

    f_x = f_x1 + f_a + f_b;
    f_x -= f_x2;
    f_x *= 0.5;

    return f_x;
}

/* Round float to int using IEEE int* hack */
static int Round( float f_x )
{
    ls_pcast32 p;

    p.f = f_x;
    p.f += ( 3 << 22 );

    return p.i - 0x4b400000;
}

/* Calculate current level from root-mean-squared of circular buffer ("RMS") */
static float RmsEnvProcess( rms_env * p_r, const float f_x )
{
    /* Remove the old term from the sum */
    p_r->f_sum -= p_r->pf_buf[p_r->i_pos];

    /* Add the new term to the sum */
    p_r->f_sum += f_x;

    /* If the sum is small enough, make it zero */
    ForceSmallToZero( &p_r->f_sum );

    /* Replace the old term in the array with the new one */
    p_r->pf_buf[p_r->i_pos] = f_x;

    /* Go to the next position for the next RMS calculation */
    p_r->i_pos = ( p_r->i_pos + 1 ) % ( p_r->i_count );

    /* Return the RMS value */
    return sqrt( p_r->f_sum / p_r->i_count );
}

/* Output the compressed delayed buffer and store the current buffer.  Uses a
 * circular array, just like the one used in calculating the RMS of the buffer
 */
static void BufferProcess( float * pf_buf, int i_channels, float f_gain,
                           float f_mug, lookahead * p_la )
{
    /* Loop through every channel */
    for( int i_chan = 0; i_chan < i_channels; i_chan++ )
    {
        float f_x = pf_buf[i_chan]; /* Current buffer value */

        /* Output the compressed delayed buffer value */
        pf_buf[i_chan] = p_la->p_buf[p_la->i_pos].pf_vals[i_chan]
                       * f_gain * f_mug;

        /* Update the delayed buffer value */
        p_la->p_buf[p_la->i_pos].pf_vals[i_chan] = f_x;
    }

    /* Go to the next delayed buffer value for the next run */
    p_la->i_pos = ( p_la->i_pos + 1 ) % ( p_la->i_count );
}
