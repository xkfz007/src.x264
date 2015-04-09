/***************************************************-*- coding: iso-8859-1 -*-
 * ratecontrol.c: h264 encoder library (Rate Control)
 *****************************************************************************
 * Copyright (C) 2005 x264 project
 * $Id: ratecontrol.c,v 1.1 2004/06/03 19:27:08 fenrir Exp $
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
 *          Michael Niedermayer <michaelni@gmx.at>
 *          Måns Rullgård <mru@mru.ath.cx>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111, USA.
 *****************************************************************************/

#define _ISOC99_SOURCE
#undef NDEBUG // always check asserts, the speed effect is far too small to disable them
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <assert.h>

#include "common.h"
#include "ratecontrol.h"
#include "analyse.h"

/*
#if defined(SYS_FREEBSD) || defined(SYS_BEOS) || defined(SYS_NETBSD)
#define exp2f(x) powf( 2, (x) )
#endif
#if defined(SYS_MACOSX)
#define exp2f(x) (float)pow( 2, (x) )
#define sqrtf sqrt
#endif
*/
#if defined(_MSC_VER)
#define isfinite _finite
#endif
#if defined(_MSC_VER) || defined(SYS_SunOS)
#define exp2f(x) pow( 2, (x) )
#define sqrtf sqrt
#endif
#ifdef WIN32 // POSIX says that rename() removes the destination, but win32 doesn't.
#define rename(src,dst) (unlink(dst), rename(src,dst))
#endif

#define COST_EST 0
#define COST_EST_AQ 1
#define INTRA_MBS 2
#define NUM_ROWS 3

typedef struct
{
    int pict_type;
    int kept_as_ref;
    float qscale;
    int mv_bits;
    int i_tex_bits;
    int p_tex_bits;
    int misc_bits;
    UINT64 expected_bits;
    float new_qscale;
    int new_qp;
    int i_count;
    int p_count;
    int s_count;
    float blurred_complexity;
    char direct_mode;
} ratecontrol_entry_t;

typedef struct
{
    double coeff;
    double count;
    double decay;
} predictor_t;

struct x264_ratecontrol_t
{
    /* constants */
    int b_abr;
    int b_2pass;
    int b_vbv;
    double fps;
    double bitrate;
    double rate_tolerance;
    int nmb;                    /* number of macroblocks in a frame */
    int qp_constant[5];

    /* current frame */
    ratecontrol_entry_t *rce;
    int qp;                     /* qp for current frame */
    int qpm;                    /* qp for current macroblock */
    float qpa_rc;               /* average of macroblocks' qp before aq */
    float qpa_rc_prev;
    int   qpa_aq;               /* average of macroblocks' qp after aq */
    int   qpa_aq_prev;
    float qp_novbv;             /* QP for the current frame if 1-pass VBV was disabled. */

    int slice_type;
    int qp_force;

    /* VBV stuff */
    double buffer_size;
    double buffer_fill;
    double buffer_rate;         /* # of bits added to buffer_fill after each frame */
    predictor_t pred[5];        /* predict frame size from satd */

    /* ABR stuff */
    int    last_satd;
    double last_rceq;
    double cplxr_sum;           /* sum of bits*qscale/rceq */
    double expected_bits_sum;   /* sum of qscale2bits after rceq, ratefactor, and overflow */
    double wanted_bits_window;  /* target bitrate * window */
    double cbr_decay;
    double short_term_cplxsum;
    double short_term_cplxcount;
    double rate_factor_constant;
    double ip_offset;
    double pb_offset;

    /* 2pass stuff */
    FILE *p_stat_file_out;
    char *psz_stat_file_tmpname;

    int num_entries;            /* number of ratecontrol_entry_ts */
    ratecontrol_entry_t *entry; /* FIXME: copy needed data and free this once init is done */
    double last_qscale;
    double last_qscale_for[5];  /* last qscale for a specific pict type, used for max_diff & ipb factor stuff  */
    int last_non_b_pict_type;
    double accum_p_qp;          /* for determining I-frame quant */
    double accum_p_norm;
    double last_accum_p_norm;
    double lmin[5];             /* min qscale by frame type */
    double lmax[5];
    double lstep;               /* max change (multiply) in qscale per frame */
    double i_cplx_sum[5];       /* estimated total texture bits in intra MBs at qscale=1 */
    double p_cplx_sum[5];
    double mv_bits_sum[5];
    int frame_count[5];         /* number of frames of each type */

    /* MBRC stuff */
    double frame_size_planned;
    int first_row, last_row;    /* region of the frame to be encoded by this thread */
    predictor_t *row_pred;
    predictor_t row_preds[5];
    predictor_t pred_b_from_p;  /* predict B-frame size from P-frame satd */
    int bframes;                /* # consecutive B-frames before this P-frame */
    int bframe_bits;            /* total cost of those frames */

    int i_zones;
    x264_zone_t *zones;
};


static int parse_zones( x264_t *h );
static int init_pass2(x264_t *);
static float rate_estimate_qscale( x264_t *h, int pict_type );
static void update_vbv( x264_t *h, int bits );
static double predict_size( predictor_t *p, double q, double var );
static void update_predictor( predictor_t *p, double q, double var, double bits );
int  x264_rc_analyse_slice( x264_t *h );

/* Terminology:
 * qp = h.264's quantizer
 * qscale = linearized quantizer = Lagrange multiplier
 */
static inline double qp2qscale(double qp)
{
    return 0.85 * pow(2.0, ( qp - 12.0 ) / 6.0);
}
static inline double qscale2qp(double qscale)
{
    return 12.0 + 6.0 * log(qscale/0.85) / log(2.0);
}

const UINT8 x264_exp2_lut[64] =
{
      0,   3,   6,   8,  11,  14,  17,  20,  23,  26,  29,  32,  36,  39,  42,  45,
     48,  52,  55,  58,  62,  65,  69,  72,  76,  80,  83,  87,  91,  94,  98, 102,
    106, 110, 114, 118, 122, 126, 130, 135, 139, 143, 147, 152, 156, 161, 165, 170,
    175, 179, 184, 189, 194, 198, 203, 208, 214, 219, 224, 229, 234, 240, 245, 250
};
/* Not a general-purpose function; multiplies input by -1/6 to convert
 * qp to qscale. */
static  int x264_exp2fix8( float x )
{
    int i = x*(-64.f/6.f) + 512.5f;
    if( i < 0 ) return 0;
    if( i > 1023 ) return 0xffff;
    return (x264_exp2_lut[i&63]+256) << (i>>6) >> 8;
}

/* Texture bitrate is not quite inversely proportional to qscale,
 * probably due the the changing number of SKIP blocks.
 * MV bits level off at about qp<=12, because the lambda used
 * for motion estimation is constant there. */
static inline double qscale2bits(ratecontrol_entry_t *rce, double qscale)
{
    if(qscale<0.1)
        qscale = 0.1;
    return (rce->i_tex_bits + rce->p_tex_bits + .1) * pow( rce->qscale / qscale, 1.1 )
           + rce->mv_bits * pow( X264_MAX(rce->qscale, 12) / X264_MAX(qscale, 12), 0.5 )
           + rce->misc_bits;
}
static inline UINT32 ac_energy_var( UINT64 sum_ssd, int shift, x264_frame_t *frame, int i, int b_store )
{
    UINT32 sum = sum_ssd;
    UINT32 ssd = sum_ssd >> 32;
    if( b_store )
    {
        frame->i_pixel_sum[i] += sum;
        frame->i_pixel_ssd[i] += ssd;
    }
    return ssd - ((UINT64)sum * sum >> shift);
}

static UINT32 x264_ac_energy_mb( x264_t *h, int mb_x, int mb_y, x264_frame_t *frame )
{
    UINT32 var;
  //  x264_prefetch_fenc( h, frame, mb_x, mb_y );

	//var  = ac_energy_plane( h, mb_x, mb_y, frame, 0, 0, PARAM_INTERLACED, 1 );
	int offset,stride;
	stride=frame->i_stride[0];
	offset=16*mb_x+16*mb_y*stride;
	var=ac_energy_var( h->pixf.var[PIXEL_16x16]( frame->plane[0] + offset, stride ), 8, frame, 0, 1);

	stride=frame->i_stride[1];
	offset=8*mb_x+8*mb_y*stride;
	var+=ac_energy_var( h->pixf.var[PIXEL_8x8]( frame->plane[1] + offset, stride ), 6, frame, 0, 1);

	stride=frame->i_stride[2];
	offset=8*mb_x+8*mb_y*stride;
	var+=ac_energy_var( h->pixf.var[PIXEL_8x8]( frame->plane[2] + offset, stride ), 6, frame, 0, 1);

	return var;
}
void x264_adaptive_quant_frame( x264_t *h, x264_frame_t *frame, float *quant_offsets )
{
float strength;
    float avg_adj = 0.f;
	int i,mb_x,mb_y,mb_xy;
    /* Initialize frame stats */
    for( i = 0; i < 3; i++ )
    {
        frame->i_pixel_sum[i] = 0;
        frame->i_pixel_ssd[i] = 0;
    }
    /* Degenerate cases */
    if( h->param.rc.i_aq_mode == X264_AQ_NONE || h->param.rc.f_aq_strength == 0 )
    {
        /* Need to init it anyways for MB tree */
        if( h->param.rc.i_aq_mode && h->param.rc.f_aq_strength == 0 )
        {
			/*
            if( quant_offsets )
            {
                for( int mb_xy = 0; mb_xy < h->mb.i_mb_count; mb_xy++ )
                    frame->f_qp_offset[mb_xy] = frame->f_qp_offset_aq[mb_xy] = quant_offsets[mb_xy];
                if( h->frames.b_have_lowres )
                    for( int mb_xy = 0; mb_xy < h->mb.i_mb_count; mb_xy++ )
                        frame->i_inv_qscale_factor[mb_xy] = x264_exp2fix8( frame->f_qp_offset[mb_xy] );
            }
            else
			*/
            {
                memset( frame->f_qp_offset, 0, h->mb.i_mb_count * sizeof(float) );
                memset( frame->f_qp_offset_aq, 0, h->mb.i_mb_count * sizeof(float) );
                if( h->frames.b_have_lowres )
                    for( mb_xy = 0; mb_xy < h->mb.i_mb_count; mb_xy++ )
                        frame->i_inv_qscale_factor[mb_xy] = 256;
            }
        }
	}
    else
    {
        if( h->param.rc.i_aq_mode == X264_AQ_AUTOVARIANCE )
        {
            float bit_depth_correction = powf(1 << (X264_BIT_DEPTH-8), 0.5f);
            float avg_adj_pow2 = 0.f;
            for( mb_y = 0; mb_y < h->sps->i_mb_height; mb_y++ )
                for( mb_x = 0; mb_x < h->sps->i_mb_width; mb_x++ )
                {
                    UINT32 energy = x264_ac_energy_mb( h, mb_x, mb_y, frame );
                    float qp_adj = powf( energy + 1, 0.125f );
                    frame->f_qp_offset[mb_x + mb_y*h->mb.i_mb_stride] = qp_adj;
                    avg_adj += qp_adj;
                    avg_adj_pow2 += qp_adj * qp_adj;
                }
            avg_adj /= h->mb.i_mb_count;
            avg_adj_pow2 /= h->mb.i_mb_count;
            strength = h->param.rc.f_aq_strength * avg_adj / bit_depth_correction;
            avg_adj = avg_adj - 0.5f * (avg_adj_pow2 - (14.f * bit_depth_correction)) / avg_adj;
        }
        else
            strength = h->param.rc.f_aq_strength * 1.0397f;

        for( mb_y = 0; mb_y < h->sps->i_mb_height; mb_y++ )
            for( mb_x = 0; mb_x < h->sps->i_mb_width; mb_x++ )
            {
                float qp_adj;
                int mb_xy = mb_x + mb_y*h->mb.i_mb_stride;
				//fprintf(stderr,"mb_y=%d my_x=%d\n",mb_y,mb_x);
				//fflush(stdout);
                if( h->param.rc.i_aq_mode == X264_AQ_AUTOVARIANCE )
                {
                    qp_adj = frame->f_qp_offset[mb_xy];
                    qp_adj = strength * (qp_adj - avg_adj);
                }
                else
                {
                    UINT32 energy = x264_ac_energy_mb( h, mb_x, mb_y, frame );
                    qp_adj = strength * (log( X264_MAX(energy, 1)*1.0 )/log(2.0) - (14.427f + 2*(X264_BIT_DEPTH-8)));
                }
				/*
                if( quant_offsets )
                    qp_adj += quant_offsets[mb_xy];
					*/
                frame->f_qp_offset[mb_xy] =
                frame->f_qp_offset_aq[mb_xy] = qp_adj;
                if( h->frames.b_have_lowres )
                    frame->i_inv_qscale_factor[mb_xy] =x264_exp2fix8(qp_adj);
            }
    }

    /* Remove mean from SSD calculation */
    for( i = 0; i < 3; i++ )
    {
        UINT64 ssd = frame->i_pixel_ssd[i];
        UINT64 sum = frame->i_pixel_sum[i];
        int width  = 16*h->sps->i_mb_width  >> (i && 1);//CHROMA_H_SHIFT);
        int height = 16*h->sps->i_mb_height >> (i && 1);//CHROMA_V_SHIFT);
        frame->i_pixel_ssd[i] = ssd - (sum * sum + width * height / 2) / (width * height);
    }
}

int x264_ratecontrol_new( x264_t *h )
{
    x264_ratecontrol_t *rc;
    int i;

 //   x264_cpu_restore( h->param.cpu );

    h->rc = rc = x264_malloc( h->param.i_threads * sizeof(x264_ratecontrol_t) );
    memset( rc, 0, h->param.i_threads * sizeof(x264_ratecontrol_t) );

    rc->b_abr = ( h->param.rc.b_cbr || h->param.rc.i_rf_constant ) && !h->param.rc.b_stat_read;
    rc->b_2pass = h->param.rc.b_cbr && h->param.rc.b_stat_read;
    
    /* FIXME: use integers */
    if(h->param.i_fps_num > 0 && h->param.i_fps_den > 0)
        rc->fps = (float) h->param.i_fps_num / h->param.i_fps_den;
    else
        rc->fps = 25.0;

    rc->bitrate = h->param.rc.i_bitrate * 1000;
    rc->rate_tolerance = h->param.rc.f_rate_tolerance;
    rc->nmb = h->mb.i_mb_count;
    rc->last_non_b_pict_type = -1;
    rc->cbr_decay = 1.0;

    if( h->param.rc.i_rf_constant && h->param.rc.b_stat_read )
    {
        x264_log(h, X264_LOG_ERROR, "constant rate-factor is incompatible with 2pass.\n");
        return -1;
    }
    if( h->param.rc.i_vbv_buffer_size && !h->param.rc.b_cbr && !h->param.rc.i_rf_constant )
        x264_log(h, X264_LOG_ERROR, "VBV is incompatible with constant QP.\n");
    if( h->param.rc.i_vbv_buffer_size && h->param.rc.b_cbr
        && h->param.rc.i_vbv_max_bitrate == 0 )
    {
        x264_log( h, X264_LOG_DEBUG, "VBV maxrate unspecified, assuming CBR\n" );
        h->param.rc.i_vbv_max_bitrate = h->param.rc.i_bitrate;
    }
    if( h->param.rc.i_vbv_max_bitrate < h->param.rc.i_bitrate &&
        h->param.rc.i_vbv_max_bitrate > 0)
        x264_log(h, X264_LOG_ERROR, "max bitrate less than average bitrate, ignored.\n");
    else if( h->param.rc.i_vbv_max_bitrate > 0 &&
             h->param.rc.i_vbv_buffer_size > 0 )
    {
        if( h->param.rc.i_vbv_buffer_size < 3 * h->param.rc.i_vbv_max_bitrate / rc->fps ) {
            h->param.rc.i_vbv_buffer_size = 3 * h->param.rc.i_vbv_max_bitrate / rc->fps;
            x264_log( h, X264_LOG_ERROR, "VBV buffer size too small, using %d kbit\n",
                      h->param.rc.i_vbv_buffer_size );
        }
        rc->buffer_rate = h->param.rc.i_vbv_max_bitrate * 1000 / rc->fps;
        rc->buffer_size = h->param.rc.i_vbv_buffer_size * 1000;
        rc->buffer_fill = rc->buffer_size * h->param.rc.f_vbv_buffer_init;
        rc->cbr_decay = 1.0 - rc->buffer_rate / rc->buffer_size
                      * 0.5 * X264_MAX(0, 1.5 - rc->buffer_rate * rc->fps / rc->bitrate);
        rc->b_vbv = 1;
    }
    else if( h->param.rc.i_vbv_max_bitrate )
        x264_log(h, X264_LOG_ERROR, "VBV maxrate specified, but no bufsize.\n");
    if(rc->rate_tolerance < 0.01) {
        x264_log(h, X264_LOG_ERROR, "bitrate tolerance too small, using .01\n");
        rc->rate_tolerance = 0.01;
    }

    h->mb.b_variable_qp = rc->b_vbv && !rc->b_2pass|| h->param.rc.i_aq_mode;

    if( rc->b_abr )
    {
        /* FIXME shouldn't need to arbitrarily specify a QP,
         * but this is more robust than BPP measures */
#define ABR_INIT_QP ( h->param.rc.i_rf_constant > 0 ? h->param.rc.i_rf_constant : 24 )
        rc->accum_p_norm = .01;
        rc->accum_p_qp = ABR_INIT_QP * rc->accum_p_norm;
        rc->cplxr_sum = .01*pow( 7.0e5, (double)h->param.rc.f_qcompress ) * pow( h->mb.i_mb_count, 0.5 );
        rc->wanted_bits_window = 1.0 * rc->bitrate / rc->fps;//.01;
        rc->last_non_b_pict_type = SLICE_TYPE_I;
    }

    if( h->param.rc.i_rf_constant )
    {
        /* arbitrary rescaling to make CRF somewhat similar to QP */
        double base_cplx = h->mb.i_mb_count * (h->param.i_bframe ? 120 : 80);
        rc->rate_factor_constant = pow( base_cplx, 1.0 - h->param.rc.f_qcompress )
                                 / qp2qscale( h->param.rc.i_rf_constant );
    }

    rc->ip_offset = 6.0 * log(h->param.rc.f_ip_factor) / log(2.0);
    rc->pb_offset = 6.0 * log(h->param.rc.f_pb_factor) / log(2.0);
    rc->qp_constant[SLICE_TYPE_P] = h->param.rc.i_qp_constant;
    rc->qp_constant[SLICE_TYPE_I] = x264_clip3( h->param.rc.i_qp_constant - rc->ip_offset + 0.5, 0, 51 );
    rc->qp_constant[SLICE_TYPE_B] = x264_clip3( h->param.rc.i_qp_constant + rc->pb_offset + 0.5, 0, 51 );

    rc->lstep = exp2f(h->param.rc.i_qp_step / 6.0);
    rc->last_qscale = qp2qscale(26);
    for( i = 0; i < 5; i++ )
    {
        rc->last_qscale_for[i] = qp2qscale(26);
        rc->lmin[i] = qp2qscale( h->param.rc.i_qp_min );
        rc->lmax[i] = qp2qscale( h->param.rc.i_qp_max );
        rc->pred[i].coeff= 2.0;
        rc->pred[i].count= 1.0;
        rc->pred[i].decay= 0.5;
		rc->row_preds[i].coeff= .25;
		rc->row_preds[i].count= 1.0;
		rc->row_preds[i].decay= 0.5;
    }
    rc->pred_b_from_p = rc->pred[0];

    if( parse_zones( h ) < 0 )
        return -1;

    /* Load stat file and init 2pass algo */
    if( h->param.rc.b_stat_read )
    {
        char *p, *stats_in, *stats_buf;

        /* read 1st pass stats */
        assert( h->param.rc.psz_stat_in );
        stats_buf = stats_in = x264_slurp_file( h->param.rc.psz_stat_in );
        if( !stats_buf )
        {
            x264_log(h, X264_LOG_ERROR, "ratecontrol_init: can't open stats file\n");
            return -1;
        }

        /* check whether 1st pass options were compatible with current options */
        if( !strncmp( stats_buf, "#options:", 9 ) )
        {
            int i;
            char *opts = stats_buf;
            stats_in = strchr( stats_buf, '\n' );
            if( !stats_in )
                return -1;
            *stats_in = '\0';
            stats_in++;

            if( ( p = strstr( opts, "bframes=" ) ) && sscanf( p, "bframes=%d", &i )
                && h->param.i_bframe != i )
            {
                x264_log( h, X264_LOG_ERROR, "different number of B-frames than 1st pass (%d vs %d)\n",
                          h->param.i_bframe, i );
                return -1;
            }

            /* since B-adapt doesn't (yet) take into account B-pyramid,
             * the converse is not a problem */
            if( strstr( opts, "b_pyramid=1" ) && !h->param.b_bframe_pyramid )
                x264_log( h, X264_LOG_WARNING, "1st pass used B-pyramid, 2nd doesn't\n" );

            if( ( p = strstr( opts, "keyint=" ) ) && sscanf( p, "keyint=%d", &i )
                && h->param.i_keyint_max != i )
                x264_log( h, X264_LOG_WARNING, "different keyint than 1st pass (%d vs %d)\n",
                          h->param.i_keyint_max, i );

            if( strstr( opts, "qp=0" ) && h->param.rc.b_cbr )
                x264_log( h, X264_LOG_WARNING, "1st pass was lossless, bitrate prediction will be inaccurate\n" );
        }

        /* find number of pics */
        p = stats_in;
        for(i=-1; p; i++)
            p = strchr(p+1, ';');
        if(i==0)
        {
            x264_log(h, X264_LOG_ERROR, "empty stats file\n");
            return -1;
        }
        rc->num_entries = i;

        if( h->param.i_frame_total < rc->num_entries && h->param.i_frame_total > 0 )
        {
            x264_log( h, X264_LOG_WARNING, "2nd pass has fewer frames than 1st pass (%d vs %d)\n",
                      h->param.i_frame_total, rc->num_entries );
        }
        if( h->param.i_frame_total > rc->num_entries + h->param.i_bframe )
        {
            x264_log( h, X264_LOG_ERROR, "2nd pass has more frames than 1st pass (%d vs %d)\n",
                      h->param.i_frame_total, rc->num_entries );
            return -1;
        }

        /* FIXME: ugly padding because VfW drops delayed B-frames */
        rc->num_entries += h->param.i_bframe;

        rc->entry = (ratecontrol_entry_t*) x264_malloc(rc->num_entries * sizeof(ratecontrol_entry_t));
        memset(rc->entry, 0, rc->num_entries * sizeof(ratecontrol_entry_t));

        /* init all to skipped p frames */
        for(i=0; i<rc->num_entries; i++){
            ratecontrol_entry_t *rce = &rc->entry[i];
            rce->pict_type = SLICE_TYPE_P;
            rce->qscale = rce->new_qscale = qp2qscale(20);
            rce->misc_bits = rc->nmb + 10;
            rce->new_qp = 0;
        }

        /* read stats */
        p = stats_in;
        for(i=0; i < rc->num_entries - h->param.i_bframe; i++){
            ratecontrol_entry_t *rce;
            int frame_number;
            char pict_type;
            int e;
            char *next;
            float qp;

            next= strchr(p, ';');
            if(next){
                (*next)=0; //sscanf is unbelievably slow on looong strings
                next++;
            }
            e = sscanf(p, " in:%d ", &frame_number);

            if(frame_number < 0 || frame_number >= rc->num_entries)
            {
                x264_log(h, X264_LOG_ERROR, "bad frame number (%d) at stats line %d\n", frame_number, i);
                return -1;
            }
            rce = &rc->entry[frame_number];
            rce->direct_mode = 0;

            e += sscanf(p, " in:%*d out:%*d type:%c q:%f itex:%d ptex:%d mv:%d misc:%d imb:%d pmb:%d smb:%d d:%c",
                   &pict_type, &qp, &rce->i_tex_bits, &rce->p_tex_bits,
                   &rce->mv_bits, &rce->misc_bits, &rce->i_count, &rce->p_count,
                   &rce->s_count, &rce->direct_mode);

            switch(pict_type){
                case 'I': rce->kept_as_ref = 1;
                case 'i': rce->pict_type = SLICE_TYPE_I; break;
                case 'P': rce->pict_type = SLICE_TYPE_P; break;
                case 'B': rce->kept_as_ref = 1;
                case 'b': rce->pict_type = SLICE_TYPE_B; break;
                default:  e = -1; break;
            }
            if(e < 10){
                x264_log(h, X264_LOG_ERROR, "statistics are damaged at line %d, parser out=%d\n", i, e);
                return -1;
            }
            rce->qscale = qp2qscale(qp);
            p = next;
        }

        x264_free(stats_buf);

        if(h->param.rc.b_cbr)
        {
            if(init_pass2(h) < 0) return -1;
        } /* else we're using constant quant, so no need to run the bitrate allocation */
    }

    /* Open output file */
    /* If input and output files are the same, output to a temp file
     * and move it to the real name only when it's complete */
    if( h->param.rc.b_stat_write )
    {
        char *p;

        rc->psz_stat_file_tmpname = x264_malloc( strlen(h->param.rc.psz_stat_out) + 6 );
        strcpy( rc->psz_stat_file_tmpname, h->param.rc.psz_stat_out );
        strcat( rc->psz_stat_file_tmpname, ".temp" );

        rc->p_stat_file_out = fopen( rc->psz_stat_file_tmpname, "wb" );
        if( rc->p_stat_file_out == NULL )
        {
            x264_log(h, X264_LOG_ERROR, "ratecontrol_init: can't open stats file\n");
            return -1;
        }

        p = x264_param2string( &h->param, 1 );
        fprintf( rc->p_stat_file_out, "#options: %s\n", p );
        x264_free( p );
    }

    return 0;
}

static int parse_zones( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;
    int i;
    if( h->param.rc.psz_zones && !h->param.rc.i_zones )
    {
        char *p;
        h->param.rc.i_zones = 1;
        for( p = h->param.rc.psz_zones; *p; p++ )
            h->param.rc.i_zones += (*p == '/');
        h->param.rc.zones = x264_malloc( h->param.rc.i_zones * sizeof(x264_zone_t) );
        p = h->param.rc.psz_zones;
        for( i = 0; i < h->param.rc.i_zones; i++)
        {
            x264_zone_t *z = &h->param.rc.zones[i];
            if( 3 == sscanf(p, "%u,%u,q=%u", &z->i_start, &z->i_end, &z->i_qp) )
                z->b_force_qp = 1;
            else if( 3 == sscanf(p, "%u,%u,b=%f", &z->i_start, &z->i_end, &z->f_bitrate_factor) )
                z->b_force_qp = 0;
            else
            {
                char *slash = strchr(p, '/');
                if(slash) *slash = '\0';
                x264_log( h, X264_LOG_ERROR, "invalid zone: \"%s\"\n", p );
                return -1;
            }
            p = strchr(p, '/') + 1;
        }
    }

    if( h->param.rc.i_zones > 0 )
    {
        for( i = 0; i < h->param.rc.i_zones; i++ )
        {
            x264_zone_t z = h->param.rc.zones[i];
            if( z.i_start < 0 || z.i_start > z.i_end )
            {
                x264_log( h, X264_LOG_ERROR, "invalid zone: start=%d end=%d\n",
                          z.i_start, z.i_end );
                return -1;
            }
            else if( !z.b_force_qp && z.f_bitrate_factor <= 0 )
            {
                x264_log( h, X264_LOG_ERROR, "invalid zone: bitrate_factor=%f\n",
                          z.f_bitrate_factor );
                return -1;
            }
        }

        rc->i_zones = h->param.rc.i_zones;
        rc->zones = x264_malloc( rc->i_zones * sizeof(x264_zone_t) );
        memcpy( rc->zones, h->param.rc.zones, rc->i_zones * sizeof(x264_zone_t) );
    }

    return 0;
}

void x264_ratecontrol_summary( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;
    if( rc->b_abr && !h->param.rc.i_rf_constant && !h->param.rc.i_vbv_max_bitrate )
    {
        double base_cplx = h->mb.i_mb_count * (h->param.i_bframe ? 120 : 80);
        x264_log( h, X264_LOG_INFO, "final ratefactor: %.2f\n", 
                  qscale2qp( pow( base_cplx, 1 - h->param.rc.f_qcompress )
                             * rc->cplxr_sum / rc->wanted_bits_window ) );
    }
}

void x264_ratecontrol_delete( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;

    if( rc->p_stat_file_out )
    {
        fclose( rc->p_stat_file_out );
        if( h->i_frame >= rc->num_entries - h->param.i_bframe )
            if( rename( rc->psz_stat_file_tmpname, h->param.rc.psz_stat_out ) != 0 )
            {
                x264_log( h, X264_LOG_ERROR, "failed to rename \"%s\" to \"%s\"\n",
                          rc->psz_stat_file_tmpname, h->param.rc.psz_stat_out );
            }
        x264_free( rc->psz_stat_file_tmpname );
    }
    x264_free( rc->entry );
    x264_free( rc->zones );
    x264_free( rc );
}

/* Before encoding a frame, choose a QP for it */
void x264_ratecontrol_start( x264_t *h, int i_slice_type, int i_force_qp )
{
    x264_ratecontrol_t *rc = h->rc;
    ratecontrol_entry_t *rce = NULL;
	float q;

    //x264_cpu_restore( h->param.cpu );

    rc->qp_force = i_force_qp;
    rc->slice_type = i_slice_type;

    if( h->param.rc.b_stat_read )
    {
        int frame = h->fenc->i_frame;
        assert( frame >= 0 && frame < rc->num_entries );
        rce = h->rc->rce = &h->rc->entry[frame];

        if( i_slice_type == SLICE_TYPE_B
            && h->param.analyse.i_direct_mv_pred == X264_DIRECT_PRED_AUTO )
        {
            h->sh.b_direct_spatial_mv_pred = ( rce->direct_mode == 's' );
            h->mb.b_direct_auto_read = ( rce->direct_mode == 's' || rce->direct_mode == 't' );
        }
    }

    if( h->fdec->i_row_bits )
    {
        memset( h->fdec->i_row_bits, 0, h->sps->i_mb_height * sizeof(int) );
    }

    if( i_slice_type != SLICE_TYPE_B )
    {
        rc->bframe_bits = 0;
        rc->bframes = 0;
        while( h->frames.current[rc->bframes] && IS_X264_TYPE_B(h->frames.current[rc->bframes]->i_type) )
            rc->bframes++;
    }

   // rc->qpa = 0;

    if( i_force_qp )
    {
        //rc->qpm = rc->qp = i_force_qp - 1;
		q=i_force_qp-1;
    }
    else if( rc->b_abr )
    {
       // rc->qpm = rc->qp =
        //    x264_clip3( (int)(qscale2qp( rate_estimate_qscale( h, i_slice_type ) ) + .5), 0, 51 );
		q=qscale2qp(rate_estimate_qscale(h,i_slice_type));
    }
    else if( rc->b_2pass )
    {
        rce->new_qscale = rate_estimate_qscale( h, i_slice_type );
//        rc->qpm = rc->qp = rce->new_qp =
//           x264_clip3( (int)(qscale2qp(rce->new_qscale) + 0.5), 0, 51 );
		q=qscale2qp(rce->new_qscale);
    }
    else /* CQP */
    {
       // int q;
        if( i_slice_type == SLICE_TYPE_B && h->fdec->b_kept_as_ref )
            q = ( rc->qp_constant[ SLICE_TYPE_B ] + rc->qp_constant[ SLICE_TYPE_P ] ) / 2;
        else
            q = rc->qp_constant[ i_slice_type ];
    //    rc->qpm = rc->qp = q;
    }

    q = x264_clip3f( q, h->param.rc.i_qp_min, h->param.rc.i_qp_max );

    rc->qpa_rc = rc->qpa_rc_prev =
    rc->qpa_aq = rc->qpa_aq_prev = 0;
    rc->qp = x264_clip3( q + 0.5f, 0, QP_MAX );
    h->fdec->f_qp_avg_rc =
    h->fdec->f_qp_avg_aq =
    rc->qpm = q;
    if( rce )
        rce->new_qp = rc->qp;
}

double predict_row_size( x264_t *h, int y, int qp )
{
    /* average between two predictors:
     * absolute SATD, and scaled bit cost of the colocated row in the previous frame */
    x264_ratecontrol_t *rc = h->rc;
    double pred_s = predict_size( rc->row_pred, qp2qscale(qp), h->fdec->i_row_satd[y] );
    double pred_t = 0;
    if( rc->slice_type != SLICE_TYPE_I 
        && h->fref0[0]->i_type == h->fdec->i_type
        && h->fref0[0]->i_row_satd[y] > 0 )
    {
        pred_t = h->fref0[0]->i_row_bits[y] * h->fdec->i_row_satd[y] / h->fref0[0]->i_row_satd[y]
                 * qp2qscale(h->fref0[0]->i_row_qp[y]) / qp2qscale(qp);
    }
    if( pred_t == 0 )
        pred_t = pred_s;

    return (pred_s + pred_t) / 2;
}

double predict_row_size_sum( x264_t *h, int y, int qp )
{
    int i;
    double bits = 0;
    for( i = h->rc->first_row; i <= y; i++ )
        bits += h->fdec->i_row_bits[i];
    for( i = y+1; i <= h->rc->last_row; i++ )
        bits += predict_row_size( h, i, qp );
    return bits;
}

void x264_ratecontrol_mb( x264_t *h, int bits )
{
    x264_ratecontrol_t *rc = h->rc;
    const int y = h->mb.i_mb_y;

    //x264_cpu_restore( h->param.cpu );

    h->fdec->i_row_bits[y] += bits;
    rc->qpa_aq += h->mb.i_qp;//rc->qpm;
    rc->qpa_rc += rc->qpm ;

    if( h->mb.i_mb_x != h->sps->i_mb_width - 1 || !h->mb.b_variable_qp )
        return;


    if( !rc->b_vbv )
        return ;

    h->fdec->i_row_qp[y] = rc->qpm;

    if( rc->slice_type == SLICE_TYPE_B )
    {
        /* B-frames shouldn't use lower QP than their reference frames */
        if( y < rc->last_row )
        {
            rc->qpm = X264_MAX( rc->qp,
                      X264_MIN( h->fref0[0]->i_row_qp[y+1],
                                h->fref1[0]->i_row_qp[y+1] ));
        }
    }
    else
    {
        update_predictor( rc->row_pred, qp2qscale(rc->qpm), h->fdec->i_row_satd[y], h->fdec->i_row_bits[y] );

        /* tweak quality based on difference from predicted size */
        if( y < rc->last_row && h->stat.i_slice_count[rc->slice_type] > 0 )
        {
            int prev_row_qp = h->fdec->i_row_qp[y];
            int b0 = predict_row_size_sum( h, y, rc->qpm );
            int b1 = b0;
            int i_qp_max = X264_MIN( prev_row_qp + h->param.rc.i_qp_step, h->param.rc.i_qp_max );
            int i_qp_min = X264_MAX( prev_row_qp - h->param.rc.i_qp_step, h->param.rc.i_qp_min );
            float buffer_left_planned = rc->buffer_fill - rc->frame_size_planned;

            while( rc->qpm < i_qp_max
                   && (b1 > rc->frame_size_planned * 1.15
                    || (rc->buffer_fill - b1 < buffer_left_planned * 0.5)))
            {
                rc->qpm ++;
                b1 = predict_row_size_sum( h, y, rc->qpm );
            }

            while( rc->qpm > i_qp_min
                   && buffer_left_planned > rc->buffer_size * 0.4
                   && ((b1 < rc->frame_size_planned * 0.8 && rc->qpm <= prev_row_qp)
                     || b1 < (rc->buffer_fill - rc->buffer_size + rc->buffer_rate) * 1.1) )
            {
                rc->qpm --;
                b1 = predict_row_size_sum( h, y, rc->qpm );
            }
        }
    }
}

int x264_ratecontrol_qp( x264_t *h )
{
    return h->rc->qpm;
}
int x264_ratecontrol_mb_qp( x264_t *h )
{
    float qp = h->rc->qpm;
    if( h->param.rc.i_aq_mode )
    {
         /* MB-tree currently doesn't adjust quantizers in unreferenced frames. */
        float qp_offset = h->fdec->b_kept_as_ref ? h->fenc->f_qp_offset[h->mb.i_mb_xy] : h->fenc->f_qp_offset_aq[h->mb.i_mb_xy];
        /* Scale AQ's effect towards zero in emergency mode. */
        if( qp > QP_MAX_SPEC )
            qp_offset *= (QP_MAX - qp) / (QP_MAX - QP_MAX_SPEC);
        qp += qp_offset;
    }
    return x264_clip3( qp + 0.5f, h->param.rc.i_qp_min, h->param.rc.i_qp_max );
}
/* In 2pass, force the same frame types as in the 1st pass */
int x264_ratecontrol_slice_type( x264_t *h, int frame_num )
{
    x264_ratecontrol_t *rc = h->rc;
    if( h->param.rc.b_stat_read )
    {
        if( frame_num >= rc->num_entries )
        {
            /* We could try to initialize everything required for ABR and
             * adaptive B-frames, but that would be complicated.
             * So just calculate the average QP used so far. */

            h->param.rc.i_qp_constant = (h->stat.i_slice_count[SLICE_TYPE_P] == 0) ? 24
                                      : 1 + h->stat.f_frame_qp[SLICE_TYPE_P] / h->stat.i_slice_count[SLICE_TYPE_P];
            rc->qp_constant[SLICE_TYPE_P] = x264_clip3( h->param.rc.i_qp_constant, 0, 51 );
            rc->qp_constant[SLICE_TYPE_I] = x264_clip3( (int)( qscale2qp( qp2qscale( h->param.rc.i_qp_constant ) / fabs( h->param.rc.f_ip_factor )) + 0.5 ), 0, 51 );
            rc->qp_constant[SLICE_TYPE_B] = x264_clip3( (int)( qscale2qp( qp2qscale( h->param.rc.i_qp_constant ) * fabs( h->param.rc.f_pb_factor )) + 0.5 ), 0, 51 );

            x264_log(h, X264_LOG_ERROR, "2nd pass has more frames than 1st pass (%d)\n", rc->num_entries);
            x264_log(h, X264_LOG_ERROR, "continuing anyway, at constant QP=%d\n", h->param.rc.i_qp_constant);
            if( h->param.i_bframe_adaptive )
                x264_log(h, X264_LOG_ERROR, "disabling adaptive B-frames\n");

            rc->b_abr = 0;
            rc->b_2pass = 0;
            h->param.rc.b_cbr = 0;
            h->param.rc.b_stat_read = 0;
            h->param.i_bframe_adaptive = 0;
            if( h->param.i_bframe > 1 )
                h->param.i_bframe = 1;
            return X264_TYPE_P;
        }
        switch( rc->entry[frame_num].pict_type )
        {
            case SLICE_TYPE_I:
                return rc->entry[frame_num].kept_as_ref ? X264_TYPE_IDR : X264_TYPE_I;

            case SLICE_TYPE_B:
                return rc->entry[frame_num].kept_as_ref ? X264_TYPE_BREF : X264_TYPE_B;

            case SLICE_TYPE_P:
            default:
                return X264_TYPE_P;
        }
    }
    else
    {
        return X264_TYPE_AUTO;
    }
}

/* After encoding one frame, save stats and update ratecontrol state */
void x264_ratecontrol_end( x264_t *h, int bits )
{
    x264_ratecontrol_t *rc = h->rc;
    const int *mbs = h->stat.frame.i_mb_count;
    int i;

    //x264_cpu_restore( h->param.cpu );

    h->stat.frame.i_mb_count_skip = mbs[P_SKIP] + mbs[B_SKIP];
    h->stat.frame.i_mb_count_i = mbs[I_16x16] + mbs[I_8x8] + mbs[I_4x4];
    h->stat.frame.i_mb_count_p = mbs[P_L0] + mbs[P_8x8];
    for( i = B_DIRECT; i < B_8x8; i++ )
        h->stat.frame.i_mb_count_p += mbs[i];
	/*
    if( h->mb.b_variable_qp )
    {
        for( i = 1; i < h->param.i_threads; i++ )
            rc->qpa += rc[i].qpa;
        rc->qpa /= h->mb.i_mb_count;
    }
    else
        rc->qpa = rc->qp;
		*/
    h->fdec->f_qp_avg_rc = rc->qpa_rc /= h->mb.i_mb_count;
    h->fdec->f_qp_avg_aq = (float)rc->qpa_aq / h->mb.i_mb_count;
    //h->fdec->f_qp_avg = rc->qpa;

    if( h->param.rc.b_stat_write )
    {
        char c_type = rc->slice_type==SLICE_TYPE_I ? (h->fenc->i_poc==0 ? 'I' : 'i')
                    : rc->slice_type==SLICE_TYPE_P ? 'P'
                    : h->fenc->b_kept_as_ref ? 'B' : 'b';
        int dir_frame = h->stat.frame.i_direct_score[1] - h->stat.frame.i_direct_score[0];
        int dir_avg = h->stat.i_direct_score[1] - h->stat.i_direct_score[0];
        char c_direct = h->mb.b_direct_auto_write ?
                        ( dir_frame>0 ? 's' : dir_frame<0 ? 't' : 
                          dir_avg>0 ? 's' : dir_avg<0 ? 't' : '-' )
                        : '-';
        fprintf( rc->p_stat_file_out,
                 "in:%d out:%d type:%c q:%.2f itex:%d ptex:%d mv:%d misc:%d imb:%d pmb:%d smb:%d d:%c;\n",
                 h->fenc->i_frame, h->i_frame,
                 c_type, rc->qpa_rc,
                 h->stat.frame.i_itex_bits, h->stat.frame.i_ptex_bits,
                 h->stat.frame.i_hdr_bits, h->stat.frame.i_misc_bits,
                 h->stat.frame.i_mb_count_i,
                 h->stat.frame.i_mb_count_p,
                 h->stat.frame.i_mb_count_skip,
                 c_direct);
    }

    if( rc->b_abr )
    {
        if( rc->slice_type != SLICE_TYPE_B )
            rc->cplxr_sum += bits * qp2qscale(rc->qpa_rc) / rc->last_rceq;
        else
        {
            /* Depends on the fact that B-frame's QP is an offset from the following P-frame's.
             * Not perfectly accurate with B-refs, but good enough. */
            rc->cplxr_sum += bits * qp2qscale(rc->qpa_rc) / (rc->last_rceq * fabs(h->param.rc.f_pb_factor));
        }
        rc->cplxr_sum *= rc->cbr_decay;
        rc->wanted_bits_window += rc->bitrate / rc->fps;
        rc->wanted_bits_window *= rc->cbr_decay;

        rc->accum_p_qp   *= .95;
        rc->accum_p_norm *= .95;
        rc->accum_p_norm += 1;
        if( rc->slice_type == SLICE_TYPE_I )
            rc->accum_p_qp += rc->qpa_rc * fabs(h->param.rc.f_ip_factor);
        else
            rc->accum_p_qp += rc->qpa_rc;
    }

    if( rc->b_2pass )
    {
        rc->expected_bits_sum += qscale2bits( rc->rce, qp2qscale(rc->rce->new_qp) );
    }

    //if( h->mb.b_variable_qp )
	if(h->rc->b_vbv)
    {
        if( rc->slice_type == SLICE_TYPE_B )
        {
            rc->bframe_bits += bits;
            if( !h->frames.current[0] || !IS_X264_TYPE_B(h->frames.current[0]->i_type) )
                update_predictor( &rc->pred_b_from_p, qp2qscale(rc->qpa_rc), h->fref1[0]->i_satd, rc->bframe_bits / rc->bframes );
        }
        else
        {
            /* Update row predictor based on data collected by other threads. */
            int y;
            for( y = rc->last_row+1; y < h->sps->i_mb_height; y++ )
                update_predictor( rc->row_pred, qp2qscale(h->fdec->i_row_qp[y]), h->fdec->i_row_satd[y], h->fdec->i_row_bits[y] );
            rc->row_preds[rc->slice_type] = *rc->row_pred;
        }
    update_vbv( h, bits );
    }


    if( rc->slice_type != SLICE_TYPE_B )
        rc->last_non_b_pict_type = rc->slice_type;
}

/****************************************************************************
 * 2 pass functions
 ***************************************************************************/

double x264_eval( char *s, double *const_value, const char **const_name,
                  double (**func1)(void *, double), const char **func1_name,
                  double (**func2)(void *, double, double), char **func2_name,
                  void *opaque );

/**
 * modify the bitrate curve from pass1 for one frame
 */
static double get_qscale(x264_t *h, ratecontrol_entry_t *rce, double rate_factor, int frame_num)
{
    x264_ratecontrol_t *rcc= h->rc;
    const int pict_type = rce->pict_type;
    double q;
    int i;
#if 0
    double const_values[]={
        rce->i_tex_bits * rce->qscale,
        rce->p_tex_bits * rce->qscale,
        (rce->i_tex_bits + rce->p_tex_bits) * rce->qscale,
        rce->mv_bits * rce->qscale,
        (double)rce->i_count / rcc->nmb,
        (double)rce->p_count / rcc->nmb,
        (double)rce->s_count / rcc->nmb,
        rce->pict_type == SLICE_TYPE_I,
        rce->pict_type == SLICE_TYPE_P,
        rce->pict_type == SLICE_TYPE_B,
        h->param.rc.f_qcompress,
        rcc->i_cplx_sum[SLICE_TYPE_I] / rcc->frame_count[SLICE_TYPE_I],
        rcc->i_cplx_sum[SLICE_TYPE_P] / rcc->frame_count[SLICE_TYPE_P],
        rcc->p_cplx_sum[SLICE_TYPE_P] / rcc->frame_count[SLICE_TYPE_P],
        rcc->p_cplx_sum[SLICE_TYPE_B] / rcc->frame_count[SLICE_TYPE_B],
        (rcc->i_cplx_sum[pict_type] + rcc->p_cplx_sum[pict_type]) / rcc->frame_count[pict_type],
        rce->blurred_complexity,
        0
    };
    static const char *const_names[]={
        "iTex",
        "pTex",
        "tex",
        "mv",
        "iCount",
        "pCount",
        "sCount",
        "isI",
        "isP",
        "isB",
        "qComp",
        "avgIITex",
        "avgPITex",
        "avgPPTex",
        "avgBPTex",
        "avgTex",
        "blurCplx",
        NULL
    };
    static double (*func1[])(void *, double)={
//      (void *)bits2qscale,
        (void *)qscale2bits,
        NULL
    };
    static const char *func1_names[]={
//      "bits2qp",
        "qp2bits",
        NULL
    };
#endif

    //q = x264_eval((char*)h->param.rc.psz_rc_eq, const_values, const_names, func1, func1_names, NULL, NULL, rce);
q = pow(rce->blurred_complexity, 1.0f - h->param.rc.f_qcompress );
    // avoid NaN's in the rc_eq
    if(!isfinite(q) || rce->i_tex_bits + rce->p_tex_bits + rce->mv_bits == 0)
        q = rcc->last_qscale_for[pict_type];
    else {
        rcc->last_rceq = q;
        q /= rate_factor;
        rcc->last_qscale = q;
    }

    for( i = rcc->i_zones-1; i >= 0; i-- )
    {
        x264_zone_t *z = &rcc->zones[i];
        if( frame_num >= z->i_start && frame_num <= z->i_end )
        {
            if( z->b_force_qp )
                q = qp2qscale(z->i_qp);
            else
                q /= z->f_bitrate_factor;
            break;
        }
    }

    return q;
}

static double get_diff_limited_q(x264_t *h, ratecontrol_entry_t *rce, double q)
{
    x264_ratecontrol_t *rcc = h->rc;
    const int pict_type = rce->pict_type;

    // force I/B quants as a function of P quants
    const double last_p_q    = rcc->last_qscale_for[SLICE_TYPE_P];
    const double last_non_b_q= rcc->last_qscale_for[rcc->last_non_b_pict_type];
    if( pict_type == SLICE_TYPE_I )
    {
        double iq = q;
        double pq = qp2qscale( rcc->accum_p_qp / rcc->accum_p_norm );
        double ip_factor = fabs( h->param.rc.f_ip_factor );
        /* don't apply ip_factor if the following frame is also I */
        if( rcc->accum_p_norm <= 0 )
            q = iq;
        else if( h->param.rc.f_ip_factor < 0 )
            q = iq / ip_factor;
        else if( rcc->accum_p_norm >= 1 )
            q = pq / ip_factor;
        else
            q = rcc->accum_p_norm * pq / ip_factor + (1 - rcc->accum_p_norm) * iq;
    }
    else if( pict_type == SLICE_TYPE_B )
    {
        if( h->param.rc.f_pb_factor > 0 )
            q = last_non_b_q;
        if( !rce->kept_as_ref )
            q *= fabs( h->param.rc.f_pb_factor );
    }
    else if( pict_type == SLICE_TYPE_P
             && rcc->last_non_b_pict_type == SLICE_TYPE_P
             && rce->i_tex_bits + rce->p_tex_bits == 0 )
    {
        q = last_p_q;
    }

    /* last qscale / qdiff stuff */
    if(rcc->last_non_b_pict_type==pict_type
       && (pict_type!=SLICE_TYPE_I || rcc->last_accum_p_norm < 1))
    {
        double last_q = rcc->last_qscale_for[pict_type];
        double max_qscale = last_q * rcc->lstep;
        double min_qscale = last_q / rcc->lstep;

        if     (q > max_qscale) q = max_qscale;
        else if(q < min_qscale) q = min_qscale;
    }

    rcc->last_qscale_for[pict_type] = q;
    if(pict_type!=SLICE_TYPE_B)
        rcc->last_non_b_pict_type = pict_type;
    if(pict_type==SLICE_TYPE_I)
    {
        rcc->last_accum_p_norm = rcc->accum_p_norm;
        rcc->accum_p_norm = 0;
        rcc->accum_p_qp = 0;
    }
    if(pict_type==SLICE_TYPE_P)
    {
        float mask = 1 - pow( (float)rce->i_count / rcc->nmb, 2 );
        rcc->accum_p_qp   = mask * (qscale2qp(q) + rcc->accum_p_qp);
        rcc->accum_p_norm = mask * (1 + rcc->accum_p_norm);
    }
    return q;
}

static double predict_size( predictor_t *p, double q, double var )
{
     return p->coeff*var / (q*p->count);
}

static void update_predictor( predictor_t *p, double q, double var, double bits )
{
    p->count *= p->decay;
    p->coeff *= p->decay;
    p->count ++;
    p->coeff += bits*q / var;
}

static void update_vbv( x264_t *h, int bits )
{
    x264_ratecontrol_t *rcc = h->rc;

    if( rcc->last_satd >= h->mb.i_mb_count )
        update_predictor( &rcc->pred[rcc->slice_type], qp2qscale(rcc->qpa_rc), rcc->last_satd, bits );

    if( !rcc->b_vbv )
        return;

    rcc->buffer_fill += rcc->buffer_rate - bits;
    if( rcc->buffer_fill < 0 && !rcc->b_2pass )
        x264_log( h, X264_LOG_WARNING, "VBV underflow (%.0f bits)\n", rcc->buffer_fill );
    rcc->buffer_fill = x264_clip3( rcc->buffer_fill, 0, rcc->buffer_size );
}

// apply VBV constraints and clip qscale to between lmin and lmax
static double clip_qscale( x264_t *h, int pict_type, double q )
{
    x264_ratecontrol_t *rcc = h->rc;
    double lmin = rcc->lmin[pict_type];
    double lmax = rcc->lmax[pict_type];
    double q0 = q;

    /* B-frames are not directly subject to VBV,
     * since they are controlled by the P-frames' QPs.
     * FIXME: in 2pass we could modify previous frames' QP too,
     *        instead of waiting for the buffer to fill */
    if( rcc->b_vbv &&
        ( pict_type == SLICE_TYPE_P ||
          ( pict_type == SLICE_TYPE_I && rcc->last_non_b_pict_type == SLICE_TYPE_I ) ) )
    {
        if( rcc->buffer_fill/rcc->buffer_size < 0.5 )
            q /= x264_clip3f( 2.0*rcc->buffer_fill/rcc->buffer_size, 0.5, 1.0 );
    }

    if( rcc->b_vbv && rcc->last_satd > 0 )
    {
        /* Now a hard threshold to make sure the frame fits in VBV.
         * This one is mostly for I-frames. */
        double bits = predict_size( &rcc->pred[rcc->slice_type], q, rcc->last_satd );
        double qf = 1.0;
        if( bits > rcc->buffer_fill/2 )
            qf = x264_clip3f( rcc->buffer_fill/(2*bits), 0.2, 1.0 );
        q /= qf;
        bits *= qf;
        if( bits < rcc->buffer_rate/2 )
            q *= bits*2/rcc->buffer_rate;
        q = X264_MAX( q0, q );

        /* Check B-frame complexity, and use up any bits that would
         * overflow before the next P-frame. */
        if( rcc->slice_type == SLICE_TYPE_P )
        {
            int nb = rcc->bframes;
            double pbbits = bits;
            double bbits = predict_size( &rcc->pred_b_from_p, q * h->param.rc.f_pb_factor, rcc->last_satd );
            double space;

            if( bbits > rcc->buffer_rate )
                nb = 0;
            pbbits += nb * bbits;

            space = rcc->buffer_fill + (1+nb)*rcc->buffer_rate - rcc->buffer_size;
            if( pbbits < space )
            {
                q *= X264_MAX( pbbits / space,
                               bits / (0.5 * rcc->buffer_size) );
            }
            q = X264_MAX( q0-5, q );
        }
    }

    if(lmin==lmax)
        return lmin;
    else if(rcc->b_2pass)
    {
        double min2 = log(lmin);
        double max2 = log(lmax);
        q = (log(q) - min2)/(max2-min2) - 0.5;
        q = 1.0/(1.0 + exp(-4*q));
        q = q*(max2-min2) + min2;
        return exp(q);
    }
    else
        return x264_clip3f(q, lmin, lmax);
}

// update qscale for 1 frame based on actual bits used so far
static float rate_estimate_qscale(x264_t *h, int pict_type)
{
    float q;
    x264_ratecontrol_t *rcc = h->rc;
    ratecontrol_entry_t rce;
    double lmin = rcc->lmin[pict_type];
    double lmax = rcc->lmax[pict_type];
    INT64 total_bits = 8*(h->stat.i_slice_size[SLICE_TYPE_I]
                          + h->stat.i_slice_size[SLICE_TYPE_P]
                          + h->stat.i_slice_size[SLICE_TYPE_B]);

    if( rcc->b_2pass )
    {
        rce = *rcc->rce;
        if(pict_type != rce.pict_type)
        {
            x264_log(h, X264_LOG_ERROR, "slice=%c but 2pass stats say %c\n",
                     slice_type_to_char[pict_type], slice_type_to_char[rce.pict_type]);
        }
    }

    if( pict_type == SLICE_TYPE_B )
    {
        /* B-frames don't have independent ratecontrol, but rather get the
         * average QP of the two adjacent P-frames + an offset */

        int i0 = IS_X264_TYPE_I(h->fref0[0]->i_type);
        int i1 = IS_X264_TYPE_I(h->fref1[0]->i_type);
        int dt0 = abs(h->fenc->i_poc - h->fref0[0]->i_poc);
        int dt1 = abs(h->fenc->i_poc - h->fref1[0]->i_poc);
        float q0 = h->fref0[0]->f_qp_avg_rc;
        float q1 = h->fref1[0]->f_qp_avg_rc;

        if( h->fref0[0]->i_type == X264_TYPE_BREF )
            q0 -= rcc->pb_offset/2;
        if( h->fref1[0]->i_type == X264_TYPE_BREF )
            q1 -= rcc->pb_offset/2;

        if(i0 && i1)
            q = (q0 + q1) / 2 + rcc->ip_offset;
        else if(i0)
            q = q1;
        else if(i1)
            q = q0;
        else
            q = (q0*dt1 + q1*dt0) / (dt0 + dt1);

        if(h->fenc->b_kept_as_ref)
            q += rcc->pb_offset/2;
        else
            q += rcc->pb_offset;

        rcc->last_satd = 0;
        return qp2qscale(q);
    }
    else
    {
        double abr_buffer = 2 * rcc->rate_tolerance * rcc->bitrate;
        if( rcc->b_2pass )
        {
            //FIXME adjust abr_buffer based on distance to the end of the video
            INT64 diff = total_bits - (INT64)rce.expected_bits;
            q = rce.new_qscale;
            q /= x264_clip3f((double)(abr_buffer - diff) / abr_buffer, .5, 2);
            if( h->fenc->i_frame > 30 )
            {
                /* Adjust quant based on the difference between
                 * achieved and expected bitrate so far */
                double time = (double)h->fenc->i_frame / rcc->num_entries;
                double w = x264_clip3f( time*100, 0.0, 1.0 );
                q *= pow( (double)total_bits / rcc->expected_bits_sum, w );
            }
            q = x264_clip3f( q, lmin, lmax );
        }
        else /* 1pass ABR */
        {
            /* Calculate the quantizer which would have produced the desired
             * average bitrate if it had been applied to all frames so far.
             * Then modulate that quant based on the current frame's complexity
             * relative to the average complexity so far (using the 2pass RCEQ).
             * Then bias the quant up or down if total size so far was far from
             * the target.
             * Result: Depending on the value of rate_tolerance, there is a
             * tradeoff between quality and bitrate precision. But at large
             * tolerances, the bit distribution approaches that of 2pass. */

            double wanted_bits, overflow, lmin, lmax;

            rcc->last_satd = x264_rc_analyse_slice( h );
            rcc->short_term_cplxsum *= 0.5;
            rcc->short_term_cplxcount *= 0.5;
            rcc->short_term_cplxsum += rcc->last_satd;
            rcc->short_term_cplxcount ++;

            rce.p_tex_bits = rcc->last_satd;
            rce.blurred_complexity = rcc->short_term_cplxsum / rcc->short_term_cplxcount;
            rce.i_tex_bits = 0;
            rce.mv_bits = 0;
            rce.p_count = rcc->nmb;
            rce.i_count = 0;
            rce.s_count = 0;
            rce.qscale = 1;
            rce.pict_type = pict_type;

            if( h->param.rc.i_rf_constant )
            {
                q = get_qscale( h, &rce, rcc->rate_factor_constant, h->fenc->i_frame );
                overflow = 1;
            }
            else
            {
                q = get_qscale( h, &rce, rcc->wanted_bits_window / rcc->cplxr_sum, h->fenc->i_frame );

                wanted_bits = h->fenc->i_frame * rcc->bitrate / rcc->fps;
                abr_buffer *= X264_MAX( 1, sqrt(h->fenc->i_frame/rcc->fps) );
                overflow = x264_clip3f( 1.0 + (total_bits - wanted_bits) / abr_buffer, .5, 2 );
                q *= overflow;
            }

            if( pict_type == SLICE_TYPE_I && h->param.i_keyint_max > 1
                /* should test _next_ pict type, but that isn't decided yet */
                && rcc->last_non_b_pict_type != SLICE_TYPE_I )
            {
                q = qp2qscale( rcc->accum_p_qp / rcc->accum_p_norm );
                q /= fabs( h->param.rc.f_ip_factor );
                q = clip_qscale( h, pict_type, q );
            }
            else
            {
				/*
                if( h->stat.i_slice_count[SLICE_TYPE_P] + h->stat.i_slice_count[SLICE_TYPE_I] < 6 )
                {
                    float w = h->stat.i_slice_count[SLICE_TYPE_P] / 5.;
                    float q2 = qp2qscale(ABR_INIT_QP);
                    q = q*w + q2*(1-w);
                }
				*/

                /* Asymmetric clipping, because symmetric would prevent
                 * overflow control in areas of rapidly oscillating complexity */
                lmin = rcc->last_qscale_for[pict_type] / rcc->lstep;
                lmax = rcc->last_qscale_for[pict_type] * rcc->lstep;
                if( overflow > 1.1 )
                    lmax *= rcc->lstep;
                else if( overflow < 0.9 )
                    lmin /= rcc->lstep;

                q = x264_clip3f(q, lmin, lmax);
                q = clip_qscale(h, pict_type, q);
                //FIXME use get_diff_limited_q() ?
            }
        }

        rcc->last_qscale_for[pict_type] =
        rcc->last_qscale = q;
        if( !(rcc->b_2pass && !rcc->b_vbv) && h->fenc->i_frame == 0 )
            rcc->last_qscale_for[SLICE_TYPE_P] = q * fabs( h->param.rc.f_ip_factor );

        rcc->frame_size_planned = predict_size( &rcc->pred[rcc->slice_type], q, rcc->last_satd );

        return q;
    }
}

/* Distribute bits among the slices, proportional to their estimated complexity */
void x264_ratecontrol_threads_start( x264_t *h )
{
    x264_ratecontrol_t *rc = h->rc;
    int t, y;
    double den = 0;
    double frame_size_planned = rc->frame_size_planned;

    for( t = 0; t < h->param.i_threads; t++ )
    {
        h->thread[t]->rc = &rc[t];
        if( t > 0 )
            rc[t] = rc[0];
    }

    if( !h->mb.b_variable_qp || rc->slice_type == SLICE_TYPE_B )
        return;

    for( t = 0; t < h->param.i_threads; t++ )
    {
        rc[t].first_row = h->thread[t]->sh.i_first_mb / h->sps->i_mb_width;
        rc[t].last_row = (h->thread[t]->sh.i_last_mb-1) / h->sps->i_mb_width;
        rc[t].frame_size_planned = 1;
        rc[t].row_pred = &rc[t].row_preds[rc->slice_type];
        if( h->param.i_threads > 1 )
        {
            for( y = rc[t].first_row; y<= rc[t].last_row; y++ )
                rc[t].frame_size_planned += predict_row_size( h, y, qscale2qp(rc[t].qp) );
        }
        den += rc[t].frame_size_planned;
    }
    for( t = 0; t < h->param.i_threads; t++ )
        rc[t].frame_size_planned *= frame_size_planned / den;
}

static int init_pass2( x264_t *h )
{
    x264_ratecontrol_t *rcc = h->rc;
    UINT64 all_const_bits = 0;
    UINT64 all_available_bits = (UINT64)(h->param.rc.i_bitrate * 1000 * (double)rcc->num_entries / rcc->fps);
    double rate_factor, step, step_mult;
    double qblur = h->param.rc.f_qblur;
    double cplxblur = h->param.rc.f_complexity_blur;
    const int filter_size = (int)(qblur*4) | 1;
    double expected_bits;
    double *qscale, *blurred_qscale;
    int i;

    /* find total/average complexity & const_bits */
    for(i=0; i<rcc->num_entries; i++){
        ratecontrol_entry_t *rce = &rcc->entry[i];
        all_const_bits += rce->misc_bits;
        rcc->i_cplx_sum[rce->pict_type] += rce->i_tex_bits * rce->qscale;
        rcc->p_cplx_sum[rce->pict_type] += rce->p_tex_bits * rce->qscale;
        rcc->mv_bits_sum[rce->pict_type] += rce->mv_bits * rce->qscale;
        rcc->frame_count[rce->pict_type] ++;
    }

    if( all_available_bits < all_const_bits)
    {
        x264_log(h, X264_LOG_ERROR, "requested bitrate is too low. estimated minimum is %d kbps\n",
                 (int)(all_const_bits * rcc->fps / (rcc->num_entries * 1000)));
        return -1;
    }

    /* Blur complexities, to reduce local fluctuation of QP.
     * We don't blur the QPs directly, because then one very simple frame
     * could drag down the QP of a nearby complex frame and give it more
     * bits than intended. */
    for(i=0; i<rcc->num_entries; i++){
        ratecontrol_entry_t *rce = &rcc->entry[i];
        double weight_sum = 0;
        double cplx_sum = 0;
        double weight = 1.0;
        int j;
        /* weighted average of cplx of future frames */
        for(j=1; j<cplxblur*2 && j<rcc->num_entries-i; j++){
            ratecontrol_entry_t *rcj = &rcc->entry[i+j];
            weight *= 1 - pow( (float)rcj->i_count / rcc->nmb, 2 );
            if(weight < .0001)
                break;
            weight_sum += weight;
            cplx_sum += weight * (qscale2bits(rcj, 1) - rcc->entry[j].misc_bits);
        }
        /* weighted average of cplx of past frames */
        weight = 1.0;
        for(j=0; j<=cplxblur*2 && j<=i; j++){
            ratecontrol_entry_t *rcj = &rcc->entry[i-j];
            weight_sum += weight;
            cplx_sum += weight * (qscale2bits(rcj, 1) - rcc->entry[j].misc_bits);
            weight *= 1 - pow( (float)rcj->i_count / rcc->nmb, 2 );
            if(weight < .0001)
                break;
        }
        rce->blurred_complexity = cplx_sum / weight_sum;
    }

    qscale = x264_malloc(sizeof(double)*rcc->num_entries);
    if(filter_size > 1)
        blurred_qscale = x264_malloc(sizeof(double)*rcc->num_entries);
    else
        blurred_qscale = qscale;

    /* Search for a factor which, when multiplied by the RCEQ values from
     * each frame, adds up to the desired total size.
     * There is no exact closed-form solution because of VBV constraints and
     * because qscale2bits is not invertible, but we can start with the simple
     * approximation of scaling the 1st pass by the ratio of bitrates.
     * The search range is probably overkill, but speed doesn't matter here. */

    expected_bits = 1;
    for(i=0; i<rcc->num_entries; i++)
        expected_bits += qscale2bits(&rcc->entry[i], get_qscale(h, &rcc->entry[i], 1.0, i));
    step_mult = all_available_bits / expected_bits;

    rate_factor = 0;
    for(step = 1E4 * step_mult; step > 1E-7 * step_mult; step *= 0.5){
        expected_bits = 0;
        rate_factor += step;

        rcc->last_non_b_pict_type = -1;
        rcc->last_accum_p_norm = 1;
        rcc->accum_p_norm = 0;
        rcc->buffer_fill = rcc->buffer_size * h->param.rc.f_vbv_buffer_init;

        /* find qscale */
        for(i=0; i<rcc->num_entries; i++){
            qscale[i] = get_qscale(h, &rcc->entry[i], rate_factor, i);
        }

        /* fixed I/B qscale relative to P */
        for(i=rcc->num_entries-1; i>=0; i--){
            qscale[i] = get_diff_limited_q(h, &rcc->entry[i], qscale[i]);
            assert(qscale[i] >= 0);
        }

        /* smooth curve */
        if(filter_size > 1){
            assert(filter_size%2==1);
            for(i=0; i<rcc->num_entries; i++){
                ratecontrol_entry_t *rce = &rcc->entry[i];
                int j;
                double q=0.0, sum=0.0;

                for(j=0; j<filter_size; j++){
                    int index = i+j-filter_size/2;
                    double d = index-i;
                    double coeff = qblur==0 ? 1.0 : exp(-d*d/(qblur*qblur));
                    if(index < 0 || index >= rcc->num_entries) continue;
                    if(rce->pict_type != rcc->entry[index].pict_type) continue;
                    q += qscale[index] * coeff;
                    sum += coeff;
                }
                blurred_qscale[i] = q/sum;
            }
        }

        /* find expected bits */
        for(i=0; i<rcc->num_entries; i++){
            ratecontrol_entry_t *rce = &rcc->entry[i];
            double bits;
            rce->new_qscale = clip_qscale(h, rce->pict_type, blurred_qscale[i]);
            assert(rce->new_qscale >= 0);
            bits = qscale2bits(rce, rce->new_qscale);

            rce->expected_bits = expected_bits;
            expected_bits += bits;
            update_vbv(h, bits);
        }

//printf("expected:%llu available:%llu factor:%lf avgQ:%lf\n", (uint64_t)expected_bits, all_available_bits, rate_factor);
        if(expected_bits > all_available_bits) rate_factor -= step;
    }

    x264_free(qscale);
    if(filter_size > 1)
        x264_free(blurred_qscale);

    if(fabs(expected_bits/all_available_bits - 1.0) > 0.01)
    {
        double avgq = 0;
        for(i=0; i<rcc->num_entries; i++)
            avgq += rcc->entry[i].new_qscale;
        avgq = qscale2qp(avgq / rcc->num_entries);

        x264_log(h, X264_LOG_ERROR, "Error: 2pass curve failed to converge\n");
        x264_log(h, X264_LOG_ERROR, "target: %.2f kbit/s, expected: %.2f kbit/s, avg QP: %.4f\n",
                 (float)h->param.rc.i_bitrate,
                 expected_bits * rcc->fps / (rcc->num_entries * 1000.),
                 avgq);
        if(expected_bits < all_available_bits && avgq < h->param.rc.i_qp_min + 2)
        {
            if(h->param.rc.i_qp_min > 0)
                x264_log(h, X264_LOG_ERROR, "try reducing target bitrate or reducing qp_min (currently %d)\n", h->param.rc.i_qp_min);
            else
                x264_log(h, X264_LOG_ERROR, "try reducing target bitrate\n");
        }
        else if(expected_bits > all_available_bits && avgq > h->param.rc.i_qp_max - 2)
        {
            if(h->param.rc.i_qp_max < 51)
                x264_log(h, X264_LOG_ERROR, "try increasing target bitrate or increasing qp_max (currently %d)\n", h->param.rc.i_qp_max);
            else
                x264_log(h, X264_LOG_ERROR, "try increasing target bitrate\n");
        }
        else
            x264_log(h, X264_LOG_ERROR, "internal error\n");
    }

    return 0;
}
static void x264_lowres_context_init( x264_t *h, x264_mb_analysis_t *a )
{
    a->i_qp = 12; // arbitrary, but low because SATD scores are 1/4 normal
    a->i_lambda = i_qp0_cost_table[ a->i_qp ];
    x264_mb_analyse_load_costs( h, a );
    h->mb.i_me_method = X264_MIN( X264_ME_HEX, h->param.analyse.i_me_method ); // maybe dia?
    h->mb.i_subpel_refine = 4; // 3 should be enough, but not tweaking for speed now
    h->mb.b_chroma_me = 0;
}

int x264_slicetype_mb_cost( x264_t *h, x264_mb_analysis_t *a,
                            x264_frame_t **frames, int p0, int p1, int b,
                            int dist_scale_factor,int *output_inter, int *output_intra )
{
    x264_frame_t *fref0 = frames[p0];
    x264_frame_t *fref1 = frames[p1];
    x264_frame_t *fenc  = frames[b];
    const int b_bidir = (b < p1);
    const int i_mb_x = h->mb.i_mb_x;
    const int i_mb_y = h->mb.i_mb_y;
    const int i_mb_stride = h->sps->i_mb_width;
    const int i_mb_xy = i_mb_x + i_mb_y * i_mb_stride;
    const int i_stride = fenc->i_stride_lowres;
    const int i_pel_offset = 8 * ( i_mb_x + i_mb_y * i_stride );
    INT16 (*fenc_mvs[2])[2] = { &fenc->lowres_mvs[0][b-p0-1][i_mb_xy], &fenc->lowres_mvs[1][p1-b-1][i_mb_xy] };
    int (*fenc_costs[2]) = { &fenc->lowres_mv_costs[0][b-p0-1][i_mb_xy], &fenc->lowres_mv_costs[1][p1-b-1][i_mb_xy] };

    int b_frame_score_mb = (i_mb_x > 0 && i_mb_x < h->sps->i_mb_width - 1 &&
                            i_mb_y > 0 && i_mb_y < h->sps->i_mb_height - 1) ||
                            h->sps->i_mb_width <= 2 || h->sps->i_mb_height <= 2;
    UINT8 pix1[9*9], pix2[8*8];
    x264_me_t m[2];
    int i_bcost = COST_MAX;
    int i_cost_bak;
    int l, i;
    int list_used = 0;
	int lowres_penalty=4;

    h->mb.pic.p_fenc[0] = h->mb.pic.fenc_buf;
    h->mc.copy[PIXEL_8x8]( h->mb.pic.p_fenc[0], FENC_STRIDE, &fenc->lowres[0][i_pel_offset], i_stride, 8 );

    //if( !p0 && !p1 && !b )
	if(p0==p1)
        goto lowres_intra_mb;

    // no need for h->mb.mv_min[]
    h->mb.mv_min_fpel[0] = -8*h->mb.i_mb_x - 4;
    h->mb.mv_max_fpel[0] = 8*( h->sps->i_mb_width - h->mb.i_mb_x - 1 ) + 4;
    h->mb.mv_min_spel[0] = 4*( h->mb.mv_min_fpel[0] - 8 );
    h->mb.mv_max_spel[0] = 4*( h->mb.mv_max_fpel[0] + 8 );
//    if( h->mb.i_mb_x <= 1 )
	if(h->mb.i_mb_x>=h->sps->i_mb_width-2)
    {
        h->mb.mv_min_fpel[1] = -8*h->mb.i_mb_y - 4;
        h->mb.mv_max_fpel[1] = 8*( h->sps->i_mb_height - h->mb.i_mb_y - 1 ) + 4;
        h->mb.mv_min_spel[1] = 4*( h->mb.mv_min_fpel[1] - 8 );
        h->mb.mv_max_spel[1] = 4*( h->mb.mv_max_fpel[1] + 8 );
    }

#define LOAD_HPELS_LUMA(dst, src) \
    { \
        (dst)[0] = &(src)[0][i_pel_offset]; \
        (dst)[1] = &(src)[1][i_pel_offset]; \
        (dst)[2] = &(src)[2][i_pel_offset]; \
        (dst)[3] = &(src)[3][i_pel_offset]; \
    }
#define SAVE_MVS( mv0, mv1 ) \
    { \
        (*fenc_mvs[0])[0] = mv0[0]; \
        (*fenc_mvs[0])[1] = mv0[1]; \
        if( b_bidir ) \
        { \
            (*fenc_mvs[1])[0] = mv1[0]; \
            (*fenc_mvs[1])[1] = mv1[1]; \
        } \
    }
#define CLIP_MV( mv ) \
    { \
        mv[0] = x264_clip3( mv[0], h->mb.mv_min_spel[0], h->mb.mv_max_spel[0] ); \
        mv[1] = x264_clip3( mv[1], h->mb.mv_min_spel[1], h->mb.mv_max_spel[1] ); \
    }
#define USE_MACRO 0
#if USE_MACRO
#define TRY_BIDIR( mv0, mv1, penalty ) \
    { \
        int stride2 = 8; \
        UINT8 *src2; \
        int i_cost; \
        h->mc.mc_luma( m[0].p_fref, m[0].i_stride[0], pix1, 8, \
                       (mv0)[0], (mv0)[1], 8, 8 ); \
        src2 = h->mc.get_ref( m[1].p_fref, m[1].i_stride[0], pix2, &stride2, \
                       (mv1)[0], (mv1)[1], 8, 8 ); \
        h->mc.avg[PIXEL_8x8]( pix1, 8, src2, stride2 ); \
        i_cost = penalty + h->pixf.mbcmp[PIXEL_8x8]( \
                           m[0].p_fenc[0], FENC_STRIDE, pix1, 8 ); \
        if( i_bcost > i_cost ) \
        { \
            i_bcost = i_cost; \
			list_used=3;\
            SAVE_MVS( mv0, mv1 ); \
        } \
    }
#endif

    m[0].i_pixel = PIXEL_8x8;
    m[0].p_cost_mv = a->p_cost_mv;
    m[0].i_stride[0] = i_stride;
    m[0].p_fenc[0] = h->mb.pic.p_fenc[0];
    LOAD_HPELS_LUMA( m[0].p_fref, fref0->lowres );

    if( b_bidir )
    {
        INT16 *mvr = fref1->lowres_mvs[0][p1-p0-1][i_mb_xy];//fref1->mv[0][i_mb_xy];
        int dmv[2][2];
        int mv0[2] = {0,0};

        m[1] = m[0];
        LOAD_HPELS_LUMA( m[1].p_fref, fref1->lowres );

        dmv[0][0] = ( mvr[0] * dist_scale_factor + 128 ) >> 8;
        dmv[0][1] = ( mvr[1] * dist_scale_factor + 128 ) >> 8;
        dmv[1][0] = dmv[0][0] - mvr[0];
        dmv[1][1] = dmv[0][1] - mvr[1];
        CLIP_MV( dmv[0] );
        CLIP_MV( dmv[1] );

#if USE_MACRO
        TRY_BIDIR( dmv[0], dmv[1], 0 );
        if( dmv[0][0] || dmv[0][1] || dmv[1][0] || dmv[1][1] )
           TRY_BIDIR( mv0, mv0, 0 );
#else

    { 
        int stride2 = 8; 
        UINT8 *src2; 
        int i_cost; 
        h->mc.mc_luma( m[0].p_fref, m[0].i_stride[0], pix1, 8, 
                       (dmv[0])[0], (dmv[0])[1], 8, 8 ); 
        src2 = h->mc.get_ref( m[1].p_fref, m[1].i_stride[0], pix2, &stride2, 
                       (dmv[1])[0], (dmv[1])[1], 8, 8 ); 
        h->mc.avg[PIXEL_8x8]( pix1, 8, src2, stride2 ); 
        i_cost = 0 + h->pixf.mbcmp[PIXEL_8x8]( 
                           m[0].p_fenc[0], FENC_STRIDE, pix1, 8 ); 
        if( i_bcost > i_cost ) 
        { 
            i_bcost = i_cost; 
			list_used=3;
            SAVE_MVS( dmv[0], dmv[1] ); 
        } 
    }
        if( dmv[0][0] || dmv[0][1] || dmv[1][0] || dmv[1][1] )
    { 
        int stride2 = 8; 
        UINT8 *src2; 
        int i_cost; 
        h->mc.mc_luma( m[0].p_fref, m[0].i_stride[0], pix1, 8, 
                       (mv0)[0], (mv0)[1], 8, 8 ); 
        src2 = h->mc.get_ref( m[1].p_fref, m[1].i_stride[0], pix2, &stride2, 
                       (mv0)[0], (mv0)[1], 8, 8 ); 
        h->mc.avg[PIXEL_8x8]( pix1, 8, src2, stride2 ); 
        i_cost = 0 + h->pixf.mbcmp[PIXEL_8x8]( 
                           m[0].p_fenc[0], FENC_STRIDE, pix1, 8 ); 
        if( i_bcost > i_cost ) 
        { 
            i_bcost = i_cost; 
			list_used=3;
            SAVE_MVS( mv0, mv0 ); 
        } 
    }
#endif
//      if( i_bcost < 60 ) // arbitrary threshold
//          return i_bcost;
    }

    i_cost_bak = i_bcost;
    for( l = 0; l < 1 + b_bidir; l++ )
    {
        int mvc[4][2] = {{0}}, i_mvc;
        INT16 (*fenc_mv)[2] = fenc_mvs[l];//&fenc->mv[l][i_mb_xy];
        i_mvc = 0;
		if( i_mb_x < h->sps->i_mb_width - 1 )// if( i_mb_x > 0 )
		{
            mvc[i_mvc][0] = fenc_mv[1][0];//fenc_mv[-1][0];
            mvc[i_mvc][1] = fenc_mv[1][0];//fenc_mv[-1][1];
            i_mvc++;
        }
		if( i_mb_y < h->sps->i_mb_height - 1 )// if( i_mb_y > 0 )
        {
            mvc[i_mvc][0] = fenc_mv[i_mb_stride][0];//fenc_mv[-i_mb_stride][0];
            mvc[i_mvc][1] = fenc_mv[i_mb_stride][1];//fenc_mv[-i_mb_stride][1];
            i_mvc++;
			if( i_mb_x > 0 )// if( i_mb_x < h->sps->i_mb_width - 1 )
            {
                mvc[i_mvc][0] = fenc_mv[i_mb_stride-1][0];//fenc_mv[-i_mb_stride+1][0];
                mvc[i_mvc][1] = fenc_mv[i_mb_stride-1][1];//fenc_mv[-i_mb_stride+1][1];
                i_mvc++;
            }
			if( i_mb_x < h->sps->i_mb_width - 1 )// if( i_mb_x > 0 )
            {
                mvc[i_mvc][0] = fenc_mv[i_mb_stride+1][0];//fenc_mv[-i_mb_stride-1][0];
                mvc[i_mvc][1] = fenc_mv[i_mb_stride+1][1];//fenc_mv[-i_mb_stride-1][1];
                i_mvc++;
            }
        }
        m[l].mvp[0] = x264_median( mvc[0][0], mvc[1][0], mvc[2][0] );
        m[l].mvp[1] = x264_median( mvc[0][1], mvc[1][1], mvc[2][1] );

        x264_me_search( h, &m[l], mvc, i_mvc );

		*fenc_costs[l]=m[l].cost;

        //i_bcost = X264_MIN( i_bcost, m[l].cost + 3 );
		if(m[l].cost+3<i_bcost){
			i_bcost=m[l].cost;
			list_used=l+1;
		}
    }

    if( b_bidir && (m[0].mv[0] || m[0].mv[1] || m[1].mv[0] || m[1].mv[1]) )
#if USE_MACRO
        TRY_BIDIR( m[0].mv, m[1].mv, 5 );
#else
    { 
        int stride2 = 8; 
        UINT8 *src2; 
        int i_cost; 
        h->mc.mc_luma( m[0].p_fref, m[0].i_stride[0], pix1, 8, 
                       (m[0].mv)[0], (m[0].mv)[1], 8, 8 ); 
        src2 = h->mc.get_ref( m[1].p_fref, m[1].i_stride[0], pix2, &stride2, 
                       (m[1].mv)[0], (m[1].mv)[1], 8, 8 ); 
        h->mc.avg[PIXEL_8x8]( pix1, 8, src2, stride2 ); 
        i_cost = 5 + h->pixf.mbcmp[PIXEL_8x8]( 
                           m[0].p_fenc[0], FENC_STRIDE, pix1, 8 ); 
        if( i_bcost > i_cost ) 
        { 
            i_bcost = i_cost; 
			list_used=3;
            SAVE_MVS( m[0].mv, m[1].mv ); 
        } 
    }
#endif

    if( i_bcost < i_cost_bak )
        SAVE_MVS( m[0].mv, m[1].mv );

lowres_intra_mb:
    if( !fenc->b_intra_calculated )
    {
        UINT8 pix_buf[9*FDEC_STRIDE];
        UINT8 *pix = &pix_buf[8+FDEC_STRIDE - 1];
        UINT8 *src = &fenc->lowres[0][i_pel_offset - 1];
        int intra_penalty = 5 + 10 * b_bidir;
		int i_icost,i_icost_aq ;
        //i_cost_bak = i_bcost;

        memcpy( pix-FDEC_STRIDE, src-i_stride, 9 );
        for( i=0; i<8; i++ )
            pix[i*FDEC_STRIDE] = src[i*i_stride];
        pix++;

		i_icost=COST_MAX;
        for( i = I_PRED_CHROMA_DC; i <= I_PRED_CHROMA_P; i++ )
        {
            int tmp_cost;
            h->predict_8x8c[i]( pix );
            tmp_cost= h->pixf.mbcmp[PIXEL_8x8]( pix, FDEC_STRIDE, h->mb.pic.p_fenc[0], FENC_STRIDE ) + intra_penalty;
            i_icost = X264_MIN( tmp_cost,i_icost );
        }
		i_icost+=lowres_penalty;
        fenc->i_intra_cost[i_mb_xy] = i_icost;
		i_icost_aq=i_icost;
    //    if( h->param.rc.i_aq_mode )
     //       i_icost_aq = (i_icost_aq * fenc->i_inv_qscale_factor[i_mb_xy] + 128) >> 8;
        if( b_frame_score_mb )
        {
            output_intra[COST_EST] += i_icost;
            output_intra[COST_EST_AQ] += i_icost_aq;
        }
		/*
        if( i_bcost != i_icost )
        {
            if( !b_bidir
                && i_mb_x > 0 && i_mb_x < h->sps->i_mb_width - 1
                && i_mb_y > 0 && i_mb_y < h->sps->i_mb_height - 1 )
            {
                fenc->i_intra_mbs[b-p0]++;
            }
            if( p1 > p0+1 )
                i_bcost = i_bcost * 9 / 8; // arbitray penalty for I-blocks in and after B-frames
        }
		*/
    }

    i_bcost += lowres_penalty;
	if( !b_bidir )
    {
        int i_icost = fenc->i_intra_cost[i_mb_xy];
        int b_intra = i_icost < i_bcost;
        if( b_intra )
        {
            i_bcost = i_icost;
            list_used = 0;
        }
        if( b_frame_score_mb )
            output_inter[INTRA_MBS] += b_intra;
    }
	if( p0 != p1 )
    {
        int i_bcost_aq = i_bcost;
        //if( h->param.rc.i_aq_mode )
         //   i_bcost_aq = (i_bcost_aq * fenc->i_inv_qscale_factor[i_mb_xy] + 128) >> 8;
        if( b_frame_score_mb )
        {
            /* Don't use AQ-weighted costs for slicetype decision, only for ratecontrol. */
            output_inter[COST_EST] += i_bcost;
            output_inter[COST_EST_AQ] += i_bcost_aq;
        }
    }

//	printf("lists_used=%d\n",list_used);
	assert(list_used==((list_used<<LOWRES_COST_SHIFT)>> LOWRES_COST_SHIFT));
	//printf("list_used=%d %d\n",list_used,((list_used<<LOWRES_COST_SHIFT)>> LOWRES_COST_SHIFT));
   fenc->lowres_costs[b-p0][p1-b][i_mb_xy] = X264_MIN( i_bcost, LOWRES_COST_MASK ) + (list_used << LOWRES_COST_SHIFT);
//	if(i_bcost!=(i_bcost&LOWRES_COST_MASK))
//		printf("i_bcost=%d %d\n",i_bcost,i_bcost&LOWRES_COST_MASK);
	//fenc->lowres_costs[b-p0][p1-b][i_mb_xy] = i_bcost;//X264_MIN( i_bcost, LOWRES_COST_MASK ) + (list_used << LOWRES_COST_SHIFT);
    return i_bcost;
}
#undef TRY_BIDIR
#undef SAVE_MVS

int x264_slicetype_frame_cost( x264_t *h, x264_mb_analysis_t *a,
                               x264_frame_t **frames, int p0, int p1, int b )
{
    int i_score = 0;
    int dist_scale_factor = 128;
	int output_inter[NUM_ROWS+1]={0}, output_intra[NUM_ROWS+1]={0};
    x264_frame_t *fenc = frames[b];
    /* Check whether we already evaluated this frame
     * If we have tried this frame as P, then we have also tried
     * the preceding frames as B. (is this still true?) */
    if( fenc->i_cost_est[b-p0][p1-b] >= 0 )
	{
#if 1
	printf("pre:fenc->i_cost_est[%d][%d]=%d:[p0,b,p1]=[%d,%d,%d:%d,%d,%d]\n",
		b-p0,p1-b,fenc->i_cost_est[b-p0][p1-b],p0,b,p1,frames[p0]->i_frame,frames[b]->i_frame,frames[p1]->i_frame);
#endif
        return fenc->i_cost_est[b-p0][p1-b];
	}

    /* Init MVs so that we don't have to check edge conditions when loading predictors. */
    /* FIXME: not needed every time */
    memset( fenc->mv[0], 0, h->sps->i_mb_height * h->sps->i_mb_width * 2*sizeof(INT16) );
    if( b != p1 )
        memset( fenc->mv[1], 0, h->sps->i_mb_height * h->sps->i_mb_width * 2*sizeof(INT16) );

    if( b == p1 )
        fenc->i_intra_mbs[b-p0] = 0;
    if( p1 != p0 )
        dist_scale_factor = ( ((b-p0) << 8) + ((p1-p0) >> 1) ) / (p1-p0);

	output_inter[NUM_ROWS] = output_intra[NUM_ROWS] = h->sps->i_mb_height;
    /* the edge mbs seem to reduce the predictive quality of the
     * whole frame's score, but are needed for a spatial distribution. */
    if( h->param.rc.i_vbv_buffer_size )
    {
		int *row_satd = fenc->i_row_satds[b-p0][p1-b];
        for( h->mb.i_mb_y = 0; h->mb.i_mb_y < h->sps->i_mb_height; h->mb.i_mb_y++ )
        {
            row_satd[ h->mb.i_mb_y ] = 0;
            for( h->mb.i_mb_x = 0; h->mb.i_mb_x < h->sps->i_mb_width; h->mb.i_mb_x++ )
            {
                int i_mb_cost = x264_slicetype_mb_cost( h, a, frames, p0, p1, b, dist_scale_factor ,output_inter, output_intra);
                row_satd[ h->mb.i_mb_y ] += i_mb_cost;
                if( h->mb.i_mb_y > 0 && h->mb.i_mb_y < h->sps->i_mb_height - 1 &&
                    h->mb.i_mb_x > 0 && h->mb.i_mb_x < h->sps->i_mb_width - 1 )
                {
                    i_score += i_mb_cost;
                }
            }
        }
    }
    else
    {
		int do_edges = h->param.rc.b_mb_tree|| h->sps->i_mb_width <= 2 || h->sps->i_mb_height <= 2;
    int start_y = h->sps->i_mb_height - 2 + do_edges ;
    int end_y = 1 - do_edges ;
    int start_x = h->sps->i_mb_width - 2 + do_edges;
    int end_x = 1 - do_edges;
        for( h->mb.i_mb_y = start_y; h->mb.i_mb_y >=end_y ; h->mb.i_mb_y-- )
            for( h->mb.i_mb_x = start_x; h->mb.i_mb_x >=end_x ; h->mb.i_mb_x-- )
                i_score += x264_slicetype_mb_cost( h, a, frames, p0, p1, b, dist_scale_factor,output_inter, output_intra );
    }
	/* Sum up accumulators */
	if( b == p1 )
		fenc->i_intra_mbs[b-p0] = 0;
	if( !fenc->b_intra_calculated )
	{
		fenc->i_cost_est[0][0] = 0;
		fenc->i_cost_est_aq[0][0] = 0;
	}
	fenc->i_cost_est[b-p0][p1-b] = 0;
	fenc->i_cost_est_aq[b-p0][p1-b] = 0;

	if( b == p1 )
		fenc->i_intra_mbs[b-p0] += output_inter[INTRA_MBS];
	if( !fenc->b_intra_calculated )
	{
		fenc->i_cost_est[0][0] += output_intra[COST_EST];
		fenc->i_cost_est_aq[0][0] += output_intra[COST_EST_AQ];
	}

	fenc->i_cost_est[b-p0][p1-b] += output_inter[COST_EST];
	fenc->i_cost_est_aq[b-p0][p1-b] += output_inter[COST_EST_AQ];

	i_score = fenc->i_cost_est[b-p0][p1-b];
	if( b != p1 )
		i_score = i_score * 100 / (120 + h->param.i_bframe_bias);
	else
		fenc->b_intra_calculated = 1;

	fenc->i_cost_est[b-p0][p1-b] = i_score;

#if 1
	printf("1st:fenc->i_cost_est[%d][%d]=%d:[p0,b,p1]=[%d,%d,%d:%d,%d,%d]\n",
		b-p0,p1-b,fenc->i_cost_est[b-p0][p1-b],p0,b,p1,frames[p0]->i_frame,frames[b]->i_frame,frames[p1]->i_frame);
#endif
//	if( b != p1 )
//		i_score = i_score * 100 / (120 + h->param.i_bframe_bias);
  //  fenc->i_cost_est[b-p0][p1-b] = i_score;
//  fprintf( stderr, "frm %d %c(%d,%d): %6d I:%d  \n", fenc->i_frame,
//           (p1==0?'I':b<p1?'B':'P'), b-p0, p1-b, i_score, fenc->i_intra_mbs[b-p0] );
    //x264_cpu_restore( h->param.cpu );
    return i_score;
}
static void mbtree_propagate_cost( int *dst, UINT16 *propagate_in, UINT16 *intra_costs,
                                   UINT16 *inter_costs, UINT16 *inv_qscales, float *fps_factor, int len )
{
    float fps = *fps_factor / 256.f;
	int i;
    for( i = 0; i < len; i++ )
    {
       // float intra_cost       = intra_costs[i] * inv_qscales[i];
        float propagate_amount = propagate_in[i] + intra_costs[i];//intra_cost*fps;
        float propagate_num    = intra_costs[i] - (inter_costs[i] & LOWRES_COST_MASK);
        float propagate_denom  = intra_costs[i];
        dst[i] = (int)(propagate_amount * propagate_num / propagate_denom + 0.5f);
    }
}
static void x264_macroblock_tree_propagate( x264_t *h, x264_frame_t **frames, int p0, int p1, int b, int referenced )
{

    UINT16 *ref_costs[2] = {frames[p0]->i_propagate_cost,frames[p1]->i_propagate_cost};
    int dist_scale_factor = ( ((b-p0) << 8) + ((p1-p0) >> 1) ) / (p1-p0);
    int i_bipred_weight = h->param.analyse.b_weighted_bipred ? 64 - (dist_scale_factor>>2) : 32;
    INT16 (*mvs[2])[2] = { frames[b]->lowres_mvs[0][b-p0-1], frames[b]->lowres_mvs[1][p1-b-1] };
    int bipred_weights[2] = {i_bipred_weight, 64 - i_bipred_weight};
    int *buf = x264_malloc(h->sps->i_mb_width*sizeof(int));//(int*) h->scratch_buffer;
    UINT16 *propagate_cost = frames[b]->i_propagate_cost;

    float fps_factor =1.0;// CLIP_DURATION(frames[b]->f_duration) / CLIP_DURATION(average_duration);

	memset(buf,0,h->sps->i_mb_width*sizeof(int));

    /* For non-reffed frames the source costs are always zero, so just memset one row and re-use it. */
    if( !referenced )
        memset( frames[b]->i_propagate_cost, 0, h->sps->i_mb_width * sizeof(UINT16) );

    for( h->mb.i_mb_y = 0; h->mb.i_mb_y < h->sps->i_mb_height; h->mb.i_mb_y++ )
	{
        int mb_index = h->mb.i_mb_y*h->mb.i_mb_stride;
		mbtree_propagate_cost( buf, propagate_cost,
            frames[b]->i_intra_cost+mb_index, frames[b]->lowres_costs[b-p0][p1-b]+mb_index,
            frames[b]->i_inv_qscale_factor+mb_index, &fps_factor, h->sps->i_mb_width );
        if( referenced )
            propagate_cost += h->sps->i_mb_width;

        for( h->mb.i_mb_x = 0; h->mb.i_mb_x < h->sps->i_mb_width; h->mb.i_mb_x++, mb_index++ )
		{
            int propagate_amount = buf[h->mb.i_mb_x];
            if( propagate_amount > 0 )
			{
                int lists_used = frames[b]->lowres_costs[b-p0][p1-b][mb_index] >> LOWRES_COST_SHIFT;
				int list;
				//printf("lists_used=%d\n",lists_used);
                for( list = 0; list < 2; list++ )
                    if( (lists_used >> list)&1 )
					{
#define CLIP_ADD(s,x) (s) = X264_MIN((s)+(x),(1<<16)-1)
                        int listamount = propagate_amount;
                        if( lists_used == 3 )
                            listamount = (listamount * bipred_weights[list] + 32) >> 6;
						if( !*(int*)( mvs[list][mb_index] ) )
                        {
                            CLIP_ADD( ref_costs[list][mb_index], listamount );
                            continue;
                        }
						{
						int x = mvs[list][mb_index][0];
                        int y = mvs[list][mb_index][1];
                        int mbx = (x>>5)+h->mb.i_mb_x;
                        int mby = (y>>5)+h->mb.i_mb_y;
                        int idx0 = mbx + mby * h->mb.i_mb_stride;
                        int idx1 = idx0 + 1;
                        int idx2 = idx0 + h->mb.i_mb_stride;
                        int idx3 = idx0 + h->mb.i_mb_stride + 1;
						int idx0weight,idx1weight,idx2weight,idx3weight;
                        x &= 31;
                        y &= 31;
                        idx0weight = (32-y)*(32-x);
                        idx1weight = (32-y)*x;
                        idx2weight = y*(32-x);
                        idx3weight = y*x;
                        if( mbx < h->sps->i_mb_width-1 && mby < h->sps->i_mb_height-1 && mbx >= 0 && mby >= 0 )
                        {
                            CLIP_ADD( ref_costs[list][idx0], (listamount*idx0weight+512)>>10 );
                            CLIP_ADD( ref_costs[list][idx1], (listamount*idx1weight+512)>>10 );
                            CLIP_ADD( ref_costs[list][idx2], (listamount*idx2weight+512)>>10 );
                            CLIP_ADD( ref_costs[list][idx3], (listamount*idx3weight+512)>>10 );
                        }
                        else /* Check offsets individually */
                        {
                            if( mbx < h->sps->i_mb_width && mby < h->sps->i_mb_height && mbx >= 0 && mby >= 0 )
                                CLIP_ADD( ref_costs[list][idx0], (listamount*idx0weight+512)>>10 );
                            if( mbx+1 < h->sps->i_mb_width && mby < h->sps->i_mb_height && mbx+1 >= 0 && mby >= 0 )
                                CLIP_ADD( ref_costs[list][idx1], (listamount*idx1weight+512)>>10 );
                            if( mbx < h->sps->i_mb_width && mby+1 < h->sps->i_mb_height && mbx >= 0 && mby+1 >= 0 )
                                CLIP_ADD( ref_costs[list][idx2], (listamount*idx2weight+512)>>10 );
                            if( mbx+1 < h->sps->i_mb_width && mby+1 < h->sps->i_mb_height && mbx+1 >= 0 && mby+1 >= 0 )
                                CLIP_ADD( ref_costs[list][idx3], (listamount*idx3weight+512)>>10 );
                        }
						}
					}
			}


		}
	}
	x264_free(buf);
}

static void x264_macroblock_tree_finish( x264_t *h, x264_frame_t *frame, int ref0_distance )
{
   int fps_factor = 1;//floor( CLIP_DURATION(average_duration) / CLIP_DURATION(frame->f_duration) * 256 +0.5);
    float weightdelta = 0.0;
	int mb_index;
  //  if( ref0_distance && frame->f_weighted_cost_delta[ref0_distance-1] > 0 )
   //     weightdelta = (1.0 - frame->f_weighted_cost_delta[ref0_distance-1]);

    /* Allow the strength to be adjusted via qcompress, since the two
     * concepts are very similar. */
    float strength = 5.0f * (1.0f - h->param.rc.f_qcompress);
    for( mb_index = 0; mb_index < h->mb.i_mb_count; mb_index++ )
    {
     //   int intra_cost = (frame->i_intra_cost[mb_index] * frame->i_inv_qscale_factor[mb_index] + 128) >> 8;
		int intra_cost=frame->i_intra_cost[mb_index];
        if( intra_cost )
        {
//            int propagate_cost = (frame->i_propagate_cost[mb_index] * fps_factor + 128) >> 8;
			int propagate_cost=frame->i_propagate_cost[mb_index];
            float log2_ratio = (log(intra_cost*1.0 + propagate_cost) - log(intra_cost*1.0))/log(2.0) + weightdelta;
            frame->f_qp_offset[mb_index] = frame->f_qp_offset_aq[mb_index] - strength * log2_ratio;
        }
    }
}
static void x264_macroblock_tree( x264_t *h, x264_mb_analysis_t *a, x264_frame_t **frames, int num_frames, int b_intra )
{

    int idx = !b_intra;
    int last_nonb, cur_nonb = 1;
    int bframes = 0;

    int i = num_frames;
    if( b_intra )
	{
       x264_slicetype_frame_cost( h, a, frames, 0, 0, 0);
	//	return;
	}
    while( i > 0 && frames[i]->i_type == X264_TYPE_B )
        i--;
    last_nonb = i;
    if( !h->param.rc.i_lookahead )
    {
        if( b_intra )
        {
            memset( frames[0]->i_propagate_cost, 0, h->mb.i_mb_count * sizeof(UINT16) );
            memcpy( frames[0]->f_qp_offset, frames[0]->f_qp_offset_aq, h->mb.i_mb_count * sizeof(float) );
            return;
        }
        XCHG( UINT16*, frames[last_nonb]->i_propagate_cost, frames[0]->i_propagate_cost );
        memset( frames[0]->i_propagate_cost, 0, h->mb.i_mb_count * sizeof(UINT16) );
    }
    else
    {
        if( last_nonb < idx )
            return;
        memset( frames[last_nonb]->i_propagate_cost, 0, h->mb.i_mb_count * sizeof(UINT16) );
    }

    while( i-- > idx )
	{
        cur_nonb = i;
        while( frames[cur_nonb]->i_type == X264_TYPE_B && cur_nonb > 0 )
            cur_nonb--;
        if( cur_nonb < idx )
            break;
        x264_slicetype_frame_cost( h, a, frames, cur_nonb, last_nonb, last_nonb);
        memset( frames[cur_nonb]->i_propagate_cost, 0, h->mb.i_mb_count * sizeof(UINT16) );
        bframes = last_nonb - cur_nonb - 1;

            while( i > cur_nonb )
            {
                x264_slicetype_frame_cost( h, a, frames, cur_nonb, last_nonb, i );
                x264_macroblock_tree_propagate( h, frames, cur_nonb, last_nonb, i, 0 );
                i--;
            }
        x264_macroblock_tree_propagate( h, frames, cur_nonb, last_nonb, last_nonb, 1 );
        last_nonb = cur_nonb;

	}

    if( !h->param.rc.i_lookahead )
    {
        x264_slicetype_frame_cost( h, a, frames, 0, last_nonb, last_nonb);
        x264_macroblock_tree_propagate( h, frames, 0, last_nonb, last_nonb, 1 );
        XCHG( UINT16*, frames[last_nonb]->i_propagate_cost, frames[0]->i_propagate_cost );
    }

    x264_macroblock_tree_finish( h, frames[last_nonb], last_nonb );
}

void x264_slicetype_analyse( x264_t *h ,int keyframe)
{
    x264_mb_analysis_t a;
   // x264_frame_t *frames[X264_BFRAME_MAX+3] = { NULL, };
	x264_frame_t *frames[X264_LOOKAHEAD_MAX+3]={NULL,};
    int num_frames,reset_start;
    int keyint_limit;
    int j,framecnt;
    int i_mb_count = (h->sps->i_mb_width - 2) * (h->sps->i_mb_height - 2);
    int cost1p0, cost2p0, cost1b1, cost2p1;
    int num_bframes = 0;
    int num_analysed_frames ;

    if( !h->frames.last_nonb )
        return;
    frames[0] = h->frames.last_nonb;
    for( framecnt = 0; h->frames.next[framecnt]; framecnt++ )
        frames[framecnt+1] = h->frames.next[framecnt];

    x264_lowres_context_init( h, &a );
    if( !framecnt )
    {
        if( h->param.rc.b_mb_tree )
            x264_macroblock_tree( h, &a, frames, 0, keyframe );
        return;
    }

    keyint_limit = h->param.i_keyint_max - frames[0]->i_frame + h->frames.i_last_idr - 1;
    num_frames = X264_MIN( framecnt, keyint_limit );
    if( num_frames == 0 )
	{
       // frames[1]->i_type = X264_TYPE_I;
		return;

	}
	/*
    if( num_frames == 1 )
    {
no_b_frames:
        frames[1]->i_type = X264_TYPE_P;
        return;
    }
	*/
	num_analysed_frames=num_frames;

	if(h->param.i_bframe){
		if(h->param.i_bframe_adaptive == X264_B_ADAPT_TRELLIS){

		}
		else if(h->param.i_bframe_adaptive == X264_B_ADAPT_FAST){
			int i;
			for(i=0;i<=num_frames-2;i++){
    cost2p1 = x264_slicetype_frame_cost( h, &a, frames, i+0, i+2, i+2 );

                if( frames[i+2]->i_intra_mbs[2] > i_mb_count / 2 )
                {
                    frames[i+1]->i_type = X264_TYPE_P;
                    frames[i+2]->i_type = X264_TYPE_P;
                    i += 2;
                    continue;
                }
    cost1b1 = x264_slicetype_frame_cost( h, &a, frames, i+0, i+2, i+1 );
    cost1p0 = x264_slicetype_frame_cost( h, &a, frames, i+0, i+1, i+1 );
    cost2p0 = x264_slicetype_frame_cost( h, &a, frames, i+1, i+2, i+2 );

                if( cost1p0 + cost2p0 < cost1b1 + cost2p1 )
                {
                    frames[i+1]->i_type = X264_TYPE_P;
                    i += 1;
                    continue;
                }

                // arbitrary and untuned
                #define INTER_THRESH 300 //??
                #define P_SENS_BIAS (50 - h->param.i_bframe_bias) //??
                frames[i+1]->i_type = X264_TYPE_B;

                for( j = i+2; j <= X264_MIN( i+h->param.i_bframe, num_frames-1 ); j++ )
                {
                    int pthresh = X264_MAX(INTER_THRESH - P_SENS_BIAS * (j-i-1), INTER_THRESH/10);
                    int pcost = x264_slicetype_frame_cost( h, &a, frames, i+0, j+1, j+1);
                    if( pcost > pthresh*i_mb_count || frames[j+1]->i_intra_mbs[j-i+1] > i_mb_count/3 )
                        break;
                    frames[j]->i_type = X264_TYPE_B;
                }
                frames[j]->i_type = X264_TYPE_P;
                i = j;
			}
   // cost2p1 = x264_slicetype_frame_cost( h, &a, frames, 0, 2, 2 );
   // if( frames[2]->i_intra_mbs[2] > i_mb_count / 2 )
   //     goto no_b_frames;

   // cost1b1 = x264_slicetype_frame_cost( h, &a, frames, 0, 2, 1 );
   // cost1p0 = x264_slicetype_frame_cost( h, &a, frames, 0, 1, 1 );
   // cost2p0 = x264_slicetype_frame_cost( h, &a, frames, 1, 2, 2 );
//  fprintf( stderr, "PP: %d + %d <=> BP: %d + %d \n",
//           cost1p0, cost2p0, cost1b1, cost2p1 );
//    if( cost1p0 + cost2p0 < cost1b1 + cost2p1 )
//        goto no_b_frames;

// arbitrary and untuned
//#define INTER_THRESH 300
//#define P_SENS_BIAS (50 - h->param.i_bframe_bias)
//    frames[1]->i_type = X264_TYPE_B;

//    for( j = 2; j <= X264_MIN( h->param.i_bframe, num_frames-1 ); j++ )
//    {
//        int pthresh = X264_MAX(INTER_THRESH - P_SENS_BIAS * (j-1), INTER_THRESH/10);
//        int pcost = x264_slicetype_frame_cost( h, &a, frames, 0, j+1, j+1 );
//      fprintf( stderr, "frm%d+%d: %d <=> %d, I:%d/%d \n",
//               frames[0]->i_frame, j-1, pthresh, pcost/i_mb_count,
//               frames[j+1]->i_intra_mbs[j+1], i_mb_count );
//        if( pcost > pthresh*i_mb_count || frames[j+1]->i_intra_mbs[j+1] > i_mb_count/3 )
//        {
//            frames[j]->i_type = X264_TYPE_P;
//            break;
//        }
 //       else
//            frames[j]->i_type = X264_TYPE_B;
//    }
		}
		else{
            num_bframes = X264_MIN(num_frames-1, h->param.i_bframe);
            for( j = 1; j < num_frames; j++ )
                frames[j]->i_type = (j%(num_bframes+1)) ? X264_TYPE_B : X264_TYPE_P;
            frames[num_frames]->i_type = X264_TYPE_P;
		}
        reset_start = keyframe ? 1 : X264_MIN( num_bframes+2, num_analysed_frames+1 );

	}
   else
    {
        for( j = 1; j <= num_frames; j++ )
            frames[j]->i_type = X264_TYPE_P;
        reset_start = !keyframe + 1;
        num_bframes = 0;
    }

    if( h->param.rc.b_mb_tree )
        x264_macroblock_tree( h, &a, frames, X264_MIN(num_frames, h->param.i_keyint_max), keyframe);
    /* Restore frametypes for all frames that haven't actually been decided yet. */
    for( j = reset_start; j <= num_frames; j++ )
        frames[j]->i_type = X264_TYPE_AUTO;
}

void x264_slicetype_decide( x264_t *h )
{
    x264_frame_t *frm;
    int bframes;
    int i;

    if( h->frames.next[0] == NULL )
        return;

    if( h->param.rc.b_stat_read )
    {
        /* Use the frame types from the first pass */
        for( i = 0; h->frames.next[i] != NULL; i++ )
            h->frames.next[i]->i_type =
                x264_ratecontrol_slice_type( h, h->frames.next[i]->i_frame );
    }
    else if( (h->param.i_bframe && h->param.i_bframe_adaptive)
		||h->param.rc.b_mb_tree)
        x264_slicetype_analyse( h ,0);

    for( bframes = 0;; bframes++ )
    {
        frm = h->frames.next[bframes];

        /* Limit GOP size */
        if( frm->i_frame - h->frames.i_last_idr >= h->param.i_keyint_max )
        {
            if( frm->i_type == X264_TYPE_AUTO )
                frm->i_type = X264_TYPE_IDR;
            if( frm->i_type != X264_TYPE_IDR )
                x264_log( h, X264_LOG_ERROR, "specified frame type (%d) is not compatible with keyframe interval\n", frm->i_type );
        }
        if( frm->i_type == X264_TYPE_IDR )
        {
            /* Close GOP */
            if( bframes > 0 )
            {
                bframes--;
                h->frames.next[bframes]->i_type = X264_TYPE_P;
            }
            else
            {
                h->i_frame_num = 0;
            }
        }

        if( bframes == h->param.i_bframe
            || h->frames.next[bframes+1] == NULL )
        {
            if( IS_X264_TYPE_B( frm->i_type ) )
                x264_log( h, X264_LOG_ERROR, "specified frame type is not compatible with max B-frames\n" );
            if( frm->i_type == X264_TYPE_AUTO
                || IS_X264_TYPE_B( frm->i_type ) )
                frm->i_type = X264_TYPE_P;
        }

        if( frm->i_type != X264_TYPE_AUTO && frm->i_type != X264_TYPE_B && frm->i_type != X264_TYPE_BREF )
            break;

        frm->i_type = X264_TYPE_B;
    }
}

int x264_rc_analyse_slice( x264_t *h )
{
    x264_mb_analysis_t a;
    x264_frame_t *frames[X264_BFRAME_MAX+2] = { NULL, };
    int p0=0, p1, b;
    int cost;

    x264_lowres_context_init( h, &a );

    if( IS_X264_TYPE_I(h->fenc->i_type) )
    {
        p1 = b = 0;
    }
    else if( X264_TYPE_P == h->fenc->i_type )
    {
        p1 = 0;
        while( h->frames.current[p1] && IS_X264_TYPE_B( h->frames.current[p1]->i_type ) )
            p1++;
        p1++;
        b = p1;
    }
    else //B
    {
        p1 = (h->fref1[0]->i_poc - h->fref0[0]->i_poc)/2;
        b  = (h->fref1[0]->i_poc - h->fenc->i_poc)/2;
        frames[p1] = h->fref1[0];
    }
    frames[p0] = h->fref0[0];
    frames[b] = h->fenc;

    cost = x264_slicetype_frame_cost( h, &a, frames, p0, p1, b );
    h->fenc->i_row_satd = h->fenc->i_row_satds[b-p0][p1-b];
    h->fdec->i_row_satd = h->fdec->i_row_satds[b-p0][p1-b];
    h->fdec->i_satd = cost;
    memcpy( h->fdec->i_row_satd, h->fenc->i_row_satd, h->sps->i_mb_height * sizeof(int) );
    return cost;
}

