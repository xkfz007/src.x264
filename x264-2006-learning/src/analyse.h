/*****************************************************************************
 * analyse.h: h264 encoder library
 *****************************************************************************
 * Copyright (C) 2003 Laurent Aimar
 * $Id: analyse.h,v 1.1 2004/06/03 19:27:08 fenrir Exp $
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
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

#ifndef _ANALYSE_H
#define _ANALYSE_H 1
#include "me.h"
typedef struct
{
    /* 16x16 */
    int i_ref;
    x264_me_t me16x16;

    /* 8x8 */
    int       i_cost8x8;
    int       mvc[16][5][2]; /* [ref][0] is 16x16 mv,
                                [ref][1..4] are 8x8 mv from partition [0..3] */
    x264_me_t me8x8[4];

    /* Sub 4x4 */
    int       i_cost4x4[4]; /* cost per 8x8 partition */
    x264_me_t me4x4[4][4];

    /* Sub 8x4 */
    int       i_cost8x4[4]; /* cost per 8x8 partition */
    x264_me_t me8x4[4][2];

    /* Sub 4x8 */
    int       i_cost4x8[4]; /* cost per 8x8 partition */
    x264_me_t me4x8[4][4];

    /* 16x8 */
    int       i_cost16x8;
    x264_me_t me16x8[2];

    /* 8x16 */
    int       i_cost8x16;
    x264_me_t me8x16[2];

} x264_mb_analysis_list_t;

typedef struct
{
    /* conduct the analysis using this lamda and QP */
    int i_lambda;
    int i_lambda2;
    int i_qp;
    INT16 *p_cost_mv;
    int b_mbrd;


    /* I: Intra part */
    /* Take some shortcuts in intra search if intra is deemed unlikely */
    int b_fast_intra;
    int i_best_satd;
    int b_try_pskip;

    /* Luma part */
    int i_sad_i16x16;
    int i_predict16x16;

    int i_sad_i8x8;
    int i_predict8x8[2][2];

    int i_sad_i4x4;
    int i_predict4x4[4][4];

    /* Chroma part */
    int i_sad_i8x8chroma;
    int i_predict8x8chroma;

    /* II: Inter part P/B frame */
    x264_mb_analysis_list_t l0;
    x264_mb_analysis_list_t l1;

    int i_cost16x16bi; /* used the same ref and mv as l0 and l1 (at least for now) */
    int i_cost16x16direct;
    int i_cost8x8bi;
    int i_cost8x8direct[4];
    int i_cost16x8bi;
    int i_cost8x16bi;

    int i_mb_partition16x8[2]; /* mb_partition_e */
    int i_mb_partition8x16[2];
    int i_mb_type16x8; /* mb_class_e */
    int i_mb_type8x16;

    int b_direct_available;

} x264_mb_analysis_t;
/* lambda = pow(2,qp/6-2) */
static const int i_qp0_cost_table[52] = {
   1, 1, 1, 1, 1, 1, 1, 1,  /*  0-7 */
   1, 1, 1, 1,              /*  8-11 */
   1, 1, 1, 1, 2, 2, 2, 2,  /* 12-19 */
   3, 3, 3, 4, 4, 4, 5, 6,  /* 20-27 */
   6, 7, 8, 9,10,11,13,14,  /* 28-35 */
  16,18,20,23,25,29,32,36,  /* 36-43 */
  40,45,51,57,64,72,81,91   /* 44-51 */
};

/* pow(lambda,2) * .9 */
static const int i_qp0_cost2_table[52] = {
   1,   1,   1,   1,   1,   1, /*  0-5  */
   1,   1,   1,   1,   1,   1, /*  6-11 */
   1,   1,   1,   2,   2,   3, /* 12-17 */
   4,   5,   6,   7,   9,  11, /* 18-23 */
  14,  18,  23,  29,  36,  46, /* 24-29 */
  58,  73,  91, 115, 145, 183, /* 30-35 */
 230, 290, 366, 461, 581, 731, /* 36-41 */
 922,1161,1463,1843,2322,2926, /* 42-47 */
3686,4645,5852,7373
};

/* TODO: calculate CABAC costs */
static const int i_mb_b_cost_table[19] = {
    9, 9, 9, 9, 0, 0, 0, 1, 3, 7, 7, 7, 3, 7, 7, 7, 5, 9, 0
};
static const int i_mb_b16x8_cost_table[17] = {
    0, 0, 0, 0, 0, 0, 0, 0, 5, 7, 7, 7, 5, 7, 9, 9, 9
};
static const int i_sub_mb_b_cost_table[13] = {
    7, 5, 5, 3, 7, 5, 7, 3, 7, 7, 7, 5, 1
};
static const int i_sub_mb_p_cost_table[4] = {
    5, 3, 3, 1
};
void x264_macroblock_analyse( x264_t *h );
void x264_slicetype_decide( x264_t *h );
void x264_slicetype_analyse( x264_t *h,int keyframe );
void x264_mb_analyse_load_costs( x264_t *h, x264_mb_analysis_t *a );
#endif
