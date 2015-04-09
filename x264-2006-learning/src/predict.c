/*****************************************************************************
 * predict.c: h264 encoder
 *****************************************************************************
 * Copyright (C) 2003 Laurent Aimar
 * $Id: predict.c,v 1.1 2004/06/03 19:27:07 fenrir Exp $
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
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

/* predict4x4 are inspired from ffmpeg h264 decoder */


#include "common.h"
//#include "clip1.h"



/****************************************************************************
 * 16x16 prediction for intra luma block
 ****************************************************************************/

#define PREDICT_16x16_DC(v) \
    for( i = 0; i < 16; i++ )\
    {\
        UINT32 *p = (UINT32*)src;\
        *p++ = v;\
        *p++ = v;\
        *p++ = v;\
        *p++ = v;\
        src += FDEC_STRIDE;\
    }

static void predict_16x16_dc( UINT8 *src )
{
    UINT32 dc = 0;
    int i;

    for( i = 0; i < 16; i++ )
    {
        dc += src[-1 + i * FDEC_STRIDE];
        dc += src[i - FDEC_STRIDE];
    }
    dc = (( dc + 16 ) >> 5) * 0x01010101;

    PREDICT_16x16_DC(dc);
}
static void predict_16x16_dc_left( UINT8 *src )
{
    UINT32 dc = 0;
    int i;

    for( i = 0; i < 16; i++ )
    {
        dc += src[-1 + i * FDEC_STRIDE];
    }
    dc = (( dc + 8 ) >> 4) * 0x01010101;

    PREDICT_16x16_DC(dc);
}
static void predict_16x16_dc_top( UINT8 *src )
{
    UINT32 dc = 0;
    int i;

    for( i = 0; i < 16; i++ )
    {
        dc += src[i - FDEC_STRIDE];
    }
    dc = (( dc + 8 ) >> 4) * 0x01010101;

    PREDICT_16x16_DC(dc);
}
static void predict_16x16_dc_128( UINT8 *src )
{
    int i;
    PREDICT_16x16_DC(0x80808080);
}
static void predict_16x16_h( UINT8 *src )
{
    int i;

    for( i = 0; i < 16; i++ )
    {
        const UINT32 v = 0x01010101 * src[-1];
        UINT32 *p = (UINT32*)src;

        *p++ = v;
        *p++ = v;
        *p++ = v;
        *p++ = v;

        src += FDEC_STRIDE;

    }
}
static void predict_16x16_v( UINT8 *src )
{
    UINT32 v0 = *(UINT32*)&src[ 0-FDEC_STRIDE];
    UINT32 v1 = *(UINT32*)&src[ 4-FDEC_STRIDE];
    UINT32 v2 = *(UINT32*)&src[ 8-FDEC_STRIDE];
    UINT32 v3 = *(UINT32*)&src[12-FDEC_STRIDE];
    int i;

    for( i = 0; i < 16; i++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = v0;
        *p++ = v1;
        *p++ = v2;
        *p++ = v3;
        src += FDEC_STRIDE;
    }
}
static void predict_16x16_p( UINT8 *src )
{
    int x, y, i;
    int a, b, c;
    int H = 0;
    int V = 0;
    int i00;

    /* calcule H and V */
    for( i = 0; i <= 7; i++ )
    {
        H += ( i + 1 ) * ( src[ 8 + i - FDEC_STRIDE ] - src[6 -i -FDEC_STRIDE] );
        V += ( i + 1 ) * ( src[-1 + (8+i)*FDEC_STRIDE] - src[-1 + (6-i)*FDEC_STRIDE] );
    }

    a = 16 * ( src[-1 + 15*FDEC_STRIDE] + src[15 - FDEC_STRIDE] );
    b = ( 5 * H + 32 ) >> 6;
    c = ( 5 * V + 32 ) >> 6;

    i00 = a - b * 7 - c * 7 + 16;

    for( y = 0; y < 16; y++ )
    {
        int pix = i00;
        for( x = 0; x < 16; x++ )
        {
            src[x] = x264_clip_uint8( pix>>5 );
            pix += b;
        }
        src += FDEC_STRIDE;
        i00 += c;
    }
}


/****************************************************************************
 * 8x8 prediction for intra chroma block
 ****************************************************************************/

static void predict_8x8c_dc_128( UINT8 *src )
{
    int y;

    for( y = 0; y < 8; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = 0x80808080;
        *p++ = 0x80808080;
        src += FDEC_STRIDE;
    }
}
static void predict_8x8c_dc_left( UINT8 *src )
{
    int y;
    UINT32 dc0 = 0, dc1 = 0;

    for( y = 0; y < 4; y++ )
    {
        dc0 += src[y * FDEC_STRIDE     - 1];
        dc1 += src[(y+4) * FDEC_STRIDE - 1];
    }
    dc0 = (( dc0 + 2 ) >> 2)*0x01010101;
    dc1 = (( dc1 + 2 ) >> 2)*0x01010101;

    for( y = 0; y < 4; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = dc0;
        *p++ = dc0;
        src += FDEC_STRIDE;
    }
    for( y = 0; y < 4; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = dc1;
        *p++ = dc1;
        src += FDEC_STRIDE;
    }

}
static void predict_8x8c_dc_top( UINT8 *src )
{
    int y, x;
    UINT32 dc0 = 0, dc1 = 0;

    for( x = 0; x < 4; x++ )
    {
        dc0 += src[x     - FDEC_STRIDE];
        dc1 += src[x + 4 - FDEC_STRIDE];
    }
    dc0 = (( dc0 + 2 ) >> 2)*0x01010101;
    dc1 = (( dc1 + 2 ) >> 2)*0x01010101;

    for( y = 0; y < 8; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = dc0;
        *p++ = dc1;
        src += FDEC_STRIDE;
    }
}
static void predict_8x8c_dc( UINT8 *src )
{
    int y;
    int s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    UINT32 dc0, dc1, dc2, dc3;
    int i;

    /*
          s0 s1
       s2
       s3
    */
    for( i = 0; i < 4; i++ )
    {
        s0 += src[i - FDEC_STRIDE];
        s1 += src[i + 4 - FDEC_STRIDE];
        s2 += src[-1 + i * FDEC_STRIDE];
        s3 += src[-1 + (i+4)*FDEC_STRIDE];
    }
    /*
       dc0 dc1
       dc2 dc3
     */
    dc0 = (( s0 + s2 + 4 ) >> 3)*0x01010101;
    dc1 = (( s1 + 2 ) >> 2)*0x01010101;
    dc2 = (( s3 + 2 ) >> 2)*0x01010101;
    dc3 = (( s1 + s3 + 4 ) >> 3)*0x01010101;

    for( y = 0; y < 4; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = dc0;
        *p++ = dc1;
        src += FDEC_STRIDE;
    }

    for( y = 0; y < 4; y++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = dc2;
        *p++ = dc3;
        src += FDEC_STRIDE;
    }
}
static void predict_8x8c_h( UINT8 *src )
{
    int i;

    for( i = 0; i < 8; i++ )
    {
        UINT32 v = 0x01010101 * src[-1];
        UINT32 *p = (UINT32*)src;
        *p++ = v;
        *p++ = v;
        src += FDEC_STRIDE;
    }
}
static void predict_8x8c_v( UINT8 *src )
{
    UINT32 v0 = *(UINT32*)&src[0-FDEC_STRIDE];
    UINT32 v1 = *(UINT32*)&src[4-FDEC_STRIDE];
    int i;

    for( i = 0; i < 8; i++ )
    {
        UINT32 *p = (UINT32*)src;
        *p++ = v0;
        *p++ = v1;
        src += FDEC_STRIDE;
    }
}
static void predict_8x8c_p( UINT8 *src )
{
    int i;
    int x,y;
    int a, b, c;
    int H = 0;
    int V = 0;
    int i00;

    for( i = 0; i < 4; i++ )
    {
        H += ( i + 1 ) * ( src[4+i - FDEC_STRIDE] - src[2 - i -FDEC_STRIDE] );
        V += ( i + 1 ) * ( src[-1 +(i+4)*FDEC_STRIDE] - src[-1+(2-i)*FDEC_STRIDE] );
    }

    a = 16 * ( src[-1+7*FDEC_STRIDE] + src[7 - FDEC_STRIDE] );
    b = ( 17 * H + 16 ) >> 5;
    c = ( 17 * V + 16 ) >> 5;
    i00 = a -3*b -3*c + 16;

    for( y = 0; y < 8; y++ )
    {
        int pix = i00;
        for( x = 0; x < 8; x++ )
        {
            src[x] = x264_clip_uint8( pix>>5 );
            pix += b;
        }
        src += FDEC_STRIDE;
        i00 += c;
    }
}

/****************************************************************************
 * 4x4 prediction for intra luma block
 ****************************************************************************/

#define PREDICT_4x4_DC(v) \
{\
    *(UINT32*)&src[0*FDEC_STRIDE] =\
    *(UINT32*)&src[1*FDEC_STRIDE] =\
    *(UINT32*)&src[2*FDEC_STRIDE] =\
    *(UINT32*)&src[3*FDEC_STRIDE] = v;\
}

static void predict_4x4_dc_128( UINT8 *src )
{
    PREDICT_4x4_DC(0x80808080);
}
static void predict_4x4_dc_left( UINT8 *src )
{
    UINT32 dc = (( src[-1+0*FDEC_STRIDE] + src[-1+FDEC_STRIDE]+
                     src[-1+2*FDEC_STRIDE] + src[-1+3*FDEC_STRIDE] + 2 ) >> 2)*0x01010101;
    PREDICT_4x4_DC(dc);
}
static void predict_4x4_dc_top( UINT8 *src )
{
    UINT32 dc = (( src[0 - FDEC_STRIDE] + src[1 - FDEC_STRIDE] +
                     src[2 - FDEC_STRIDE] + src[3 - FDEC_STRIDE] + 2 ) >> 2)*0x01010101;
    PREDICT_4x4_DC(dc);
}
static void predict_4x4_dc( UINT8 *src )
{
    UINT32 dc = (( src[-1+0*FDEC_STRIDE] + src[-1+FDEC_STRIDE] +
                     src[-1+2*FDEC_STRIDE] + src[-1+3*FDEC_STRIDE] +
                     src[0 - FDEC_STRIDE]  + src[1 - FDEC_STRIDE] +
                     src[2 - FDEC_STRIDE]  + src[3 - FDEC_STRIDE] + 4 ) >> 3)*0x01010101;
    PREDICT_4x4_DC(dc);
}
static void predict_4x4_h( UINT8 *src )
{
    *(UINT32*)&src[0*FDEC_STRIDE] = src[0*FDEC_STRIDE-1] * 0x01010101;
    *(UINT32*)&src[1*FDEC_STRIDE] = src[1*FDEC_STRIDE-1] * 0x01010101;
    *(UINT32*)&src[2*FDEC_STRIDE] = src[2*FDEC_STRIDE-1] * 0x01010101;
    *(UINT32*)&src[3*FDEC_STRIDE] = src[3*FDEC_STRIDE-1] * 0x01010101;
}
static void predict_4x4_v( UINT8 *src )
{
    UINT32 top = *((UINT32*)&src[-FDEC_STRIDE]);
    PREDICT_4x4_DC(top);
}

#define PREDICT_4x4_LOAD_LEFT \
    const int l0 = src[-1+0*FDEC_STRIDE];   \
    const int l1 = src[-1+1*FDEC_STRIDE];   \
    const int l2 = src[-1+2*FDEC_STRIDE];   \
    UNUSED const int l3 = src[-1+3*FDEC_STRIDE];

#define PREDICT_4x4_LOAD_TOP \
    const int t0 = src[0-1*FDEC_STRIDE];   \
    const int t1 = src[1-1*FDEC_STRIDE];   \
    const int t2 = src[2-1*FDEC_STRIDE];   \
    UNUSED const int t3 = src[3-1*FDEC_STRIDE];

#define PREDICT_4x4_LOAD_TOP_RIGHT \
    const int t4 = src[4-1*FDEC_STRIDE];   \
    const int t5 = src[5-1*FDEC_STRIDE];   \
    const int t6 = src[6-1*FDEC_STRIDE];   \
    UNUSED const int t7 = src[7-1*FDEC_STRIDE];

static void predict_4x4_ddl( UINT8 *src )
{
    PREDICT_4x4_LOAD_TOP
    PREDICT_4x4_LOAD_TOP_RIGHT

    src[0*FDEC_STRIDE+0] = ( t0 + 2*t1 + t2 + 2 ) >> 2;

    src[0*FDEC_STRIDE+1] =
    src[1*FDEC_STRIDE+0] = ( t1 + 2*t2 + t3 + 2 ) >> 2;

    src[0*FDEC_STRIDE+2] =
    src[1*FDEC_STRIDE+1] =
    src[2*FDEC_STRIDE+0] = ( t2 + 2*t3 + t4 + 2 ) >> 2;

    src[0*FDEC_STRIDE+3] =
    src[1*FDEC_STRIDE+2] =
    src[2*FDEC_STRIDE+1] =
    src[3*FDEC_STRIDE+0] = ( t3 + 2*t4 + t5 + 2 ) >> 2;

    src[1*FDEC_STRIDE+3] =
    src[2*FDEC_STRIDE+2] =
    src[3*FDEC_STRIDE+1] = ( t4 + 2*t5 + t6 + 2 ) >> 2;

    src[2*FDEC_STRIDE+3] =
    src[3*FDEC_STRIDE+2] = ( t5 + 2*t6 + t7 + 2 ) >> 2;

    src[3*FDEC_STRIDE+3] = ( t6 + 3*t7 + 2 ) >> 2;
}
static void predict_4x4_ddr( UINT8 *src )
{
    const int lt = src[-1-FDEC_STRIDE];
    PREDICT_4x4_LOAD_LEFT
    PREDICT_4x4_LOAD_TOP

    src[0*FDEC_STRIDE+0] =
    src[1*FDEC_STRIDE+1] =
    src[2*FDEC_STRIDE+2] =
    src[3*FDEC_STRIDE+3] = ( t0 + 2 * lt + l0 + 2 ) >> 2;

    src[0*FDEC_STRIDE+1] =
    src[1*FDEC_STRIDE+2] =
    src[2*FDEC_STRIDE+3] = ( lt + 2 * t0 + t1 + 2 ) >> 2;

    src[0*FDEC_STRIDE+2] =
    src[1*FDEC_STRIDE+3] = ( t0 + 2 * t1 + t2 + 2 ) >> 2;

    src[0*FDEC_STRIDE+3] = ( t1 + 2 * t2 + t3 + 2 ) >> 2;

    src[1*FDEC_STRIDE+0] =
    src[2*FDEC_STRIDE+1] =
    src[3*FDEC_STRIDE+2] = ( lt + 2 * l0 + l1 + 2 ) >> 2;

    src[2*FDEC_STRIDE+0] =
    src[3*FDEC_STRIDE+1] = ( l0 + 2 * l1 + l2 + 2 ) >> 2;

    src[3*FDEC_STRIDE+0] = ( l1 + 2 * l2 + l3 + 2 ) >> 2;
}

static void predict_4x4_vr( UINT8 *src )
{
    const int lt = src[-1-FDEC_STRIDE];
    PREDICT_4x4_LOAD_LEFT
    PREDICT_4x4_LOAD_TOP

    src[0*FDEC_STRIDE+0]=
    src[2*FDEC_STRIDE+1]= ( lt + t0 + 1 ) >> 1;

    src[0*FDEC_STRIDE+1]=
    src[2*FDEC_STRIDE+2]= ( t0 + t1 + 1 ) >> 1;

    src[0*FDEC_STRIDE+2]=
    src[2*FDEC_STRIDE+3]= ( t1 + t2 + 1 ) >> 1;

    src[0*FDEC_STRIDE+3]= ( t2 + t3 + 1 ) >> 1;

    src[1*FDEC_STRIDE+0]=
    src[3*FDEC_STRIDE+1]= ( l0 + 2 * lt + t0 + 2 ) >> 2;

    src[1*FDEC_STRIDE+1]=
    src[3*FDEC_STRIDE+2]= ( lt + 2 * t0 + t1 + 2 ) >> 2;

    src[1*FDEC_STRIDE+2]=
    src[3*FDEC_STRIDE+3]= ( t0 + 2 * t1 + t2 + 2) >> 2;

    src[1*FDEC_STRIDE+3]= ( t1 + 2 * t2 + t3 + 2 ) >> 2;
    src[2*FDEC_STRIDE+0]= ( lt + 2 * l0 + l1 + 2 ) >> 2;
    src[3*FDEC_STRIDE+0]= ( l0 + 2 * l1 + l2 + 2 ) >> 2;
}

static void predict_4x4_hd( UINT8 *src )
{
    const int lt= src[-1-1*FDEC_STRIDE];
    PREDICT_4x4_LOAD_LEFT
    PREDICT_4x4_LOAD_TOP

    src[0*FDEC_STRIDE+0]=
    src[1*FDEC_STRIDE+2]= ( lt + l0 + 1 ) >> 1;
    src[0*FDEC_STRIDE+1]=
    src[1*FDEC_STRIDE+3]= ( l0 + 2 * lt + t0 + 2 ) >> 2;
    src[0*FDEC_STRIDE+2]= ( lt + 2 * t0 + t1 + 2 ) >> 2;
    src[0*FDEC_STRIDE+3]= ( t0 + 2 * t1 + t2 + 2 ) >> 2;
    src[1*FDEC_STRIDE+0]=
    src[2*FDEC_STRIDE+2]= ( l0 + l1 + 1 ) >> 1;
    src[1*FDEC_STRIDE+1]=
    src[2*FDEC_STRIDE+3]= ( lt + 2 * l0 + l1 + 2 ) >> 2;
    src[2*FDEC_STRIDE+0]=
    src[3*FDEC_STRIDE+2]= ( l1 + l2+ 1 ) >> 1;
    src[2*FDEC_STRIDE+1]=
    src[3*FDEC_STRIDE+3]= ( l0 + 2 * l1 + l2 + 2 ) >> 2;
    src[3*FDEC_STRIDE+0]= ( l2 + l3 + 1 ) >> 1;
    src[3*FDEC_STRIDE+1]= ( l1 + 2 * l2 + l3 + 2 ) >> 2;
}

static void predict_4x4_vl( UINT8 *src )
{
    PREDICT_4x4_LOAD_TOP
    PREDICT_4x4_LOAD_TOP_RIGHT

    src[0*FDEC_STRIDE+0]= ( t0 + t1 + 1 ) >> 1;
    src[0*FDEC_STRIDE+1]=
    src[2*FDEC_STRIDE+0]= ( t1 + t2 + 1 ) >> 1;
    src[0*FDEC_STRIDE+2]=
    src[2*FDEC_STRIDE+1]= ( t2 + t3 + 1 ) >> 1;
    src[0*FDEC_STRIDE+3]=
    src[2*FDEC_STRIDE+2]= ( t3 + t4 + 1 ) >> 1;
    src[2*FDEC_STRIDE+3]= ( t4 + t5 + 1 ) >> 1;
    src[1*FDEC_STRIDE+0]= ( t0 + 2 * t1 + t2 + 2 ) >> 2;
    src[1*FDEC_STRIDE+1]=
    src[3*FDEC_STRIDE+0]= ( t1 + 2 * t2 + t3 + 2 ) >> 2;
    src[1*FDEC_STRIDE+2]=
    src[3*FDEC_STRIDE+1]= ( t2 + 2 * t3 + t4 + 2 ) >> 2;
    src[1*FDEC_STRIDE+3]=
    src[3*FDEC_STRIDE+2]= ( t3 + 2 * t4 + t5 + 2 ) >> 2;
    src[3*FDEC_STRIDE+3]= ( t4 + 2 * t5 + t6 + 2 ) >> 2;
}

static void predict_4x4_hu( UINT8 *src )
{
    PREDICT_4x4_LOAD_LEFT

    src[0*FDEC_STRIDE+0]= ( l0 + l1 + 1 ) >> 1;
    src[0*FDEC_STRIDE+1]= ( l0 + 2 * l1 + l2 + 2 ) >> 2;

    src[0*FDEC_STRIDE+2]=
    src[1*FDEC_STRIDE+0]= ( l1 + l2 + 1 ) >> 1;

    src[0*FDEC_STRIDE+3]=
    src[1*FDEC_STRIDE+1]= ( l1 + 2*l2 + l3 + 2 ) >> 2;

    src[1*FDEC_STRIDE+2]=
    src[2*FDEC_STRIDE+0]= ( l2 + l3 + 1 ) >> 1;

    src[1*FDEC_STRIDE+3]=
    src[2*FDEC_STRIDE+1]= ( l2 + 2 * l3 + l3 + 2 ) >> 2;

    src[2*FDEC_STRIDE+3]=
    src[3*FDEC_STRIDE+1]=
    src[3*FDEC_STRIDE+0]=
    src[2*FDEC_STRIDE+2]=
    src[3*FDEC_STRIDE+2]=
    src[3*FDEC_STRIDE+3]= l3;
}

/****************************************************************************
 * 8x8 prediction for intra luma block
 ****************************************************************************/

#define SRC(x,y) src[(x)+(y)*FDEC_STRIDE]
#define PL(y) \
    const int l##y = (SRC(-1,y-1) + 2*SRC(-1,y) + SRC(-1,y+1) + 2) >> 2;
#define PREDICT_8x8_LOAD_LEFT(have_tl) \
    const int l0 = ((have_tl || (i_neighbor&MB_TOPLEFT) ? SRC(-1,-1) : SRC(-1,0)) \
                     + 2*SRC(-1,0) + SRC(-1,1) + 2) >> 2; \
    PL(1) PL(2) PL(3) PL(4) PL(5) PL(6) \
    UNUSED const int l7 = (SRC(-1,6) + 3*SRC(-1,7) + 2) >> 2;

#define PT(x) \
    const int t##x = (SRC(x-1,-1) + 2*SRC(x,-1) + SRC(x+1,-1) + 2) >> 2;
#define PREDICT_8x8_LOAD_TOP(have_tl) \
    const int t0 = ((have_tl || (i_neighbor&MB_TOPLEFT) ? SRC(-1,-1) : SRC(0,-1)) \
                     + 2*SRC(0,-1) + SRC(1,-1) + 2) >> 2; \
    PT(1) PT(2) PT(3) PT(4) PT(5) PT(6) \
    UNUSED const int t7 = ((i_neighbor&MB_TOPRIGHT ? SRC(8,-1) : SRC(7,-1)) \
                     + 2*SRC(7,-1) + SRC(6,-1) + 2) >> 2; \

#define PTR(x) \
    t##x = (SRC(x-1,-1) + 2*SRC(x,-1) + SRC(x+1,-1) + 2) >> 2;
#define PREDICT_8x8_LOAD_TOPRIGHT \
    int t8, t9, t10, t11, t12, t13, t14, t15; \
    if(i_neighbor&MB_TOPRIGHT) { \
        PTR(8) PTR(9) PTR(10) PTR(11) PTR(12) PTR(13) PTR(14) \
        t15 = (SRC(14,-1) + 3*SRC(15,-1) + 2) >> 2; \
    } else t8=t9=t10=t11=t12=t13=t14=t15= SRC(7,-1);

#define PREDICT_8x8_LOAD_TOPLEFT \
    const int lt = (SRC(-1,0) + 2*SRC(-1,-1) + SRC(0,-1) + 2) >> 2;

#define PREDICT_8x8_DC(v) \
    int y; \
    for( y = 0; y < 8; y++ ) { \
        ((UINT32*)src)[0] = \
        ((UINT32*)src)[1] = v; \
        src += FDEC_STRIDE; \
    }

static void predict_8x8_dc_128( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_DC(0x80808080);
}
static void predict_8x8_dc_left( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_LEFT(0)
    const UINT32 dc = ((l0+l1+l2+l3+l4+l5+l6+l7+4) >> 3) * 0x01010101;
    PREDICT_8x8_DC(dc);
}
static void predict_8x8_dc_top( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(0)
    const UINT32 dc = ((t0+t1+t2+t3+t4+t5+t6+t7+4) >> 3) * 0x01010101;
    PREDICT_8x8_DC(dc);
}
static void predict_8x8_dc( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_LEFT(0)
    PREDICT_8x8_LOAD_TOP(0)
    const UINT32 dc = ((l0+l1+l2+l3+l4+l5+l6+l7
                         +t0+t1+t2+t3+t4+t5+t6+t7+8) >> 4) * 0x01010101;
    PREDICT_8x8_DC(dc);
}
static void predict_8x8_h( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_LEFT(0)
#define ROW(y) ((UINT32*)(src+y*FDEC_STRIDE))[0] =\
               ((UINT32*)(src+y*FDEC_STRIDE))[1] = 0x01010101U * l##y
    ROW(0); ROW(1); ROW(2); ROW(3); ROW(4); ROW(5); ROW(6); ROW(7);
#undef ROW
}
static void predict_8x8_v( UINT8 *src, int i_neighbor )
{
    int y;
    PREDICT_8x8_LOAD_TOP(0);
    src[0] = t0;
    src[1] = t1;
    src[2] = t2;
    src[3] = t3;
    src[4] = t4;
    src[5] = t5;
    src[6] = t6;
    src[7] = t7;
    for( y = 1; y < 8; y++ )
        *(UINT64*)(src+y*FDEC_STRIDE) = *(UINT64*)src;
}
static void predict_8x8_ddl( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(0)
    PREDICT_8x8_LOAD_TOPRIGHT
    SRC(0,0)= (t0 + 2*t1 + t2 + 2) >> 2;
    SRC(0,1)=SRC(1,0)= (t1 + 2*t2 + t3 + 2) >> 2;
    SRC(0,2)=SRC(1,1)=SRC(2,0)= (t2 + 2*t3 + t4 + 2) >> 2;
    SRC(0,3)=SRC(1,2)=SRC(2,1)=SRC(3,0)= (t3 + 2*t4 + t5 + 2) >> 2;
    SRC(0,4)=SRC(1,3)=SRC(2,2)=SRC(3,1)=SRC(4,0)= (t4 + 2*t5 + t6 + 2) >> 2;
    SRC(0,5)=SRC(1,4)=SRC(2,3)=SRC(3,2)=SRC(4,1)=SRC(5,0)= (t5 + 2*t6 + t7 + 2) >> 2;
    SRC(0,6)=SRC(1,5)=SRC(2,4)=SRC(3,3)=SRC(4,2)=SRC(5,1)=SRC(6,0)= (t6 + 2*t7 + t8 + 2) >> 2;
    SRC(0,7)=SRC(1,6)=SRC(2,5)=SRC(3,4)=SRC(4,3)=SRC(5,2)=SRC(6,1)=SRC(7,0)= (t7 + 2*t8 + t9 + 2) >> 2;
    SRC(1,7)=SRC(2,6)=SRC(3,5)=SRC(4,4)=SRC(5,3)=SRC(6,2)=SRC(7,1)= (t8 + 2*t9 + t10 + 2) >> 2;
    SRC(2,7)=SRC(3,6)=SRC(4,5)=SRC(5,4)=SRC(6,3)=SRC(7,2)= (t9 + 2*t10 + t11 + 2) >> 2;
    SRC(3,7)=SRC(4,6)=SRC(5,5)=SRC(6,4)=SRC(7,3)= (t10 + 2*t11 + t12 + 2) >> 2;
    SRC(4,7)=SRC(5,6)=SRC(6,5)=SRC(7,4)= (t11 + 2*t12 + t13 + 2) >> 2;
    SRC(5,7)=SRC(6,6)=SRC(7,5)= (t12 + 2*t13 + t14 + 2) >> 2;
    SRC(6,7)=SRC(7,6)= (t13 + 2*t14 + t15 + 2) >> 2;
    SRC(7,7)= (t14 + 3*t15 + 2) >> 2;
}
static void predict_8x8_ddr( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(1)
    PREDICT_8x8_LOAD_LEFT(1)
    PREDICT_8x8_LOAD_TOPLEFT
    SRC(0,7)= (l7 + 2*l6 + l5 + 2) >> 2;
    SRC(0,6)=SRC(1,7)= (l6 + 2*l5 + l4 + 2) >> 2;
    SRC(0,5)=SRC(1,6)=SRC(2,7)= (l5 + 2*l4 + l3 + 2) >> 2;
    SRC(0,4)=SRC(1,5)=SRC(2,6)=SRC(3,7)= (l4 + 2*l3 + l2 + 2) >> 2;
    SRC(0,3)=SRC(1,4)=SRC(2,5)=SRC(3,6)=SRC(4,7)= (l3 + 2*l2 + l1 + 2) >> 2;
    SRC(0,2)=SRC(1,3)=SRC(2,4)=SRC(3,5)=SRC(4,6)=SRC(5,7)= (l2 + 2*l1 + l0 + 2) >> 2;
    SRC(0,1)=SRC(1,2)=SRC(2,3)=SRC(3,4)=SRC(4,5)=SRC(5,6)=SRC(6,7)= (l1 + 2*l0 + lt + 2) >> 2;
    SRC(0,0)=SRC(1,1)=SRC(2,2)=SRC(3,3)=SRC(4,4)=SRC(5,5)=SRC(6,6)=SRC(7,7)= (l0 + 2*lt + t0 + 2) >> 2;
    SRC(1,0)=SRC(2,1)=SRC(3,2)=SRC(4,3)=SRC(5,4)=SRC(6,5)=SRC(7,6)= (lt + 2*t0 + t1 + 2) >> 2;
    SRC(2,0)=SRC(3,1)=SRC(4,2)=SRC(5,3)=SRC(6,4)=SRC(7,5)= (t0 + 2*t1 + t2 + 2) >> 2;
    SRC(3,0)=SRC(4,1)=SRC(5,2)=SRC(6,3)=SRC(7,4)= (t1 + 2*t2 + t3 + 2) >> 2;
    SRC(4,0)=SRC(5,1)=SRC(6,2)=SRC(7,3)= (t2 + 2*t3 + t4 + 2) >> 2;
    SRC(5,0)=SRC(6,1)=SRC(7,2)= (t3 + 2*t4 + t5 + 2) >> 2;
    SRC(6,0)=SRC(7,1)= (t4 + 2*t5 + t6 + 2) >> 2;
    SRC(7,0)= (t5 + 2*t6 + t7 + 2) >> 2;
  
}
static void predict_8x8_vr( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(1)
    PREDICT_8x8_LOAD_LEFT(1)
    PREDICT_8x8_LOAD_TOPLEFT
    SRC(0,6)= (l5 + 2*l4 + l3 + 2) >> 2;
    SRC(0,7)= (l6 + 2*l5 + l4 + 2) >> 2;
    SRC(0,4)=SRC(1,6)= (l3 + 2*l2 + l1 + 2) >> 2;
    SRC(0,5)=SRC(1,7)= (l4 + 2*l3 + l2 + 2) >> 2;
    SRC(0,2)=SRC(1,4)=SRC(2,6)= (l1 + 2*l0 + lt + 2) >> 2;
    SRC(0,3)=SRC(1,5)=SRC(2,7)= (l2 + 2*l1 + l0 + 2) >> 2;
    SRC(0,1)=SRC(1,3)=SRC(2,5)=SRC(3,7)= (l0 + 2*lt + t0 + 2) >> 2;
    SRC(0,0)=SRC(1,2)=SRC(2,4)=SRC(3,6)= (lt + t0 + 1) >> 1;
    SRC(1,1)=SRC(2,3)=SRC(3,5)=SRC(4,7)= (lt + 2*t0 + t1 + 2) >> 2;
    SRC(1,0)=SRC(2,2)=SRC(3,4)=SRC(4,6)= (t0 + t1 + 1) >> 1;
    SRC(2,1)=SRC(3,3)=SRC(4,5)=SRC(5,7)= (t0 + 2*t1 + t2 + 2) >> 2;
    SRC(2,0)=SRC(3,2)=SRC(4,4)=SRC(5,6)= (t1 + t2 + 1) >> 1;
    SRC(3,1)=SRC(4,3)=SRC(5,5)=SRC(6,7)= (t1 + 2*t2 + t3 + 2) >> 2;
    SRC(3,0)=SRC(4,2)=SRC(5,4)=SRC(6,6)= (t2 + t3 + 1) >> 1;
    SRC(4,1)=SRC(5,3)=SRC(6,5)=SRC(7,7)= (t2 + 2*t3 + t4 + 2) >> 2;
    SRC(4,0)=SRC(5,2)=SRC(6,4)=SRC(7,6)= (t3 + t4 + 1) >> 1;
    SRC(5,1)=SRC(6,3)=SRC(7,5)= (t3 + 2*t4 + t5 + 2) >> 2;
    SRC(5,0)=SRC(6,2)=SRC(7,4)= (t4 + t5 + 1) >> 1;
    SRC(6,1)=SRC(7,3)= (t4 + 2*t5 + t6 + 2) >> 2;
    SRC(6,0)=SRC(7,2)= (t5 + t6 + 1) >> 1;
    SRC(7,1)= (t5 + 2*t6 + t7 + 2) >> 2;
    SRC(7,0)= (t6 + t7 + 1) >> 1;
}
static void predict_8x8_hd( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(1)
    PREDICT_8x8_LOAD_LEFT(1)
    PREDICT_8x8_LOAD_TOPLEFT
    SRC(0,7)= (l6 + l7 + 1) >> 1;
    SRC(1,7)= (l5 + 2*l6 + l7 + 2) >> 2;
    SRC(0,6)=SRC(2,7)= (l5 + l6 + 1) >> 1;
    SRC(1,6)=SRC(3,7)= (l4 + 2*l5 + l6 + 2) >> 2;
    SRC(0,5)=SRC(2,6)=SRC(4,7)= (l4 + l5 + 1) >> 1;
    SRC(1,5)=SRC(3,6)=SRC(5,7)= (l3 + 2*l4 + l5 + 2) >> 2;
    SRC(0,4)=SRC(2,5)=SRC(4,6)=SRC(6,7)= (l3 + l4 + 1) >> 1;
    SRC(1,4)=SRC(3,5)=SRC(5,6)=SRC(7,7)= (l2 + 2*l3 + l4 + 2) >> 2;
    SRC(0,3)=SRC(2,4)=SRC(4,5)=SRC(6,6)= (l2 + l3 + 1) >> 1;
    SRC(1,3)=SRC(3,4)=SRC(5,5)=SRC(7,6)= (l1 + 2*l2 + l3 + 2) >> 2;
    SRC(0,2)=SRC(2,3)=SRC(4,4)=SRC(6,5)= (l1 + l2 + 1) >> 1;
    SRC(1,2)=SRC(3,3)=SRC(5,4)=SRC(7,5)= (l0 + 2*l1 + l2 + 2) >> 2;
    SRC(0,1)=SRC(2,2)=SRC(4,3)=SRC(6,4)= (l0 + l1 + 1) >> 1;
    SRC(1,1)=SRC(3,2)=SRC(5,3)=SRC(7,4)= (lt + 2*l0 + l1 + 2) >> 2;
    SRC(0,0)=SRC(2,1)=SRC(4,2)=SRC(6,3)= (lt + l0 + 1) >> 1;
    SRC(1,0)=SRC(3,1)=SRC(5,2)=SRC(7,3)= (l0 + 2*lt + t0 + 2) >> 2;
    SRC(2,0)=SRC(4,1)=SRC(6,2)= (t1 + 2*t0 + lt + 2) >> 2;
    SRC(3,0)=SRC(5,1)=SRC(7,2)= (t2 + 2*t1 + t0 + 2) >> 2;
    SRC(4,0)=SRC(6,1)= (t3 + 2*t2 + t1 + 2) >> 2;
    SRC(5,0)=SRC(7,1)= (t4 + 2*t3 + t2 + 2) >> 2;
    SRC(6,0)= (t5 + 2*t4 + t3 + 2) >> 2;
    SRC(7,0)= (t6 + 2*t5 + t4 + 2) >> 2;
}
static void predict_8x8_vl( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_TOP(0)
    PREDICT_8x8_LOAD_TOPRIGHT
    SRC(0,0)= (t0 + t1 + 1) >> 1;
    SRC(0,1)= (t0 + 2*t1 + t2 + 2) >> 2;
    SRC(0,2)=SRC(1,0)= (t1 + t2 + 1) >> 1;
    SRC(0,3)=SRC(1,1)= (t1 + 2*t2 + t3 + 2) >> 2;
    SRC(0,4)=SRC(1,2)=SRC(2,0)= (t2 + t3 + 1) >> 1;
    SRC(0,5)=SRC(1,3)=SRC(2,1)= (t2 + 2*t3 + t4 + 2) >> 2;
    SRC(0,6)=SRC(1,4)=SRC(2,2)=SRC(3,0)= (t3 + t4 + 1) >> 1;
    SRC(0,7)=SRC(1,5)=SRC(2,3)=SRC(3,1)= (t3 + 2*t4 + t5 + 2) >> 2;
    SRC(1,6)=SRC(2,4)=SRC(3,2)=SRC(4,0)= (t4 + t5 + 1) >> 1;
    SRC(1,7)=SRC(2,5)=SRC(3,3)=SRC(4,1)= (t4 + 2*t5 + t6 + 2) >> 2;
    SRC(2,6)=SRC(3,4)=SRC(4,2)=SRC(5,0)= (t5 + t6 + 1) >> 1;
    SRC(2,7)=SRC(3,5)=SRC(4,3)=SRC(5,1)= (t5 + 2*t6 + t7 + 2) >> 2;
    SRC(3,6)=SRC(4,4)=SRC(5,2)=SRC(6,0)= (t6 + t7 + 1) >> 1;
    SRC(3,7)=SRC(4,5)=SRC(5,3)=SRC(6,1)= (t6 + 2*t7 + t8 + 2) >> 2;
    SRC(4,6)=SRC(5,4)=SRC(6,2)=SRC(7,0)= (t7 + t8 + 1) >> 1;
    SRC(4,7)=SRC(5,5)=SRC(6,3)=SRC(7,1)= (t7 + 2*t8 + t9 + 2) >> 2;
    SRC(5,6)=SRC(6,4)=SRC(7,2)= (t8 + t9 + 1) >> 1;
    SRC(5,7)=SRC(6,5)=SRC(7,3)= (t8 + 2*t9 + t10 + 2) >> 2;
    SRC(6,6)=SRC(7,4)= (t9 + t10 + 1) >> 1;
    SRC(6,7)=SRC(7,5)= (t9 + 2*t10 + t11 + 2) >> 2;
    SRC(7,6)= (t10 + t11 + 1) >> 1;
    SRC(7,7)= (t10 + 2*t11 + t12 + 2) >> 2;
}
static void predict_8x8_hu( UINT8 *src, int i_neighbor )
{
    PREDICT_8x8_LOAD_LEFT(0)
    SRC(0,0)= (l0 + l1 + 1) >> 1;
    SRC(1,0)= (l0 + 2*l1 + l2 + 2) >> 2;
    SRC(0,1)=SRC(2,0)= (l1 + l2 + 1) >> 1;
    SRC(1,1)=SRC(3,0)= (l1 + 2*l2 + l3 + 2) >> 2;
    SRC(0,2)=SRC(2,1)=SRC(4,0)= (l2 + l3 + 1) >> 1;
    SRC(1,2)=SRC(3,1)=SRC(5,0)= (l2 + 2*l3 + l4 + 2) >> 2;
    SRC(0,3)=SRC(2,2)=SRC(4,1)=SRC(6,0)= (l3 + l4 + 1) >> 1;
    SRC(1,3)=SRC(3,2)=SRC(5,1)=SRC(7,0)= (l3 + 2*l4 + l5 + 2) >> 2;
    SRC(0,4)=SRC(2,3)=SRC(4,2)=SRC(6,1)= (l4 + l5 + 1) >> 1;
    SRC(1,4)=SRC(3,3)=SRC(5,2)=SRC(7,1)= (l4 + 2*l5 + l6 + 2) >> 2;
    SRC(0,5)=SRC(2,4)=SRC(4,3)=SRC(6,2)= (l5 + l6 + 1) >> 1;
    SRC(1,5)=SRC(3,4)=SRC(5,3)=SRC(7,2)= (l5 + 2*l6 + l7 + 2) >> 2;
    SRC(0,6)=SRC(2,5)=SRC(4,4)=SRC(6,3)= (l6 + l7 + 1) >> 1;
    SRC(1,6)=SRC(3,5)=SRC(5,4)=SRC(7,3)= (l6 + 3*l7 + 2) >> 2;
    SRC(0,7)=SRC(1,7)=SRC(2,6)=SRC(2,7)=SRC(3,6)=
    SRC(3,7)=SRC(4,5)=SRC(4,6)=SRC(4,7)=SRC(5,5)=
    SRC(5,6)=SRC(5,7)=SRC(6,4)=SRC(6,5)=SRC(6,6)=
    SRC(6,7)=SRC(7,4)=SRC(7,5)=SRC(7,6)=SRC(7,7)= l7;
}

/****************************************************************************
 * Exported functions:
 ****************************************************************************/
void x264_predict_16x16_init( int cpu, x264_predict_t pf[7] )
{
    pf[I_PRED_16x16_V ]     = predict_16x16_v;
    pf[I_PRED_16x16_H ]     = predict_16x16_h;
    pf[I_PRED_16x16_DC]     = predict_16x16_dc;
    pf[I_PRED_16x16_P ]     = predict_16x16_p;
    pf[I_PRED_16x16_DC_LEFT]= predict_16x16_dc_left;
    pf[I_PRED_16x16_DC_TOP ]= predict_16x16_dc_top;
    pf[I_PRED_16x16_DC_128 ]= predict_16x16_dc_128;


}

void x264_predict_8x8c_init( int cpu, x264_predict_t pf[7] )
{
    pf[I_PRED_CHROMA_V ]     = predict_8x8c_v;
    pf[I_PRED_CHROMA_H ]     = predict_8x8c_h;
    pf[I_PRED_CHROMA_DC]     = predict_8x8c_dc;
    pf[I_PRED_CHROMA_P ]     = predict_8x8c_p;
    pf[I_PRED_CHROMA_DC_LEFT]= predict_8x8c_dc_left;
    pf[I_PRED_CHROMA_DC_TOP ]= predict_8x8c_dc_top;
    pf[I_PRED_CHROMA_DC_128 ]= predict_8x8c_dc_128;


}

void x264_predict_8x8_init( int cpu, x264_predict8x8_t pf[12] )
{
    pf[I_PRED_8x8_V]      = predict_8x8_v;
    pf[I_PRED_8x8_H]      = predict_8x8_h;
    pf[I_PRED_8x8_DC]     = predict_8x8_dc;
    pf[I_PRED_8x8_DDL]    = predict_8x8_ddl;
    pf[I_PRED_8x8_DDR]    = predict_8x8_ddr;
    pf[I_PRED_8x8_VR]     = predict_8x8_vr;
    pf[I_PRED_8x8_HD]     = predict_8x8_hd;
    pf[I_PRED_8x8_VL]     = predict_8x8_vl;
    pf[I_PRED_8x8_HU]     = predict_8x8_hu;
    pf[I_PRED_8x8_DC_LEFT]= predict_8x8_dc_left;
    pf[I_PRED_8x8_DC_TOP] = predict_8x8_dc_top;
    pf[I_PRED_8x8_DC_128] = predict_8x8_dc_128;


}

void x264_predict_4x4_init( int cpu, x264_predict_t pf[12] )
{
    pf[I_PRED_4x4_V]      = predict_4x4_v;
    pf[I_PRED_4x4_H]      = predict_4x4_h;
    pf[I_PRED_4x4_DC]     = predict_4x4_dc;
    pf[I_PRED_4x4_DDL]    = predict_4x4_ddl;
    pf[I_PRED_4x4_DDR]    = predict_4x4_ddr;
    pf[I_PRED_4x4_VR]     = predict_4x4_vr;
    pf[I_PRED_4x4_HD]     = predict_4x4_hd;
    pf[I_PRED_4x4_VL]     = predict_4x4_vl;
    pf[I_PRED_4x4_HU]     = predict_4x4_hu;
    pf[I_PRED_4x4_DC_LEFT]= predict_4x4_dc_left;
    pf[I_PRED_4x4_DC_TOP] = predict_4x4_dc_top;
    pf[I_PRED_4x4_DC_128] = predict_4x4_dc_128;


}

