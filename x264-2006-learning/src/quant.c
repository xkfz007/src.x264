/*****************************************************************************
 * quant.c: h264 encoder library
 *****************************************************************************
 * Copyright (C) 2005 x264 project
 *
 * Authors: Christian Heine <sennindemokrit@gmx.net>
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

#include "common.h"


#define QUANT_ONE( coef, mf ) \
{ \
    if( (coef) > 0 ) \
        (coef) = ( f + (coef) * (mf) ) >> i_qbits; \
    else \
        (coef) = - ( ( f - (coef) * (mf) ) >> i_qbits ); \
}

static void quant_8x8_core( INT16 dct[8][8], int quant_mf[8][8], int i_qbits, int f )
{
    int i;
    for( i = 0; i < 64; i++ )
        QUANT_ONE( dct[0][i], quant_mf[0][i] );
}

static void quant_4x4_core( INT16 dct[4][4], int quant_mf[4][4], int i_qbits, int f )
{
    int i;
    for( i = 0; i < 16; i++ )
        QUANT_ONE( dct[0][i], quant_mf[0][i] );
}

static void quant_4x4_dc_core( INT16 dct[4][4], int i_quant_mf, int i_qbits, int f )
{
    int i;
    for( i = 0; i < 16; i++ )
        QUANT_ONE( dct[0][i], i_quant_mf );
}

static void quant_2x2_dc_core( INT16 dct[2][2], int i_quant_mf, int i_qbits, int f )
{
    QUANT_ONE( dct[0][0], i_quant_mf );
    QUANT_ONE( dct[0][1], i_quant_mf );
    QUANT_ONE( dct[0][2], i_quant_mf );
    QUANT_ONE( dct[0][3], i_quant_mf );
}

#define DEQUANT_SHL( x ) \
    dct[y][x] = ( dct[y][x] * dequant_mf[i_mf][y][x] ) << i_qbits

#define DEQUANT_SHR( x ) \
    dct[y][x] = ( dct[y][x] * dequant_mf[i_mf][y][x] + f ) >> (-i_qbits)

static void dequant_4x4( INT16 dct[4][4], int dequant_mf[6][4][4], int i_qp )
{
    const int i_mf = i_qp%6;
    const int i_qbits = i_qp/6 - 4;
    int y;

    if( i_qbits >= 0 )
    {
        for( y = 0; y < 4; y++ )
        {
            DEQUANT_SHL( 0 );
            DEQUANT_SHL( 1 );
            DEQUANT_SHL( 2 );
            DEQUANT_SHL( 3 );
        }
    }
    else
    {
        const int f = 1 << (-i_qbits-1);
        for( y = 0; y < 4; y++ )
        {
            DEQUANT_SHR( 0 );
            DEQUANT_SHR( 1 );
            DEQUANT_SHR( 2 );
            DEQUANT_SHR( 3 );
        }
    }
}

static void dequant_8x8( INT16 dct[8][8], int dequant_mf[6][8][8], int i_qp )
{
    const int i_mf = i_qp%6;
    const int i_qbits = i_qp/6 - 6;
    int y;

    if( i_qbits >= 0 )
    {
        for( y = 0; y < 8; y++ )
        {
            DEQUANT_SHL( 0 );
            DEQUANT_SHL( 1 );
            DEQUANT_SHL( 2 );
            DEQUANT_SHL( 3 );
            DEQUANT_SHL( 4 );
            DEQUANT_SHL( 5 );
            DEQUANT_SHL( 6 );
            DEQUANT_SHL( 7 );
        }
    }
    else
    {
        const int f = 1 << (-i_qbits-1);
        for( y = 0; y < 8; y++ )
        {
            DEQUANT_SHR( 0 );
            DEQUANT_SHR( 1 );
            DEQUANT_SHR( 2 );
            DEQUANT_SHR( 3 );
            DEQUANT_SHR( 4 );
            DEQUANT_SHR( 5 );
            DEQUANT_SHR( 6 );
            DEQUANT_SHR( 7 );
        }
    }
}

void x264_mb_dequant_2x2_dc( INT16 dct[2][2], int dequant_mf[6][4][4], int i_qp )
{
    const int i_qbits = i_qp/6 - 5;

    if( i_qbits >= 0 )
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0] << i_qbits;
        dct[0][0] *= i_dmf;
        dct[0][1] *= i_dmf;
        dct[1][0] *= i_dmf;
        dct[1][1] *= i_dmf;
    }
    else
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0];
        // chroma DC is truncated, not rounded
        dct[0][0] = ( dct[0][0] * i_dmf ) >> (-i_qbits);
        dct[0][1] = ( dct[0][1] * i_dmf ) >> (-i_qbits);
        dct[1][0] = ( dct[1][0] * i_dmf ) >> (-i_qbits);
        dct[1][1] = ( dct[1][1] * i_dmf ) >> (-i_qbits);
    }
}

void x264_mb_dequant_4x4_dc( INT16 dct[4][4], int dequant_mf[6][4][4], int i_qp )
{
    const int i_qbits = i_qp/6 - 6;
    int y;

    if( i_qbits >= 0 )
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0] << i_qbits;

        for( y = 0; y < 4; y++ )
        {
            dct[y][0] *= i_dmf;
            dct[y][1] *= i_dmf;
            dct[y][2] *= i_dmf;
            dct[y][3] *= i_dmf;
        }
    }
    else
    {
        const int i_dmf = dequant_mf[i_qp%6][0][0];
        const int f = 1 << (-i_qbits-1);

        for( y = 0; y < 4; y++ )
        {
            dct[y][0] = ( dct[y][0] * i_dmf + f ) >> (-i_qbits);
            dct[y][1] = ( dct[y][1] * i_dmf + f ) >> (-i_qbits);
            dct[y][2] = ( dct[y][2] * i_dmf + f ) >> (-i_qbits);
            dct[y][3] = ( dct[y][3] * i_dmf + f ) >> (-i_qbits);
        }
    }
}

void x264_quant_init( x264_t *h, int cpu, x264_quant_function_t *pf )
{
    int i, maxQ8=0, maxQ4=0, maxQdc=0;

    pf->quant_8x8_core = quant_8x8_core;
    pf->quant_4x4_core = quant_4x4_core;
    pf->quant_4x4_dc_core = quant_4x4_dc_core;
    pf->quant_2x2_dc_core = quant_2x2_dc_core;

    pf->dequant_4x4 = dequant_4x4;
    pf->dequant_8x8 = dequant_8x8;


}
