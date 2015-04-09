#ifndef _RDO_H_
#define _RDO_H_

int x264_rd_cost_mb( x264_t *h, int i_lambda2 );
int ssd_mb( x264_t *h );
void x264_rdo_init( );
void x264_quant_4x4_trellis( x264_t *h, INT16 dct[4][4], int i_quant_cat,
                             int i_qp, int i_ctxBlockCat, int b_intra );
void x264_quant_8x8_trellis( x264_t *h, INT16 dct[8][8], int i_quant_cat,
                             int i_qp, int b_intra );
#endif