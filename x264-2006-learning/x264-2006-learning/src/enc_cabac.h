#ifndef _ENC_CABAC_H_
#define _ENC_CABAC_H_
static const int significant_coeff_flag_offset[6] = { 105, 120, 134, 149, 152, 402 };
static const int last_coeff_flag_offset[6] = { 166, 181, 195, 210, 213, 417 };
static const int coeff_abs_level_m1_offset[6] = { 227, 237, 247, 257, 266, 426 };
static const int significant_coeff_flag_offset_8x8[63] = {
    0, 1, 2, 3, 4, 5, 5, 4, 4, 3, 3, 4, 4, 4, 5, 5,
    4, 4, 4, 4, 3, 3, 6, 7, 7, 7, 8, 9,10, 9, 8, 7,
    7, 6,11,12,13,11, 6, 7, 8, 9,14,10, 9, 8, 6,11,
   12,13,11, 6, 9,14,10, 9,11,12,13,11,14,10,12
};
static const int last_coeff_flag_offset_8x8[63] = {
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8
};
#endif