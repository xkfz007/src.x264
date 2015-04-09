#ifndef X264_COMMON_COMMON_H
#define X264_COMMON_COMMON_H

#include "extras/stdint.h"
//Moved from common.h
#if HIGH_BIT_DEPTH
typedef uint16_t pixel;
typedef uint64_t pixel4;
typedef int32_t  dctcoef;
typedef uint32_t udctcoef;

#   define PIXEL_SPLAT_X4(x) ((x)*0x0001000100010001ULL)
#   define MPIXEL_X4(src) M64(src)
#else
typedef uint8_t  pixel;
typedef uint32_t pixel4;
typedef int16_t  dctcoef;
typedef uint16_t udctcoef;

#   define PIXEL_SPLAT_X4(x) ((x)*0x01010101U)
#   define MPIXEL_X4(src) M32(src)
#endif

//Moved from common.h
/* Unions for type-punning.
 * Mn: load or store n bits, aligned, native-endian
 * CPn: copy n bits, aligned, native-endian
 * we don't use memcpy for CPn because memcpy's args aren't assumed to be aligned */
typedef union { uint16_t i; uint8_t  c[2]; } MAY_ALIAS x264_union16_t;
typedef union { uint32_t i; uint16_t b[2]; uint8_t  c[4]; } MAY_ALIAS x264_union32_t;
typedef union { uint64_t i; uint32_t a[2]; uint16_t b[4]; uint8_t c[8]; } MAY_ALIAS x264_union64_t;
typedef struct { uint64_t i[2]; } x264_uint128_t;
typedef union { x264_uint128_t i; uint64_t a[2]; uint32_t b[4]; uint16_t c[8]; uint8_t d[16]; } MAY_ALIAS x264_union128_t;
#define M16(src) (((x264_union16_t*)(src))->i)
#define M32(src) (((x264_union32_t*)(src))->i)
#define M64(src) (((x264_union64_t*)(src))->i)
#define M128(src) (((x264_union128_t*)(src))->i)


#endif