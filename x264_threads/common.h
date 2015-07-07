#ifndef _COMMON_H_
#define _COMMON_H_
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <stdio.h>

#include "config.h"
#include "stdint.h"
#include "win32thread.h"
#include "osdep.h"

#define X264_MIN(a,b) ( (a)<(b) ? (a) : (b) )
#define X264_MAX(a,b) ( (a)>(b) ? (a) : (b) )
#if (ARCH_X86 || STACK_ALIGNMENT > 16) && HAVE_MMX
intptr_t x264_stack_align( void (*func)(), ... );
#define x264_stack_align(func,...) x264_stack_align((void (*)())func, __VA_ARGS__)
#else
#define x264_stack_align(func,...) func(__VA_ARGS__)
#endif

/* For AVX2 */
#if ARCH_X86 || ARCH_X86_64
#define NATIVE_ALIGN 32
#define ALIGNED_N ALIGNED_32
#define ALIGNED_ARRAY_N ALIGNED_ARRAY_32
#else
#define NATIVE_ALIGN 16
#define ALIGNED_N ALIGNED_16
#define ALIGNED_ARRAY_N ALIGNED_ARRAY_16
#endif


#define X264_LOG_NONE          (-1)
#define X264_LOG_ERROR          0
#define X264_LOG_WARNING        1
#define X264_LOG_INFO           2
#define X264_LOG_DEBUG          3

void *x264_malloc( int );
void  x264_free( void * );

#define CHECKED_MALLOC( var, size )\
do {\
    var = x264_malloc( size );\
    if( !var )\
        goto fail;\
} while( 0 )
#define CHECKED_MALLOCZERO( var, size )\
do {\
    CHECKED_MALLOC( var, size );\
    memset( var, 0, size );\
} while( 0 )
#include "threadpool.h"
#endif