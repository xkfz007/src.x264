#ifndef _FRAME_H_
#define _FRAME_H_
#include "common.h"
typedef struct x264_frame{
    int     i_poc;
    int     i_delta_poc[2];
    int     i_type;
    int     i_qpplus1;
}x264_frame_t;

/* synchronized frame list */
typedef struct
{
   x264_frame_t **list;
   int i_max_size;
   int i_size;
   x264_pthread_mutex_t     mutex;
   x264_pthread_cond_t      cv_fill;  /* event signaling that the list became fuller */
   x264_pthread_cond_t      cv_empty; /* event signaling that the list became emptier */
} x264_sync_frame_list_t;
int           x264_sync_frame_list_init( x264_sync_frame_list_t *slist, int nelem );
void          x264_sync_frame_list_delete( x264_sync_frame_list_t *slist );
void          x264_sync_frame_list_push( x264_sync_frame_list_t *slist, x264_frame_t *frame );
x264_frame_t *x264_sync_frame_list_pop( x264_sync_frame_list_t *slist );
#endif