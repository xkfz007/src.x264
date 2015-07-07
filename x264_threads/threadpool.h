/*****************************************************************************
 * threadpool.h: thread pooling
 *****************************************************************************
 * Copyright (C) 2010-2014 x264 project
 *
 * Authors: Steven Walters <kemuri9@gmail.com>
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#ifndef X264_THREADPOOL_H
#define X264_THREADPOOL_H
#include "frame.h"
//typedef struct x264_threadpool_t x264_threadpool_t;

typedef struct
{
    void *(*func)(void *);
    void *arg;
    void *ret;
} x264_threadpool_job_t;

typedef struct
{
    int            exit;
    int            threads;
    x264_pthread_t *thread_handle;
    void           (*init_func)(void *);
    void           *init_arg;

    /* requires a synchronized list structure and associated methods,
       so use what is already implemented for frames */
    x264_sync_frame_list_t uninit; /* list of jobs that are awaiting use */
    x264_sync_frame_list_t run;    /* list of jobs that are queued for processing by the pool */
    x264_sync_frame_list_t done;   /* list of jobs that have finished processing */
} x264_threadpool_t ;

#if HAVE_THREAD
int   x264_threadpool_init( x264_threadpool_t **p_pool, int threads,
                            void (*init_func)(void *), void *init_arg );
void  x264_threadpool_run( x264_threadpool_t *pool, void *(*func)(void *), void *arg );
void *x264_threadpool_wait( x264_threadpool_t *pool, void *arg );
void  x264_threadpool_delete( x264_threadpool_t *pool );
#else
#define x264_threadpool_init(p,t,f,a) -1
#define x264_threadpool_run(p,f,a)
#define x264_threadpool_wait(p,a)     NULL
#define x264_threadpool_delete(p)
#endif

#endif
