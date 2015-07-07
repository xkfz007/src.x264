
#include "common.h"
x264_frame_t *x264_frame_shift( x264_frame_t **list )
{
    x264_frame_t *frame = list[0];
    int i;
    for( i = 0; list[i]; i++ )
        list[i] = list[i+1];
    assert(frame);
    return frame;
}

void x264_frame_delete( x264_frame_t *frame )
{
    /* Duplicate frames are blank copies of real frames (including pointers),
     * so freeing those pointers would cause a double free later. */
//    if( !frame->b_duplicate )
//    {
//        x264_free( frame->base );
//
//        if( frame->param && frame->param->param_free )
//            frame->param->param_free( frame->param );
//        if( frame->mb_info_free )
//            frame->mb_info_free( frame->mb_info );
//        if( frame->extra_sei.sei_free )
//        {
//            for( int i = 0; i < frame->extra_sei.num_payloads; i++ )
//                frame->extra_sei.sei_free( frame->extra_sei.payloads[i].payload );
//            frame->extra_sei.sei_free( frame->extra_sei.payloads );
//        }
//        x264_pthread_mutex_destroy( &frame->mutex );
//        x264_pthread_cond_destroy( &frame->cv );
//#if HAVE_OPENCL
//        x264_opencl_frame_delete( frame );
//#endif
//    }
    x264_free( frame );
}
void x264_frame_delete_list( x264_frame_t **list )
{
    int i = 0;
    if( !list )
        return;
    while( list[i] )
        x264_frame_delete( list[i++] );
    x264_free( list );
}
void x264_sync_frame_list_push( x264_sync_frame_list_t *slist, x264_frame_t *frame )
{
    x264_pthread_mutex_lock( &slist->mutex );
    while( slist->i_size == slist->i_max_size )
        x264_pthread_cond_wait( &slist->cv_empty, &slist->mutex );
    slist->list[ slist->i_size++ ] = frame;
    x264_pthread_mutex_unlock( &slist->mutex );
    x264_pthread_cond_broadcast( &slist->cv_fill );
}

void x264_sync_frame_list_delete( x264_sync_frame_list_t *slist )
{
    x264_pthread_mutex_destroy( &slist->mutex );
    x264_pthread_cond_destroy( &slist->cv_fill );
    x264_pthread_cond_destroy( &slist->cv_empty );
    x264_frame_delete_list( slist->list );
}
int x264_sync_frame_list_init( x264_sync_frame_list_t *slist, int max_size )
{
    if( max_size < 0 )
        return -1;
    slist->i_max_size = max_size;
    slist->i_size = 0;
    CHECKED_MALLOCZERO( slist->list, (max_size+1) * sizeof(x264_frame_t*) );
    if( x264_pthread_mutex_init( &slist->mutex, NULL ) ||
        x264_pthread_cond_init( &slist->cv_fill, NULL ) ||
        x264_pthread_cond_init( &slist->cv_empty, NULL ) )
        return -1;
    return 0;
fail:
    return -1;
}
x264_frame_t *x264_sync_frame_list_pop( x264_sync_frame_list_t *slist )
{
    x264_frame_t *frame;
    x264_pthread_mutex_lock( &slist->mutex );
    while( !slist->i_size )
        x264_pthread_cond_wait( &slist->cv_fill, &slist->mutex );
    frame = slist->list[ --slist->i_size ];
    slist->list[ slist->i_size ] = NULL;
    x264_pthread_cond_broadcast( &slist->cv_empty );
    x264_pthread_mutex_unlock( &slist->mutex );
    return frame;
}