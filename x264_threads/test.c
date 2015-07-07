#include "common.h"

typedef struct job_t job_t;
struct job_t {
    int num;
    int b_thread_active;
    job_t*thread[3];
    int i_thread_phase;
};
static void x264_encoder_thread_init( job_t *h )
{
    //if( h->param.i_sync_lookahead )
        x264_lower_thread_priority( 10 );
}
void func(job_t* h){
    printf("this is num %d\n",h->num );
    Sleep(1000);
}
int en_end(x264_threadpool_t*pool,job_t*h,job_t* thread_current){
    if( h->b_thread_active )
    {
        h->b_thread_active = 0;
        if( (intptr_t)x264_threadpool_wait( pool, h ) )
            return -1;
    }
    return 0;
}
int main(){
    int i;
    x264_threadpool_t *pool;
    job_t *thread_current, *thread_prev, *thread_oldest;
    const int thread_num=3;
    job_t t[100]={0};
    job_t* h=&t[0];
    int i_thread_phase=0;
    h->thread[0]=h;
    for(i=1;i<thread_num;i++){
        h->thread[i]=&t[i];
    }
    for(i=1;i<thread_num;i++){
        *h->thread[i]=*h;
    }
    x264_threadpool_init( &pool, thread_num, x264_encoder_thread_init,h);
    
    for(i=0;i<100000;i++){
        thread_prev    = h->thread[ i_thread_phase ];
        i_thread_phase = (i_thread_phase + 1) % thread_num;
        thread_current = h->thread[ i_thread_phase ];
        thread_oldest  = h->thread[ (i_thread_phase + 1) % thread_num];
        h = thread_current;
        h->num=i;
        x264_threadpool_run( pool, (void*)func, h );
        h->b_thread_active=1;
        en_end(pool,thread_oldest,thread_current);

    }
    x264_threadpool_delete( pool );

    return 0;
}