#ifndef ___CONFIG_H__
#define ___CONFIG_H__
 

 
#define X264_VERSION "2245"
#define PRIx32         "x"
#define PRId32         "d"
#define HAVE_VISUALIZE 0
#define HAVE_THREAD    1
#define HAVE_WIN32THREAD 1
#define HAVE_GPL       1
#define SYS_WINDOWS    1
#define HAVE_INTERLACED 0 //TODO

//#define HAVE_MMX  1

#define HAVE_STRING_H 1
#ifdef _WIN32_WINNT
#undef _WIN32_WINNT
#endif
#define _WIN32_WINNT 0x0501 

#ifdef WIN32_LEAN_AND_MEAN
#undef WIN32_LEAN_AND_MEAN
#endif // WIN32_LEAN_AND_MEAN


#endif // ___CONFIG_H__