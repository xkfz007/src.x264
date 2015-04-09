#ifndef _STDINT_TYPES_H_
#define _STDINT_TYPES_H_
/* 7.18.1.1  Exact-width integer types */

typedef signed char INT8;
typedef unsigned char   UINT8;
typedef short  INT16;
typedef unsigned short  UINT16;
typedef int  INT32;
typedef unsigned   UINT32;
#ifdef _MSC_VER
typedef __int64  INT64;
typedef unsigned __int64 UINT64;
#else
typedef long long  INT64;
typedef unsigned long long UINT64;
#endif
#endif