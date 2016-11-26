// -*- C++ -*-
//
// $Id

#include<cmath>

#if !defined(__format_h__)
#define __format_h__

#if !defined(u_char)
#define u_char unsigned char
#endif

#if !defined(u_int)
#define u_int unsigned int
#endif

#if !defined(__GNUC__)
double rint(double val);
#endif

#include <cstdio>
#include <cstdarg>
#include <string>

using namespace std;

//
// string format(const char * form , ... );
//
// Purpose:
//   To generate fprintf-like formatted strings
//
#if defined(__GNUC__)
static inline
string format(const char * form , ... ) __attribute__ ((format (printf, 1, 2)));
#endif

static inline
string format(const char * form , ... ) {
  const int myBUFF_SIZE = 1024;
  char most2long1s2short[myBUFF_SIZE];
  va_list ap;
  va_start(ap,form);
#if defined(__GNUC__)
  // have vsnprintf
  if(vsnprintf(most2long1s2short,myBUFF_SIZE,form,ap) == -1)
    most2long1s2short[myBUFF_SIZE - 1] =0x00;
  return string(most2long1s2short);
#else
  most2long1s2short[myBUFF_SIZE] =0x00;
  vsprintf(most2long1s2short,form,ap);
  return string(most2long1s2short);
#endif
}

#endif
