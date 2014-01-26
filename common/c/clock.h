#ifndef _CLOCK_H
#define _CLOCK_H

#include <time.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

struct timespec
evAddTime(struct timespec addend1, struct timespec addend2);

struct timespec
evSubTime(struct timespec minuend, struct timespec subtrahend);

struct timespec
evNowTime(void);

#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif

#endif /* _CLOCK_H */
