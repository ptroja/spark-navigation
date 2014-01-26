#ifndef _CLOCK_H
#define _CLOCK_H

#include <time.h>

struct timespec
evAddTime(struct timespec addend1, struct timespec addend2);

struct timespec
evSubTime(struct timespec minuend, struct timespec subtrahend);

struct timespec
evNowTime(void);

#endif /* _CLOCK_H */
