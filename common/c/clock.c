#include <time.h>
#include <stdio.h>

#include "clock.h"

#define BILLION 1000000000

struct timespec
evAddTime(struct timespec addend1, struct timespec addend2) {
  struct timespec x;

  x.tv_sec = addend1.tv_sec + addend2.tv_sec;
  x.tv_nsec = addend1.tv_nsec + addend2.tv_nsec;
  if (x.tv_nsec >= BILLION) {
    x.tv_sec++;
    x.tv_nsec -= BILLION;
  }
  return (x);
}

struct timespec
evSubTime(struct timespec minuend, struct timespec subtrahend) {
  struct timespec x;

  x.tv_sec = minuend.tv_sec - subtrahend.tv_sec;
  if (minuend.tv_nsec >= subtrahend.tv_nsec)
    x.tv_nsec = minuend.tv_nsec - subtrahend.tv_nsec;
  else {
    x.tv_nsec = BILLION - subtrahend.tv_nsec + minuend.tv_nsec;
    x.tv_sec--;
  }
  return (x);
}

struct timespec
evNowTime(void)
{
  struct timespec tsnow;

  const clockid_t m = CLOCK_MONOTONIC;

  if (clock_gettime(m, &tsnow) == 0)
    return (tsnow);

  tsnow.tv_sec = tsnow.tv_nsec = 0;
  return (tsnow);
}
