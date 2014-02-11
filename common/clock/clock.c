#include <time.h>
#include <stdio.h>

#include "clock.h"

#define BILLION 1000000000

static const clockid_t m = CLOCK_PROCESS_CPUTIME_ID;

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

  if (clock_gettime(m, &tsnow) == 0)
    return (tsnow);

  tsnow.tv_sec = tsnow.tv_nsec = 0;
  return (tsnow);
}

void
statReset(stat_t * s)
{
  s->total_time.tv_sec = 0;
  s->total_time.tv_nsec = 0;
  s->count = 0;
}

void
statStart(stat_t * s)
{
  s->start = evNowTime();
}

void
statStop(stat_t * s)
{
  s->total_time = evAddTime(s->total_time, evSubTime(evNowTime(), s->start));

  s->count++;
}

void
statPrint(const stat_t * s)
{
  printf("Statistics: count = %u, total time = %lu.%09lu\n", s->count, s->total_time.tv_sec, s->total_time.tv_nsec);
}
