#ifndef _STATISTICS_H
#define _STATISTICS_H

#include <time.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

typedef struct stat {
  unsigned int count;
  struct timespec total_time;
  struct timespec start;
} stat_t;

void
statReset(stat_t * s);

void
statStart(stat_t * s);

void
statStop(stat_t * s);

void
statPrint(const stat_t * s);

#ifdef __cplusplus /* If this is a C++ compiler, end C linkage */
}
#endif

#endif /* _STATISTICS_H */
