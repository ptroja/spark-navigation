#ifndef NDD_LIB_H
#define NDD_LIB_H

#include "nddStruct.h"

/* #define VERBOSE_NDDLIB  */

#define NDDLIB_EPSILON_ESTIMATE_FROM_PND 0.1

#define NDD_SPLIT_MAX_FRAC 6


#define NDDLIB_STRAT_LS1 0
#define NDDLIB_STRAT_LS2 1
#define NDDLIB_STRAT_HSGV 2
#define NDDLIB_STRAT_HSNV 3
#define NDDLIB_STRAT_HSWV 4
#define NDDLIB_STRAT_LSG 5
#define NDDLIB_STRAT_NO 6

#define NDD_BOOL int
#define NDD_STRATEGY int
#define NDD_SECTOR int
#define LINEAR_SPEED double
#define ROTATIONNAL_SPEED double

#define NDDLIB_LIMIT_ANGLE 2.0

typedef struct NDD_SPEED_REF {
  double v;
  double w;
} NDD_SPEED_REF;

typedef struct NDD_POINT {
  double x;
  double y;
} NDD_POINT;

typedef struct NDD_SEGMENT {
  double r1, t1;
  double r2, t2;
} NDD_SEGMENT;

float * nddComputeEnlargement(int nSect, float lmaxRobot, float secuDist,
			      float vmaxRobot, float timeStep);


int nddComputeRefFromPoints(NDD_DIAGRAM * diag, 
			    NDD_POINT * points, 
			    int n_pt, 
			    double dMaxLocalMap,
			    float * enlargement,
			    double security_distance, 
			    NDD_POINT * loc_goal,
			    double smax, 
			    double vmax,
			    double wmax,
			    double *thetaRef,
			    NDD_SPEED_REF *sr, int linFactor);

int nddComputeRefFromSegments(NDD_DIAGRAM * diag, 
			      NDD_SEGMENT * segments,	
			      int n_segs, 
			      NDD_POINT * lonelyPoints,
			      int n_lPoints,
			      double dMaxLocalMap,
			      float * enlargement,
			      double security_distance, 
			      NDD_POINT * loc_goal,
			      double smax, 
			      double vmax,
			      double wmax,
			      double *thetaRef,
			      NDD_SPEED_REF *sr, int linFactor);


#endif /* NDD_LIB_H */
