#ifndef NDDLIB_UTILS_H
#define NDDLIB_UTILS_H

#include "nddLib.h"

#define NDDLIB_PND_OCCUPIED -1
#define NDDLIB_PND_FREE 0


#define NDD_FINDVALLEYS_START 1
#define NDD_FINDVALLEYS_GOT_I 2
#define NDD_FINDVALLEYS_GOT_J_FOR_FIRST_BEGIN_DISC 3
#define NDD_FINDVALLEYS_ONE_BEGIN_DISC 4
#define NDD_FINDVALLEYS_BEGIN_DISC_MARKED 5
#define NDD_FINDVALLEYS_SEARCH_J_FOR_END_DISC 6
#define NDD_FINDVALLEYS_GOT_J_FOR_END_DISC 7
#define NDD_FINDVALLEYS_END_DISC_MARKED 8
#define NDD_FINDVALLEYS_SEARCH_J_FOR_NEXT_DISC 9
#define NDD_FINDVALLEYS_GOT_J_FOR_NEXT_DISC 10
#define NDD_FINDVALLEYS_LOWERING_DISC_DETECTED 11
#define NDD_FINDVALLEYS_END 12

/* local structures  for fillPND */
typedef struct nddFloatList {
  float v;
  struct nddFloatList *next;
} nddFloatList;


int nddSectorDistance (int d1, int d2, int n);
void nddFloatListFree(nddFloatList * list);
float nddEstimateDistToObsFromPND(int i, NDD_DIAGRAM * diag);
double nddEstimateDistToObstFromValues(nddFloatList* values_list);

void incJ (int *j, int modulo);
void incIincJ(int *i, int *j, int modulo);
int loweringDisc (int i, int j, NDD_DIAGRAM * diag);
int raisingDisc(int i, int j, NDD_DIAGRAM * diag);

void nddSplitNValleys(NDD_DIAGRAM *diag, int valleyToSplit, int width, 
		      int nSlices);
int nddHowManySplits(int width, int nsectors, int maxFrac);

void nddGetBbAndBe(int sb, int se, int nsects, int *bb, int *be);

void nddGetSiAndSjFromSelectedValley (NDD_DIAGRAM * diag, int*si, int*sj);
void nddGetSiAndSjFromSector(NDD_DIAGRAM * diag, int sector, int*si, int*sj);

double nddGiveAngleFromSector(NDD_SECTOR sector, int nSectors);

NDD_SEGMENT * nddFiltrateSegments(NDD_SEGMENT * segments, int n_segs, double dmax, int * newNSegs);

int nddComputeAlpha(double lmax, double secuDist, double dDisc, int nsects);
int nddComputeGamma(double dObs, int sectObs,
		    int sectDisc, double dEnlargement, 
		    int alpha, int nsects);
int nddComputeBeta(double dObs, int discObs,
		   int discGoal, double dEnlargement, int nsects);

int nddGetBissector(int s1, int s2, int nsects);

int nddIsGoalInValley(NDD_DIAGRAM * diag, NDD_POINT * goal);

int nddGetGoalSector(NDD_POINT * goal, int nsectors);


double nddGiveEnhacedSecurityDistance(int sector, int nSectors, 
				      double secuDist, double lmax_robot);


double nddGiveSpeedFactorFromAngle(double theta, int linFactor);

#endif
