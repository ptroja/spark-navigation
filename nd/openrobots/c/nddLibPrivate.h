#ifndef NDD_LIB_PRIVATE_H
#define NDD_LIB_PRIVATE_H

static NDD_BOOL nddIsNarrowValley (NDD_VALLEY * v, int max_n_sectors, int smax);
static NDD_BOOL nddIsWideValley (NDD_VALLEY * v, int max_n_sectors, int smax);


static int nddFillPndFromSegments(NDD_DIAGRAM * diag, NDD_SEGMENT * segments, 
			   int n_segs, NDD_POINT * lonelyPoints, int n_lPoints);

static int nddFillPndFromPoints(NDD_DIAGRAM * diag, NDD_POINT * points, int n_pt, 
			 double distMax);

static void nddFillRnd(NDD_DIAGRAM * diag,  float * enlargement);

static int nddFindValleys(NDD_DIAGRAM * diag, NDD_POINT * local_goal);
static void nddSelectValley(NDD_DIAGRAM * diag, NDD_POINT * local_goal);


static NDD_SECTOR nddSectorLowSafetyOneSide(NDD_DIAGRAM * diag, float * enlargement, double secuDist);
static NDD_SECTOR nddSectorLowSafetyTwoSide(NDD_DIAGRAM * diag, float * enlargement, double secuDist);
static NDD_SECTOR nddSectorHighSafetyGoalValley(NDD_DIAGRAM * diag,
					 NDD_POINT * loc_goal);
static NDD_SECTOR nddSectorLowSafetyGoalValley(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
					float * enlargement);

static LINEAR_SPEED nddLinearSpeedHighSafety(double theta, double vmax, int linFactor);
static LINEAR_SPEED nddLinearSpeedLowSafety(double theta, double vmax,
				     NDD_DIAGRAM * diag,
				     float security_distance,
					    float * enlargement, int linFactor);
static ROTATIONNAL_SPEED nddRotationnalVelocity(double theta, double wmax);

static NDD_STRATEGY nddChooseStrategy(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
			       int smax);


#endif /* NDD_LIB_H */
