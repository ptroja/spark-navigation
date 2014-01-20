with nddStruct; use nddStruct;

package nddLib is

--  #ifndef NDD_LIB_H
--  #define NDD_LIB_H
--
--  #include "nddStruct.h"
--
--  /* #define VERBOSE_NDDLIB  */
--
--  #define NDDLIB_EPSILON_ESTIMATE_FROM_PND 0.1
   NDDLIB_EPSILON_ESTIMATE_FROM_PND : constant Float := 0.1;
--
--  #define NDD_SPLIT_MAX_FRAC 6
--
--
--  #define NDDLIB_STRAT_LS1 0
--  #define NDDLIB_STRAT_LS2 1
--  #define NDDLIB_STRAT_HSGV 2
--  #define NDDLIB_STRAT_HSNV 3
--  #define NDDLIB_STRAT_HSWV 4
--  #define NDDLIB_STRAT_LSG 5
--  #define NDDLIB_STRAT_NO 6
--
--  #define NDD_BOOL int

   subtype NDD_BOOL is Boolean;

   --  #define NDD_STRATEGY int

   type NDD_STRATEGY is (NDDLIB_STRAT_LS1,
                         NDDLIB_STRAT_LS2,
                         NDDLIB_STRAT_HSGV,
                         NDDLIB_STRAT_HSNV,
                         NDDLIB_STRAT_HSWV,
                         NDDLIB_STRAT_LSG,
                         NDDLIB_STRAT_NO);

--  #define NDD_SECTOR int
--  #define LINEAR_SPEED double
--  #define ROTATIONNAL_SPEED double
--
   NDDLIB_LIMIT_ANGLE : constant Float := 2.0;
--
--  typedef struct NDD_SPEED_REF {
--    double v;
--    double w;
--  } NDD_SPEED_REF;

   type NDD_SPEED_REF is
      record
         v : Float; -- forward velocity [m/s]
         w : Float; -- angular velocity [rad/s]
      end record;

   type NDD_POINT is
      record
         x,y : Float; -- [m]
      end record;

--  typedef struct NDD_SEGMENT {
--    double r1, t1;
--    double r2, t2;
--  } NDD_SEGMENT;

   type NDD_SEGMENT is
      record
         r1,r2 : Float; -- distance?
         t1,t2 : Float; -- angle?
      end record;

--  float * nddComputeEnlargement(int nSect, float lmaxRobot, float secuDist,
--  			      float vmaxRobot, float timeStep);
--
--
--  int nddComputeRefFromPoints(NDD_DIAGRAM * diag,
--  			    NDD_POINT * points,
--  			    int n_pt,
--  			    double dMaxLocalMap,
--  			    float * enlargement,
--  			    double security_distance,
--  			    NDD_POINT * loc_goal,
--  			    double smax,
--  			    double vmax,
--  			    double wmax,
--  			    double *thetaRef,
--  			    NDD_SPEED_REF *sr, int linFactor);
--
--  int nddComputeRefFromSegments(NDD_DIAGRAM * diag,
--  			      NDD_SEGMENT * segments,
--  			      int n_segs,
--  			      NDD_POINT * lonelyPoints,
--  			      int n_lPoints,
--  			      double dMaxLocalMap,
--  			      float * enlargement,
--  			      double security_distance,
--  			      NDD_POINT * loc_goal,
--  			      double smax,
--  			      double vmax,
--  			      double wmax,
--  			      double *thetaRef,
--  			      NDD_SPEED_REF *sr, int linFactor);
--
--
--  #endif /* NDD_LIB_H */

   --  static NDD_BOOL nddIsNarrowValley (NDD_VALLEY * v, int max_n_sectors, int smax);
   --
   function nddIsNarrowValley (v : NDD_VALLEY;
                               max_n_sectors : Integer;
                               smax : Integer)
                               return Boolean;

   --  static NDD_BOOL nddIsWideValley (NDD_VALLEY * v, int max_n_sectors, int smax);
   --
   function nddIsWideValley (v : NDD_VALLEY;
                             max_n_sectors : Integer;
                             smax : Integer)
                             return NDD_BOOL;

--  static int nddFillPndFromSegments(NDD_DIAGRAM * diag, NDD_SEGMENT * segments,
--  			   int n_segs, NDD_POINT * lonelyPoints, int n_lPoints);

--   function nddFillPndFromSegments(diag : NDD_DIAGRAM;
--                                     segments : NDD_SEGMENT; -- array?
--                                     n_segs : Integer

--  			   int n_segs, NDD_POINT * lonelyPoints, int n_lPoints);

--
--  static int nddFillPndFromPoints(NDD_DIAGRAM * diag, NDD_POINT * points, int n_pt,
--  			 double distMax);
--
--  static void nddFillRnd(NDD_DIAGRAM * diag,  float * enlargement);

   procedure nddFillRnd(diag : in out NDD_DIAGRAM;
                        enlargement : in NDD_SECTOR_RANGES);
--
--  static int nddFindValleys(NDD_DIAGRAM * diag, NDD_POINT * local_goal);

   function nddFindValleys(diag : in out NDD_DIAGRAM) return Integer;
--  static void nddSelectValley(NDD_DIAGRAM * diag, NDD_POINT * local_goal);
--
--
--  static NDD_SECTOR nddSectorLowSafetyOneSide(NDD_DIAGRAM * diag, float * enlargement, double secuDist);
--  static NDD_SECTOR nddSectorLowSafetyTwoSide(NDD_DIAGRAM * diag, float * enlargement, double secuDist);
--  static NDD_SECTOR nddSectorHighSafetyGoalValley(NDD_DIAGRAM * diag,
--  					 NDD_POINT * loc_goal);
--  static NDD_SECTOR nddSectorLowSafetyGoalValley(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
--  					float * enlargement);
--
--  static LINEAR_SPEED nddLinearSpeedHighSafety(double theta, double vmax, int linFactor);
--  static LINEAR_SPEED nddLinearSpeedLowSafety(double theta, double vmax,
--  				     NDD_DIAGRAM * diag,
--  				     float security_distance,
--  					    float * enlargement, int linFactor);
--  static ROTATIONNAL_SPEED nddRotationnalVelocity(double theta, double wmax);
--
--  static NDD_STRATEGY nddChooseStrategy(NDD_DIAGRAM * diag, NDD_POINT * loc_goal,
--  			       int smax);
--
--
--  #endif /* NDD_LIB_H */
end nddLib;
