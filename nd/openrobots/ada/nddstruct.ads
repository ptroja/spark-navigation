with nddConst; use nddConst;

package nddStruct is

--  #ifndef NDD_STRUCT_H
--  #define NDD_STRUCT_H
--
--  #include "nddConst.h"
--
--  typedef struct NDD_SPEED {
--    double linear;
--    double angular;
--    double accelLinear;
--    double accelAngular;
--  } NDD_SPEED;
--
--  typedef enum NDD_OBST_TYPE {
--    NDD_INPUT_CART_POINTS,
--    NDD_INPUT_POLAR_POINTS,
--    NDD_INPUT_DEBUG,
--    NDD_INPUT_POLAR_SEGS,
--    NDD_INPUT_CART_SEGS     /* the best when it will work but still BUGGY */
--  } NDD_OBST_TYPE;
--
--  typedef enum NDD_USING_THETA {
--    NDD_USE_THETA,
--    NDD_DONT_USE_THETA
--  } NDD_USING_THETA;
--
--  #define NDD_NAME_LENGTH 64
--
--  typedef struct NDD_INPUT_POSTERS {
--    NDD_OBST_TYPE obstType;
--    int unused;
--    char obstPosterName[NDD_NAME_LENGTH];
--    char currentPosPosterName[NDD_NAME_LENGTH];
--  } NDD_INPUT_POSTERS;
--
--  typedef struct NDD_STRING{
--    char name[NDD_NAME_LENGTH];
--  } NDD_STRING;
--
--
--  typedef struct NDD_PARAMS {
--    double distToGoal;       /* From this distance it decelerates */
--    double epsilonTheta;     /* Threshold to stop final rotation (if NDD_USE_THETA) */
--    double securityDistance; /* Min dist in front, either vlin = 0 */
--    double lmaxRobot;        /* Radius or diagonal of the robot, mainly for lateral obstacles */
--    double dMaxLocalMap;     /* Max distance from the obstacles  */
--    double distSwitchTraj;   /* Distance to an intermediate goal before switching to the next one  */
--    double decelMax;         /* Decceleration for final goal approach  */
--    int smax;                /* Unity: a number of sector (NDD uses a total of NDD_NSECTORS number of sectors).
--  			      Defines the limit between 'large' and 'narrow' valley  */
--    int linFactor;    /* if small (1) trajectory less smooth; if huge (eg, 5) large traj */
--    float enlargementFactor; /* [1..5]: nb of cycle to anticipate obstacles (ie, reachable area) */
--    NDD_USING_THETA useTheta;  /* if NDD_USE_THETA than oriente the robot according to angle ref once at final goal */
--  } NDD_PARAMS;
--
--  typedef struct NDD_VALLEY {
--    int begin;
--    int end;
--  } NDD_VALLEY;

   type NDD_VALLEY is
      record
         begin_idx, end_idx : Integer;
      end record;
--
--  typedef struct NDD_DIAGRAM {
--    float dmax_sensor;
--    float lmax_robot;
--    short nsectors;
--    short currentStrat;
--    short n_valleys;
--    short selected_valley;
--    NDD_VALLEY valleys[NDD_NSECTORS/2];
--    float pnd[NDD_NSECTORS];
--    float rnd[NDD_NSECTORS];
--  } NDD_DIAGRAM;

   -- Array types must be defined outside of record types.
   type NDD_VALLEYS is array (Integer range 0 .. NDD_NSECTORS/2) of NDD_VALLEY;
   type NDD_SECTOR_RANGES is array (Integer range 0 .. NDD_NSECTORS) of Float;

   type NDD_DIAGRAM is
      record
         dmax_sensor : Float;
         lmax_robot : Float;
         nsectors : Positive;
         currentStrat : Integer;
         n_valleys : Integer;
         selected_valley : Integer;
         valleys : NDD_VALLEYS;
         pnd : NDD_SECTOR_RANGES;
         rnd : NDD_SECTOR_RANGES;
      end record;
--
--
--  typedef struct NDD_CHOICE {
--    double motionDirection;   /* angle relatively to robot frame */
--    double valley;            /* direction selected by ndd */
--    double speedPercent;      /* linear speed in max percent */
--  } NDD_CHOICE;

   type NDD_CHOICE is
      record
         motionDirection : Float;
         valley : Float;
         speedPercent : Float range 0.0 .. 100.0;
      end record;

--  #endif /* NDD_STRUCT_H */

end nddStruct;
