#ifndef NDD_CONST_H
#define NDD_CONST_H

#define NDD_DIST_MAX_SUBGOAL 2.0

#   define NDD_MAX_LIN_SPEED 0
#   define NDD_MAX_ANG_SPEED 0
#   define NDD_MAX_LIN_ACCEL 0
#   define NDD_MAX_ANG_ACCEL 0
#   define NDD_DEFAULT_LMAX_ROBOT 1.0
    /* security distance around the sick */
    /* making it too big will make the robot go really too far from obstacles */
    /* this distance is around the SICK!!! try to make it at least 20~30 cms */
    /* from obstacles */
    /* to have a good motion the max angular speed should be high*/
    /* a good one is 0.25 for linear and 0.2 for angular (1.0 for secu_dist)*/
    /* or    linear: 0.5 angular: 0.5 secu: 1.3 */
#   define NDD_DEFAULT_SECURITY_DISTANCE 0.6

/* internal parameter, needs recompile */
/* Defines the angular resolution for internal valleys representation */
/* Perhaps better and more precise with more sectors, but never test with other value */
#define NDD_NSECTORS 144

/* dinamycally changeable parameters*/

/* data flow connexions */
#define NDD_DEFAULT_OBST_TYPE NDD_INPUT_POLAR_SEGS  /* NDD_INPUT_CART_SEGS */
#define NDD_DEFAULT_OBST_POSTER "aspectPolarAspect" /* aspectCartesianAspect */
#define NDD_DEFAULT_TRAJ_POSTER "vstpposter"
#define NDD_DEFAULT_POS_POSTER "pomPos"
/* navigations stop when at that distance from the goal */
#define NDD_DEFAULT_DIST_TO_GOAL 0.50

/* threshold between wide and narrow valleys */
#define NDD_DEFAULT_SMAX (NDD_NSECTORS / 8) 
#define NDD_DEFAULT_LIN_FACTOR 3
#define NDD_DEFAULT_ENLARGEMENT_FACTOR 1.0


/* how often will the main function be called ?? */
/* XXXXXXXXXXXXXXXXX
   #define NDD_TICS_PERIOD 20 */  /* Period in tics (1 tic is 10 ms) */
/* for jido */
#define NDD_TICS_PERIOD 10   /* Period in tics (1 tic is 10 ms) */

/* dmax_sensor, change at your own risks */
#define NDD_DMAX_SICK 32

/* how large will be the map around the robot?  */
/* you can safely use a huge length, computational time and memory */
/* will not really increase thanks to aspect */
#define NDD_DEFAULT_DMAXLOCALMAP 10.0

#define NDD_DEFAULT_EPSILON_THETA 0.2

#define NDD_DEFAULT_THETA_USE NDD_USE_THETA

#define NDD_DEFAULT_DISTSWITCHTRAJ 1.0

#define NDD_BLOCKED_THRESHOLD_NSECS 10  /* 50 -> 20 secondes */

#define NDD_BLOCKED_THRESHOLD_METERS 0.1

#define NDD_DEFAULT_MAXDECEL 0.05

#define NDD_DEFAULT_STOPTHRESHOLD 0.01

#endif  /* NDD_CONST_H */
