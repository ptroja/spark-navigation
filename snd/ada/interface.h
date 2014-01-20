#ifndef _INTERFACE_H
#define _INTERFACE_H

extern "C" {

void ndinit(void);
void ndfinal(void);

struct proxy_c {
  double robot_radius;
  double min_gap_width;
  double obstacle_avoid_dist;
  double max_speed;
  double max_turn_rate;
  double goal_position_tol;
  double goal_angle_tol;
  double goalX, goalY, goalA;

  void * robot_proxy_ptr;

  unsigned int (*getScanCount)(void *);
  double (*getScanRes)(void *);
  double (*getMaxRange)(void *);
  double (*getRange)(void *, unsigned int);
  double (*getXPos)(void *);
  double (*getYPos)(void *);
  double (*getYaw)(void *);

  int    (*isNewGoalData)(void *);
  int    (*PeekInputData)(void *);
  void   (*setSpeed)(void *, double, double);
  void   (*goalReached)(void *);
};

void
step_c(struct proxy_c *);

} /* extern "C" */

#endif /* _INTERFACE_H */
