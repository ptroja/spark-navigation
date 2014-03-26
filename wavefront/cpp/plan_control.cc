/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003
 *     Andrew Howard
 *     Brian Gerkey    
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cassert>

#include "plan.h"

#if !defined (M_PI)
  #include <libplayercommon/playercommon.h>
#endif

static double _angle_diff(double a, double b);

bool
plan_t::check_done(double lx, double ly, double la,
                   double gx, double gy, double ga,
                   double goal_d, double goal_a) const
{
  double dt, da;
  dt = hypot(gx-lx,gy-ly);
  da = std::abs(_angle_diff(ga,la));

  return ((dt < goal_d) && (da < goal_a));
}

int
plan_t::compute_diffdrive_cmds(double* vx, double *va,
                               int* rotate_dir,
                               double lx, double ly, double la,
                               double gx, double gy, double ga,
                               double goal_d, double goal_a,
                               double maxd, double dweight,
                               double tvmin, double tvmax,
                               double avmin, double avmax,
                               double amin, double amax)
{
  double cx, cy;
  double d,b,a,ad;
  
  //puts("*******plan_compute_diffdrive_cmds************");
  
  // Are we at the goal?
  if(check_done(lx,ly,la,gx,gy,ga,goal_d,goal_a))
  {
    *vx = 0.0;
    *va = 0.0;
    return(0);
  }

  // Are we on top of the goal?
  d = hypot(gx-lx,gy-ly);
  //printf("d: %.3f\n", d);
  if(d < goal_d)
  {
    ad = _angle_diff(ga,la);
    if(!*rotate_dir)
    {
      if(ad < 0)
        *rotate_dir = -1;
      else
        *rotate_dir = 1;
    }
    *vx = 0.0;
    *va = *rotate_dir * (avmin + (fabs(ad)/M_PI) * (avmax-avmin));
    //printf("on top; vx:%.3f va: %.3f\n", *vx, *va);
    return(0);
  }

  *rotate_dir = 0;

  // We're away from the goal; compute velocities
  if(get_carrot(&cx, &cy, lx, ly, maxd, dweight) < 0.0)
  {
    //puts("no carrot");
    return(-1);
  }

  d = hypot(lx-cx,ly-cy);
  b = atan2(cy - ly, cx - lx);
  a = amin + (d / maxd) * (amax-amin);
  //printf("a: %.3f\n", a*180.0/M_PI);
  ad = _angle_diff(b,la);
  //printf("ad: %.3f\n", ad*180.0/M_PI);

  if(fabs(ad) > a)
    *vx = 0.0;
  else
    *vx = tvmin + (d / maxd) * (tvmax-tvmin);
  *va = avmin + (fabs(ad)/M_PI) * (avmax-avmin);
  if(ad < 0)
    *va = -*va;

  //printf("away; vx:%.3f va: %.3f\n", *vx, *va);
  return(0);
}

double
plan_t::get_carrot(double* px, double* py,
                   double lx, double ly, double maxdist, double distweight)
{
  plan_cell_t* cell, *ncell;
  int li, lj;
  double dist, d;
  double cost, bestcost;
  char old_occ_state;
  float old_occ_dist;

  li = PLAN_GXWX(lx);
  lj = PLAN_GYWY(ly);

  cell = cells + PLAN_INDEX(li,lj);

  // Latch and clear the obstacle state for the cell I'm in
  cell = cells + PLAN_INDEX(li, lj);
  old_occ_state = cell->occ_state_dyn;
  old_occ_dist = cell->occ_dist_dyn;
  cell->occ_state_dyn = -1;
  cell->occ_dist_dyn = (float) (max_radius);

  // Step back from maxdist, looking for the best carrot
  bestcost = -1.0;
  for(dist = maxdist; dist >= scale; dist -= scale)
  {
    // Find a point the required distance ahead, following the cost gradient
    d=scale;
    for(ncell = cell;
        (ncell->plan_next && (d < dist));
        ncell = ncell->plan_next, d+=scale);

    // Check whether the straight-line path is clear
    if((cost = check_path(*cell, *ncell)) < 0.0)
    {
      //printf("no path from (%d,%d) to (%d,%d)\n",
             //cell->ci, cell->cj, ncell->ci, ncell->cj);
      continue;
    }

    // Weight distance
    cost += distweight * (1.0/(dist*dist));
    if((bestcost < 0.0) || (cost < bestcost))
    {
      bestcost = cost;
      *px = PLAN_WXGX(ncell->ci);
      *py = PLAN_WYGY(ncell->cj);
    }
  }
 
  // Restore the obstacle state for the cell I'm in
  cell = cells + PLAN_INDEX(li, lj);
  cell->occ_state_dyn = old_occ_state;
  cell->occ_dist_dyn = old_occ_dist;

  return(bestcost);
}

double
plan_t::check_path(const plan_cell_t & s, const plan_cell_t & g) const
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;
  int obscost=0;

  x0 = s.ci;
  y0 = s.cj;
  
  x1 = g.ci;
  y1 = g.cj;

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(cells[PLAN_INDEX(y,x)].occ_dist_dyn < abs_min_radius)
      return -1;
    else if(cells[PLAN_INDEX(y,x)].occ_dist_dyn < max_radius)
      obscost += (int) (dist_penalty *
              (max_radius -
               cells[PLAN_INDEX(y,x)].occ_dist_dyn));
  }
  else
  {
    if(cells[PLAN_INDEX(x,y)].occ_dist_dyn < abs_min_radius)
      return -1;
    else if(cells[PLAN_INDEX(x,y)].occ_dist_dyn < max_radius)
      obscost += (int) (dist_penalty *
              (max_radius -
               cells[PLAN_INDEX(x,y)].occ_dist_dyn));
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(cells[PLAN_INDEX(y,x)].occ_dist_dyn < abs_min_radius)
        return -1;
      else if(cells[PLAN_INDEX(y,x)].occ_dist_dyn < max_radius)
        obscost += (int) (dist_penalty *
                (max_radius -
                 cells[PLAN_INDEX(y,x)].occ_dist_dyn));
    }
    else
    {
      if(cells[PLAN_INDEX(x,y)].occ_dist_dyn < abs_min_radius)
        return -1;
      else if(cells[PLAN_INDEX(x,y)].occ_dist_dyn < max_radius)
        obscost += (int) (dist_penalty *
                (max_radius -
                 cells[PLAN_INDEX(x,y)].occ_dist_dyn));
    }
  }

  return(obscost);
}

#define ANG_NORM(a) atan2(sin((a)),cos((a)))
static double
_angle_diff(double a, double b)
{
  double d1, d2;
  a = ANG_NORM(a);
  b = ANG_NORM(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}
