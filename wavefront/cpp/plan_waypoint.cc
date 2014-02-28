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


/**************************************************************************
 * Desc: Path planner: waypoint generation
 * Author: Andrew Howard
 * Date: 10 Oct 2002
 * CVS: $Id: plan_waypoint.c 9120 2013-01-07 00:18:52Z jpgr87 $
**************************************************************************/

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>

#include <libplayercommon/playercommon.h>

#include "plan.h"

// Generate a path to the goal
void plan_t::update_waypoints(double px, double py)
{
  double dist;
  int ni, nj;
  plan_cell_t *cell, *ncell;

  waypoint_count = 0;

  ni = PLAN_GXWX(this, px);
  nj = PLAN_GYWY(this, py);

  // Can't plan a path if we're off the map
  if(!PLAN_VALID(this,ni,nj))
    return;

  cell = cells + PLAN_INDEX(this, ni, nj);

  while (cell != NULL)
  {
    if (waypoint_count >= waypoint_size)
    {
      waypoint_size *= 2;
      waypoints = (plan_cell_t **) realloc(waypoints,
                                           waypoint_size * sizeof(waypoints[0]));
    }
    
    waypoints[waypoint_count++] = cell;

    if (cell->plan_next == NULL)
    {
      // done
      break;
    }

    // Find the farthest cell in the path that is reachable from the
    // currrent cell.
    dist = 0;
    for(ncell = cell; ncell->plan_next != NULL; ncell = ncell->plan_next)
    {
      if(dist > 0.50)
      {
        if(!test_reachable(cell, ncell->plan_next))
          break;
      }
      dist += scale;
    }
    if(ncell == cell)
    {
      break;
    }
    
    cell = ncell;
  }

  if(cell && (cell->plan_cost > 0))
  {
    // no path
    waypoint_count = 0;
  }
  
  return;
}


// Get the ith waypoint; returns non-zero of there are no more waypoints
int plan_t::get_waypoint(int i, double *px, double *py) const
{
  if (i < 0 || i >= waypoint_count)
    return 0;

  *px = PLAN_WXGX(this, waypoints[i]->ci);
  *py = PLAN_WYGY(this, waypoints[i]->cj);

  return 1;
}

// Convert given waypoint cell to global x,y
void plan_t::convert_waypoint(plan_cell_t *waypoint, double *px, double *py) const
{
  *px = PLAN_WXGX(this, waypoint->ci);
  *py = PLAN_WYGY(this, waypoint->cj);
}

// Test to see if once cell is reachable from another.
int plan_t::test_reachable(plan_cell_t *cell_a, plan_cell_t *cell_b) const
{
  double theta;
  double sinth, costh;
  double i,j;
  int lasti, lastj;

  theta = atan2((double)(cell_b->cj - cell_a->cj), 
                (double)(cell_b->ci - cell_a->ci));
  sinth = sin(theta);
  costh = cos(theta);

  lasti = lastj = -1;
  i = (double)cell_a->ci;
  j = (double)cell_a->cj;

  while((lasti != cell_b->ci) || (lastj != cell_b->cj))
  {
    if((lasti != (int)floor(i)) || (lastj != (int)floor(j)))
    {
      lasti = (int)floor(i);
      lastj = (int)floor(j);
      if(!PLAN_VALID(this,lasti,lastj))
      {
        //PLAYER_WARN("stepped off the map!");
        return(0);
      }
      if(cells[PLAN_INDEX(this,lasti,lastj)].occ_dist <
         abs_min_radius)
        return(0);
    }
    
    if(lasti != cell_b->ci)
      i += costh;
    if(lastj != cell_b->cj)
      j += sinth;
  }
  return(1);
}

#if 0
// Test to see if once cell is reachable from another.
// This could be improved.
int plan_t::test_reachable(plan_cell_t *cell_a, plan_cell_t *cell_b) const
{
  int i, j;
  int ai, aj, bi, bj;
  double ox, oy, oa;
  double dx, dy;
  plan_cell_t *cell;

  ai = cell_a->ci;
  aj = cell_a->cj;
  bi = cell_b->ci;
  bj = cell_b->cj;

  ox = PLAN_WXGX(plan, ai);
  oy = PLAN_WYGY(plan, aj);
  oa = atan2(bj - aj, bi - ai);
  
  if (fabs(cos(oa)) > fabs(sin(oa)))
  {
    dy = tan(oa) * plan->scale;

    if (ai < bi)
    {
      for (i = ai; i < bi; i++)
      {
        j = PLAN_GYWY(plan, oy + (i - ai) * dy);
        if (PLAN_VALID(plan, i, j))
        {
          cell = plan->cells + PLAN_INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
    else
    {
      for (i = ai; i > bi; i--)
      {
        j = PLAN_GYWY(plan, oy + (i - ai) * dy);
        if (PLAN_VALID(plan, i, j))
        {
          cell = plan->cells + PLAN_INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
  }
  else
  {
    dx = tan(M_PI/2 - oa) * plan->scale;

    if (aj < bj)
    {
      for (j = aj; j < bj; j++)
      {
        i = PLAN_GXWX(plan, ox + (j - aj) * dx);
        if (PLAN_VALID(plan, i, j))
        {
          cell = plan->cells + PLAN_INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
    else
    {
      for (j = aj; j > bj; j--)
      {
        i = PLAN_GXWX(plan, ox + (j - aj) * dx);
        if (PLAN_VALID(plan, i, j))
        {
          cell = plan->cells + PLAN_INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
  }
  return 1;
}
#endif

