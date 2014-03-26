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

#include <cmath>

#include <libplayercommon/playercommon.h>

#include "plan.h"

// Generate a path to the goal
void plan_t::update_waypoints(const pos2d<double> & p)
{
  waypoints.clear();

  int ni, nj;

  ni = GXWX(p.x);
  nj = GYWY(p.y);

  // Can't plan a path if we're off the map
  if(!VALID(ni,nj))
    return;

  plan_cell_t *cell = cells + INDEX(ni, nj);

  while (cell != NULL)
  {
    waypoints.push_back(cell);

    if (cell->plan_next == NULL)
    {
      // done
      break;
    }

    // Find the farthest cell in the path that is reachable from the
    // current cell.
    double dist = 0.0;
    plan_cell_t *ncell;
    for(ncell = cell; ncell->plan_next != NULL; ncell = ncell->plan_next)
    {
      if(dist > 0.50)
      {
        if(!test_reachable(*cell, *ncell->plan_next))
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
    waypoints.clear();
  }
  
  return;
}


// Get the i-th waypoint; returns false if there are no more waypoints
bool plan_t::get_waypoint(waypoints_t::size_type i, double *px, double *py) const
{
  if (i >= waypoints.size())
    return false;

  *px = WXGX(waypoints[i]->ci);
  *py = WYGY(waypoints[i]->cj);

  return true;
}

// Convert given waypoint cell to global x,y
void plan_t::convert_waypoint(const plan_cell_t & waypoint, double *px, double *py) const
{
  *px = WXGX(waypoint.ci);
  *py = WYGY(waypoint.cj);
}

// See if once cell is reachable from another.
bool plan_t::test_reachable(const plan_cell_t & cell_a, const plan_cell_t & cell_b) const
{
  double theta;
  double sinth, costh;
  double i,j;
  int lasti, lastj;

  theta = atan2((double)(cell_b.cj - cell_a.cj),
                (double)(cell_b.ci - cell_a.ci));
  // FIXME: use GNU sincos if available
  sinth = sin(theta);
  costh = cos(theta);

  lasti = lastj = -1;
  i = (double)cell_a.ci;
  j = (double)cell_a.cj;

  while((lasti != cell_b.ci) || (lastj != cell_b.cj))
  {
    if((lasti != (int)floor(i)) || (lastj != (int)floor(j)))
    {
      lasti = (int)floor(i);
      lastj = (int)floor(j);
      if(!VALID(lasti,lastj))
      {
        //PLAYER_WARN("stepped off the map!");
        return false;
      }
      if(cells[INDEX(lasti,lastj)].occ_dist <
         abs_min_radius)
        return false;
    }
    
    if(lasti != cell_b.ci)
      i += costh;
    if(lastj != cell_b.cj)
      j += sinth;
  }
  return true;
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

  ox = WXGX(plan, ai);
  oy = WYGY(plan, aj);
  oa = atan2(bj - aj, bi - ai);
  
  if (fabs(cos(oa)) > fabs(sin(oa)))
  {
    dy = tan(oa) * plan->scale;

    if (ai < bi)
    {
      for (i = ai; i < bi; i++)
      {
        j = GYWY(plan, oy + (i - ai) * dy);
        if (plan->VALID(i, j))
        {
          cell = plan->cells + INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
    else
    {
      for (i = ai; i > bi; i--)
      {
        j = GYWY(plan, oy + (i - ai) * dy);
        if (plan->VALID(i, j))
        {
          cell = plan->cells + INDEX(plan, i, j);
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
        i = GXWX(plan, ox + (j - aj) * dx);
        if (VALID(plan, i, j))
        {
          cell = plan->cells + INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
    else
    {
      for (j = aj; j > bj; j--)
      {
        i = GXWX(plan, ox + (j - aj) * dx);
        if (VALID(plan, i, j))
        {
          cell = plan->cells + INDEX(plan, i, j);
          if (cell->occ_dist < plan->abs_min_radius)
            return 0;
        }
      }
    }
  }
  return 1;
}
#endif

