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
 * Desc: Path planner: plan generation
 * Author: Andrew Howard
 * Date: 10 Oct 2002
 * CVS: $Id: plan_plan.c 9120 2013-01-07 00:18:52Z jpgr87 $
**************************************************************************/

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <climits>
#include <cfloat>

#include "plan.h"

int
plan_t::do_global(const pos2d<double> & l, const pos2d<double> & g)
{
  double t0,t1;

  t0 = get_time();

  // Set bounds to look over the entire grid
  set_bounds(0, 0, size.x - 1, size.y - 1);

  // Reset plan costs
  reset();

  path.clear();
  if(update_plan(l, g) < 0)
  {
    // no path
    return(-1);
  }

  int li, lj;
  li = PLAN_GXWX(l.x);
  lj = PLAN_GYWY(l.y);

  // Cache the path
  for(plan_cell_t * cell = cells + PLAN_INDEX(li,lj);
                    cell;
                    cell = cell->plan_next)
  {
    path.push_back(cell);
  }

  t1 = get_time();

  //printf("computed global path: %.6lf\n", t1-t0);

  return(0);
}

int
plan_t::do_local(const pos2d<double> & l, double plan_halfwidth)
{
  double t0 = get_time();

  // Set bounds as directed
  int xmin,ymin,xmax,ymax;

  xmin = PLAN_GXWX(l.x - plan_halfwidth);
  ymin = PLAN_GYWY(l.y - plan_halfwidth);
  xmax = PLAN_GXWX(l.x + plan_halfwidth);
  ymax = PLAN_GYWY(l.y + plan_halfwidth);
  set_bounds(xmin, ymin, xmax, ymax);

  // Reset plan costs (within the local patch)
  reset();

  // Find a local goal to pursue
  pos2d<double> g;

  if(find_local_goal(&g, l) != 0)
  {
    //puts("no local goal");
    return(-1);
  }

  //printf("local goal: %.3lf, %.3lf\n", gx, gy);

  lpath.clear();
  if(update_plan(l, g) != 0)
  {
    puts("local plan update failed");
    return(-1);
  }

  int li, lj;
  li = PLAN_GXWX(l.x);
  lj = PLAN_GYWY(l.y);

  // Reset path marks (TODO: find a smarter place to do this)
  for(int i=0;i<size.x*size.y;i++)
    cells[i].lpathmark = false;

  // Cache the path
  for(plan_cell_t * cell = cells + PLAN_INDEX(li,lj);
                    cell;
                    cell = cell->plan_next)
  {
    lpath.push_back(cell);
    cell->lpathmark = true;
  }

  double t1 = get_time();

  //printf("computed local path: %.6lf\n", t1-t0);
  return(0);
}


// Generate the plan
int 
plan_t::update_plan(const pos2d<double> & l, const pos2d<double> & g)
{
  int oi, oj, di, dj, ni, nj;
  int gi, gj, li,lj;
  float cost;
  plan_cell_t *cell, *ncell;
  char old_occ_state;
  float old_occ_dist;

  // Reset the queue
  // TODO: use C++11 swap with empty heap.
  while(!heap.empty()) heap.pop();

  // Initialize the goal cell
  gi = PLAN_GXWX(g.x);
  gj = PLAN_GYWY(g.y);

  // Initialize the start cell
  li = PLAN_GXWX(l.x);
  lj = PLAN_GYWY(l.y);

  //printf("planning from %d,%d to %d,%d\n", li,lj,gi,gj);

  if(!PLAN_VALID_BOUNDS(gi, gj))
  {
    puts("goal out of bounds");
    return(-1);
  }
  
  if(!PLAN_VALID_BOUNDS(li, lj))
  {
    puts("start out of bounds");
    return(-1);
  }

  // Latch and clear the obstacle state for the cell I'm in
  cell = cells + PLAN_INDEX(li, lj);
  old_occ_state = cell->occ_state_dyn;
  old_occ_dist = cell->occ_dist_dyn;
  cell->occ_state_dyn = -1;
  cell->occ_dist_dyn = (float) max_radius;

  cell = cells + PLAN_INDEX(gi, gj);
  cell->plan_cost = 0;

  // Are we done?
  if((li == gi) && (lj == gj))
    return(0);
  
  push(cell);

  while (true)
  {
    cell = pop();

    if (cell == NULL)
      break;

    oi = cell->ci;
    oj = cell->cj;

    //printf("pop %d %d %f\n", cell->ci, cell->cj, cell->plan_cost);

    float * p = dist_kernel_3x3;
    for (dj = -1; dj <= +1; dj++)
    {
      ncell = cells + PLAN_INDEX(oi-1,oj+dj);
      for (di = -1; di <= +1; di++, p++, ncell++)
      {
        if (di == 0 && dj == 0)
          continue;
        //if (di && dj)
          //continue;
        
        ni = oi + di;
        nj = oj + dj;

        if (!PLAN_VALID_BOUNDS(ni, nj))
          continue;

        if(ncell->mark)
          continue;

        if (ncell->occ_dist_dyn < abs_min_radius)
          continue;

        cost = cell->plan_cost;
        if(ncell->lpathmark)
          cost += (float) ((*p) * hysteresis_factor);
        else
          cost += *p;

        if(ncell->occ_dist_dyn < max_radius)
          cost += (float) (dist_penalty * (max_radius - ncell->occ_dist_dyn));

        if(cost < ncell->plan_cost)
        {
          ncell->plan_cost = cost;
          ncell->plan_next = cell;

          push(ncell);
        }
      }
    }
  }

  // Restore the obstacle state for the cell I'm in
  cell = cells + PLAN_INDEX(li, lj);
  cell->occ_state_dyn = old_occ_state;
  cell->occ_dist_dyn = old_occ_dist;

  if(!cell->plan_next)
  {
    //puts("never found start");
    return(-1);
  }
  else
    return(0);
}

int 
plan_t::find_local_goal(pos2d<double> * g,
                        const pos2d<double> & l) const
{
  // Must already have computed a global goal
  if(path.empty())
  {
    //puts("no global path");
    return(-1);
  }

  int li,lj;
  li = PLAN_GXWX(l.x);
  lj = PLAN_GYWY(l.y);

  assert(PLAN_VALID_BOUNDS(li,lj));

  // Find the closest place to jump on the global path
  double squared_d_min = DBL_MAX;
  int c_min = -1;
  for(int c=0;c<path.size();c++)
  {
	plan_cell_t* cell = path[c];
    double squared_d = ((cell->ci - li) * (cell->ci - li) + 
                 (cell->cj - lj) * (cell->cj - lj));
    if(squared_d < squared_d_min)
    {
      squared_d_min = squared_d;
      c_min = c;
    }
  }
  assert(c_min > -1);

  // Follow the path to find the last cell that's inside the local planning
  // area
  int c;
  for(c=c_min; c<path.size(); c++)
  {
	plan_cell_t* cell = path[c];
    
    //printf("step %d: (%d,%d)\n", c, cell->ci, cell->cj);

    if((cell->ci < min_x) || (cell->ci > max_x) ||
       (cell->cj < min_y) || (cell->cj > max_y))
    {
      // Did we move at least one cell along the path?
      if(c == c_min)
      {
        // nope; the entire global path is outside the local region; can't
        // fix that here
        puts("global path not in local region");
        return(-1);
      }
      else
        break;
    }
  }

  assert(c > c_min);

  plan_cell_t* cell = path[c-1];

  //printf("ci: %d cj: %d\n", cell->ci, cell->cj);
  g->x = PLAN_WXGX(cell->ci);
  g->y = PLAN_WYGY(cell->cj);
  
  return(0);
}

// Push a plan location onto the queue
void plan_t::push(plan_cell_t *cell)
{
  // Substract from max cost because the heap is set up to return the max
  // element.  This could of course be changed.
  assert(PLAN_MAX_COST-cell->plan_cost > 0);
  cell->mark = true;
  heap.push(cell);
}

// Pop a plan location from the queue
plan_cell_t *plan_t::pop()
{
  if(heap.empty()) {
    return(NULL);
  } else {
    plan_cell_t * top = heap.top();
    heap.pop();
    return top;
  }
}
