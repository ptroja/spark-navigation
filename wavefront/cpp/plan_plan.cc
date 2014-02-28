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

#if defined (WIN32)
  #include <replace/replace.h>
  #include <winsock2.h> // For struct timeval
#else
  #include <sys/time.h>
#endif

static double get_time(void);

#include "plan.h"

int
plan_t::do_global(double lx, double ly, double gx, double gy)
{
  plan_cell_t* cell;
  int li, lj;
  double t0,t1;

  t0 = get_time();

  // Set bounds to look over the entire grid
  set_bounds(0, 0, size_x - 1, size_y - 1);

  // Reset plan costs
  reset();

  path_count = 0;
  if(update_plan(lx, ly, gx, gy) < 0)
  {
    // no path
    return(-1);
  }

  li = PLAN_GXWX(this, lx);
  lj = PLAN_GYWY(this, ly);

  // Cache the path
  for(cell = cells + PLAN_INDEX(this,li,lj);
      cell;
      cell = cell->plan_next)
  {
    if(path_count >= path_size)
    {
      path_size *= 2;
      path = (plan_cell_t**)realloc(path,
                                    path_size * sizeof(plan_cell_t*));
      assert(path);
    }
    path[path_count++] = cell;
  }

  t1 = get_time();

  //printf("computed global path: %.6lf\n", t1-t0);

  return(0);
}

int
plan_t::do_local(double lx, double ly, double plan_halfwidth)
{
  double gx, gy;
  int li, lj;
  int xmin,ymin,xmax,ymax;
  plan_cell_t* cell;
  double t0,t1;
  int i;

  t0 = get_time();

  // Set bounds as directed
  xmin = PLAN_GXWX(this, lx - plan_halfwidth);
  ymin = PLAN_GYWY(this, ly - plan_halfwidth);
  xmax = PLAN_GXWX(this, lx + plan_halfwidth);
  ymax = PLAN_GYWY(this, ly + plan_halfwidth);
  set_bounds(xmin, ymin, xmax, ymax);

  // Reset plan costs (within the local patch)
  reset();

  // Find a local goal to pursue
  if(find_local_goal(&gx, &gy, lx, ly) != 0)
  {
    //puts("no local goal");
    return(-1);
  }

  //printf("local goal: %.3lf, %.3lf\n", gx, gy);

  lpath_count = 0;
  if(update_plan(lx, ly, gx, gy) != 0)
  {
    puts("local plan update failed");
    return(-1);
  }

  li = PLAN_GXWX(this, lx);
  lj = PLAN_GYWY(this, ly);

  // Reset path marks (TODO: find a smarter place to do this)
  cell = cells;
  for(i=0;i<size_x*size_y;i++,cell++)
    cell->lpathmark = 0;

  // Cache the path
  for(cell = cells + PLAN_INDEX(this,li,lj);
      cell;
      cell = cell->plan_next)
  {
    if(lpath_count >= lpath_size)
    {
      lpath_size *= 2;
      lpath = (plan_cell_t**)realloc(lpath,
                                     lpath_size * sizeof(plan_cell_t*));
      assert(lpath);
    }
    lpath[lpath_count++] = cell;
    cell->lpathmark = 1;
  }

  t1 = get_time();

  //printf("computed local path: %.6lf\n", t1-t0);
  return(0);
}


// Generate the plan
int 
plan_t::update_plan(double lx, double ly, double gx, double gy)
{
  int oi, oj, di, dj, ni, nj;
  int gi, gj, li,lj;
  float cost;
  plan_cell_t *cell, *ncell;
  char old_occ_state;
  float old_occ_dist;

  // Reset the queue
  heap_reset(heap);

  // Initialize the goal cell
  gi = PLAN_GXWX(this, gx);
  gj = PLAN_GYWY(this, gy);

  // Initialize the start cell
  li = PLAN_GXWX(this, lx);
  lj = PLAN_GYWY(this, ly);

  //printf("planning from %d,%d to %d,%d\n", li,lj,gi,gj);

  if(!PLAN_VALID_BOUNDS(this, gi, gj))
  {
    puts("goal out of bounds");
    return(-1);
  }
  
  if(!PLAN_VALID_BOUNDS(this, li, lj))
  {
    puts("start out of bounds");
    return(-1);
  }

  // Latch and clear the obstacle state for the cell I'm in
  cell = cells + PLAN_INDEX(this, li, lj);
  old_occ_state = cell->occ_state_dyn;
  old_occ_dist = cell->occ_dist_dyn;
  cell->occ_state_dyn = -1;
  cell->occ_dist_dyn = (float) max_radius;

  cell = cells + PLAN_INDEX(this, gi, gj);
  cell->plan_cost = 0;

  // Are we done?
  if((li == gi) && (lj == gj))
    return(0);
  
  push(cell);

  while (1)
  {
    float * p;
    cell = pop();
    if (cell == NULL)
      break;

    oi = cell->ci;
    oj = cell->cj;

    //printf("pop %d %d %f\n", cell->ci, cell->cj, cell->plan_cost);

    p = dist_kernel_3x3;
    for (dj = -1; dj <= +1; dj++)
    {
      ncell = cells + PLAN_INDEX(this,oi-1,oj+dj);
      for (di = -1; di <= +1; di++, p++, ncell++)
      {
        if (!di && !dj)
          continue;
        //if (di && dj)
          //continue;
        
        ni = oi + di;
        nj = oj + dj;

        if (!PLAN_VALID_BOUNDS(this, ni, nj))
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
  cell = cells + PLAN_INDEX(this, li, lj);
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
plan_t::find_local_goal(double* gx, double* gy,
                        double lx, double ly)
{
  int c;
  int c_min;
  double squared_d;
  double squared_d_min;
  int li,lj;
  plan_cell_t* cell;

  // Must already have computed a global goal
  if(path_count == 0)
  {
    //puts("no global path");
    return(-1);
  }

  li = PLAN_GXWX(this, lx);
  lj = PLAN_GYWY(this, ly);

  assert(PLAN_VALID_BOUNDS(this,li,lj));

  // Find the closest place to jump on the global path
  squared_d_min = DBL_MAX;
  c_min = -1;
  for(c=0;c<path_count;c++)
  {
    cell = path[c];
    squared_d = ((cell->ci - li) * (cell->ci - li) + 
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
  for(c=c_min; c<path_count; c++)
  {
    cell = path[c];
    
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

  cell = path[c-1];

  //printf("ci: %d cj: %d\n", cell->ci, cell->cj);
  *gx = PLAN_WXGX(this, cell->ci);
  *gy = PLAN_WYGY(this, cell->cj);
  
  return(0);
}

// Push a plan location onto the queue
void plan_t::push(plan_cell_t *cell)
{
  // Substract from max cost because the heap is set up to return the max
  // element.  This could of course be changed.
  assert(PLAN_MAX_COST-cell->plan_cost > 0);
  cell->mark = 1;
  heap_insert(heap, PLAN_MAX_COST - cell->plan_cost, cell);
}


// Pop a plan location from the queue
plan_cell_t *plan_t::pop()
{

  if(heap_empty(heap))
    return(NULL);
  else
    return((plan_cell_t *) heap_extract_max(heap));
}

double 
static get_time(void)
{
  struct timeval curr;
  gettimeofday(&curr,NULL);
  return(curr.tv_sec + curr.tv_usec / 1e6);
}
