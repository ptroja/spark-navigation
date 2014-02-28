/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2003  Brian Gerkey   gerkey@robotics.stanford.edu
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

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_wavefront wavefront
 * @brief Wavefront-propagation path-planner

The wavefront driver implements a global path planner for a planar
mobile robot.

This driver works in the following way: upon receiving a new @ref
interface_planner target, a path is planned from the robot's
current pose, as reported by the underlying @ref interface_localize
device.  The waypoints in this path are handed down, in sequence,
to the underlying @ref interface_position2d device, which should
be capable of local navigation (the @ref driver_vfh driver is a
great candidate). By tying everything together in this way, this driver
offers the mythical "global goto" for your robot.

The planner first creates a configuration space of grid cells from the
map that is given, treating both occupied and unknown cells as occupied.
The planner assigns a cost to each of the free cells based on their
distance to the nearest obstacle. The nearer the obstacle, the higher
the cost. Beyond the max_radius given by the user, the cost in the
c-space cells is zero.

When the planner is given a new goal, it finds a path by working its
way outwards from the goal cell, assigning plan costs to the cells as
it expands (like a wavefront expanding outwards in water). The plan
cost in each cell is dependant on its distance from the goal, as well
as the obstacle cost assigned in the configuration space step. Once the
plan costs for all the cells have been evaluated, the robot can simply
follow the gradient of each lowest adjacent cell all the way to the goal.

In order to function effectively with an underlying obstacle avoidance
algorithm, such as Vector Field Histogram (the @ref driver_vfh
driver), the planner only hands off waypoints, not the entire path. The
wavefront planner finds the longest straight-line distances that don't
cross obstacles between cells that are on the path. The endpoints of
these straight lines become sequential goal locations for the underlying
device driving the robot.

For help in using this driver, try the @ref util_playernav utility.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_planner

@par Requires

This driver controls two named position2d devices: one for input and one
for output.  That way you can read poses from a localization or SLAM system
and send commands directly to the robot.  The input and output devices may
be the same.

- "input" @ref interface_position2d : source of current pose information
  (usually you would use the @ref driver_amcl driver)
- "output" @ref interface_position2d : robot to be controlled;
  this device must be capable of position control (usually you would
  use the @ref driver_vfh driver)
- @ref interface_map : the map to plan paths in

@par Configuration requests

- PLAYER_PLANNER_REQ_GET_WAYPOINTS

@par Configuration file options

Note that the various thresholds should be set to GREATER than the
underlying position device; otherwise the planner could wait indefinitely
for the position device to achieve a target, when the position device
thinks it has already achieved it.

- safety_dist (length)
  - Default: 0.25 m
  - Don't plan a path any closer than this distance to any obstacle.
    Set this to be GREATER than the corresponding threshold of
    the underlying position device!
- max_radius (length)
  - Default: 1.0 m
  - For planning purposes, all cells that are at least this far from
    any obstacle are equally good (save CPU cycles).
- dist_penalty (float)
  - Default: 1.0
  - Extra cost to discourage cutting corners
- distance_epsilon (length)
  - Default: 0.5 m
  - Planar distance from the target position that will be considered
    acceptable.
    Set this to be GREATER than the corresponding threshold of
    the underlying position device!
- angle_epsilon (angle)
  - Default: 10 deg
  - Angular difference from target angle that will considered acceptable.
    Set this to be GREATER than the corresponding threshold of the
    underlying position device!
- replan_dist_thresh (length)
  - Default: 2.0 m
  - Change in robot's position (in localization space) that will
    trigger replanning.  Set to -1 for no replanning (i.e, make
    a plan one time and then stick with it until the goal is reached).
    Replanning is pretty cheap computationally and can really help in
    dynamic environments.  Note that no changes are made to the map in
    order to replan; support is forthcoming for explicitly replanning
    around obstacles that were not in the map.  See also replan_min_time.
- replan_min_time (float)
  - Default: 2.0
  - Minimum time in seconds between replanning.  Set to -1 for no
    replanning.  See also replan_dist_thresh;
- cspace_file (filename)
  - Default: "player.cspace"
  - Use this file to cache the configuration space (c-space) data.
    At startup, if this file can be read and if the metadata (e.g., size,
    scale) in it matches the current map, then the c-space data is
    read from the file.  Otherwise, the c-space data is computed.
    In either case, the c-space data will be cached to this file for
    use next time.  C-space computation can be expensive and so caching
    can save a lot of time, especially when the planner is frequently
    stopped and started.  This feature requires md5 hashing functions
    in libcrypto.
- add_rotational_waypoints (integer)
  - Default: 1
  - If non-zero, add an in-place rotational waypoint before the next
    waypoint if the difference between the robot's current heading and the
    heading to the next waypoint is greater than 45 degrees.  Generally
    helps the low-level position controller, but sacrifices speed.
- force_map_refresh (integer)
  - Default: 0
  - If non-zero, map is updated from subscribed map device whenever
    new goal is set
- update_rate (integer)
  - Default: 10
  - How many times a second the driver should attempt to run its main loop

@par Example

This example shows how to use the wavefront driver to plan and execute paths
on a laser-equipped Pioneer.

@verbatim
driver
(
  name "p2os"
  provides ["odometry:::position2d:0"]
  port "/dev/ttyS0"
)
driver
(
  name "sicklms200"
  provides ["laser:0"]
  port "/dev/ttyS1"
)
driver
(
  name "mapfile"
  provides ["map:0"]
  filename "mymap.pgm"
  resolution 0.1
)
driver
(
  name "amcl"
  provides ["position2d:2"]
  requires ["odometry:::position2d:1" "laser:0" "laser:::map:0"]
)
driver
(
  name "vfh"
  provides ["position2d:1"]
  requires ["position2d:0" "laser:0"]
  safety_dist 0.1
  distance_epsilon 0.3
  angle_epsilon 5
)
driver
(
  name "wavefront"
  provides ["planner:0" "offline:::planner:1"]
  requires ["output:::position2d:1" "input:::position2d:2" "map:0"]
  safety_dist 0.15
  distance_epsilon 0.5
  angle_epsilon 10
)
@endverbatim

@author Brian Gerkey, Andrew Howard
*/
/** @} */

#include <cstring>
#include <cstddef>
#include <cassert>
#include <cmath>
#include <vector>

#ifndef WIN32
  #include <unistd.h>
  #include <sys/time.h>
#endif

#include <libplayercore/playercore.h>
#include <libplayerinterface/functiontable.h>
#include "plan.h"

static double get_time(void);
//extern "C" { void draw_cspace(plan_t* plan, const char* fname); }

// TODO: monitor localize timestamps, and slow or stop robot accordingly

class Wavefront : public ThreadedDriver
{
  private:
    // Main function for device thread.
    virtual void Main();

    void Sleep(double loopstart);

    // bookkeeping
    player_devaddr_t position_id;
    player_devaddr_t localize_id;
    player_devaddr_t map_id;
    player_devaddr_t laser_id;
    player_devaddr_t graphics2d_id;
    player_devaddr_t offline_planner_id;

    double map_res;
    double robot_radius;
    double safety_dist;
    double max_radius;
    double dist_penalty;
    double dist_eps;
    double ang_eps;
    double cycletime;
    double tvmin, tvmax, avmin, avmax, amin, amax;

    // the plan object
    plan_t* plan;
    // another plan object for offline path computation
    plan_t* offline_plan;

    // pointers to the underlying devices
    Device* position_dev;
    Device* localize_dev;
    Device* map_dev;
    Device* laser_dev;
    Device* graphics2d_dev;

    // are we disabled?
    bool enable;
    // current target (m,m,rad)
    player_pose2d_t target;
    int curr_waypoint;
    // current waypoint (m,m,rad)
    player_pose2d_t waypoint;
    // current waypoint, in odometric coords (m,m,rad)
    player_pose2d_t waypoint_odom;
    // are we pursuing a new goal?
    bool new_goal;
    // current odom pose
    player_pose2d_t position;
    // current waypoints
    std::vector<player_point_2d_t> waypoints;
    // current localize pose
    player_pose2d_t localize;
    // have we told the underlying position device to stop?
    bool stopped;
    // have we reached the goal (used to decide whether or not to replan)?
    bool atgoal;
    // replan each time the robot's localization position changes by at
    // least this much (meters)
    double replan_dist_thresh;
    // leave at least this much time (seconds) between replanning cycles
    double replan_min_time;
    // should we request the map at startup? (or wait for it to be pushed
    // to us as data?)
    bool request_map;
    // Do we have a map yet?
    bool have_map;
    // Has the map changed since last time we planned?
    bool new_map;
    // Is there a new map available (which we haven't retrieved yet)?
    bool new_map_available;
    // Do we consider inserting a rotational waypoint between every pair of
    // waypoints, or just before the first one?
    bool always_insert_rotational_waypoints;
    // Should map be updated on every new goal?
    int force_map_refresh;
    // Should we do velocity control, or position control?
    bool velocity_control;
    // How many laser scans should we buffer?
    int scans_size;
    // How far out do we insert obstacles?
    double scan_maxrange;
    // The scan buffer
    player_laser_data_scanpose_t* scans;
    int scans_count;
    int scans_idx;
    std::vector<double> scan_points;
    // TODO: remove _size and _count and use only std::vector
    size_t scan_points_size;
    size_t scan_points_count;

    // Do we have an offline planner
    bool have_offline_planner;
    // offline planner goal postion
    player_pose2d_t offline_goal;
    // offline planner start position
    player_pose2d_t offline_start;

    // methods for internal use
    int SetupLocalize();
    int SetupLaser();
    int SetupPosition();
    int SetupMap();
    int SetupGraphics2d();
    int GetMap(bool threaded);
    int GetMapInfo(bool threaded);
    int ShutdownPosition();
    int ShutdownLocalize();
    int ShutdownLaser();
    int ShutdownMap();
    int ShutdownGraphics2d();
    static double angle_diff(double a, double b);

    void ProcessCommand(const player_planner_cmd_t & cmd);
    void ProcessLaserScan(player_laser_data_scanpose_t* data);
    void ProcessLocalizeData(const player_position2d_data_t & data);
    void ProcessPositionData(const player_position2d_data_t & data);
    void ProcessMapInfo(const player_map_info_t & info);

    enum POSITION_CMD_TYPE { VELOCITY_CMD, POSITION_CMD };

    void PutPositionCommand(double x, double y, double a, POSITION_CMD_TYPE type);
    void PutPlannerData();
    void StopPosition();
    void LocalizeToPosition(player_pose2d_t * p,
                            const player_pose2d_t & l);
    void SetWaypoint(const player_pose2d_t & w);
    void ComputeOfflineWaypoints(player_planner_waypoints_req_t* req, player_planner_waypoints_req_t* reply);

  public:
    // Constructor
    Wavefront( ConfigFile* cf, int section);

    // Setup/shutdown routines.
    virtual int MainSetup();
    virtual void MainQuit();

    // Process incoming messages from clients
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);
};


// Initialization function
Driver* Wavefront_Init( ConfigFile* cf, int section)
{
  return ((Driver*) (new Wavefront( cf, section)));
}


// a driver registration function
void wavefront_Register(DriverTable* table)
{
  table->AddDriver("wavefront",  Wavefront_Init);
}


// Extra stuff for building a shared object.
/* need the extern to avoid C++ name-mangling  */
extern "C"
{

int player_driver_init(DriverTable* table)
{
	wavefront_Register(table);
	return(0);
}

}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Wavefront::Wavefront( ConfigFile* cf, int section)
  : ThreadedDriver(cf, section, true),
    scans(NULL)
{
  this->have_offline_planner = false;
  // Must have a position device to control
  if (cf->ReadDeviceAddr(&this->position_id, section, "requires",
                         PLAYER_POSITION2D_CODE, -1, "output") != 0)
  {
    this->SetError(-1);
    return;
  }
  // Must have a position device from which to read global poses
  if (cf->ReadDeviceAddr(&this->localize_id, section, "requires",
                         PLAYER_POSITION2D_CODE, -1, "input") != 0)
  {
    this->SetError(-1);
    return;
  }
  // Must have a map device
  if (cf->ReadDeviceAddr(&this->map_id, section, "requires",
                         PLAYER_MAP_CODE, -1, NULL) != 0)
  {
    this->SetError(-1);
    return;
  }

  if (cf->ReadDeviceAddr(&this->device_addr, section, "provides",
                         PLAYER_PLANNER_CODE, -1, "") != 0)
  {
    PLAYER_ERROR("Wavefront must provide a Planner");
    this->SetError(-1);
    return;
  } else {
    if (this->AddInterface(this->device_addr) != 0)
    {
      this->SetError(-1);
      return;
    }
  }
  if (cf->ReadDeviceAddr(&this->offline_planner_id, section, "provides",
                         PLAYER_PLANNER_CODE, -1, "offline") != 0)
  {
    this->have_offline_planner = true;
    PLAYER_WARN("Wavefront providing offline planner");
  } else {
    if (this->AddInterface(this->offline_planner_id) != 0)
    {
      this->SetError(-1);
      return;
    }
  }

  // Can use a laser device
  memset(&this->laser_id,0,sizeof(player_devaddr_t));
  cf->ReadDeviceAddr(&this->laser_id, section, "requires",
                     PLAYER_LASER_CODE, -1, NULL);

  // Can use a graphics2d device
  memset(&this->graphics2d_id,0,sizeof(player_devaddr_t));
  cf->ReadDeviceAddr(&this->graphics2d_id, section, "requires",
                     PLAYER_GRAPHICS2D_CODE, -1, NULL);

  this->safety_dist = cf->ReadLength(section,"safety_dist", 0.25);
  this->max_radius = cf->ReadLength(section,"max_radius",1.0);
  this->dist_penalty = cf->ReadFloat(section,"dist_penalty",1.0);
  this->dist_eps = cf->ReadLength(section,"distance_epsilon", 0.5);
  this->ang_eps = cf->ReadAngle(section,"angle_epsilon",DTOR(10));
  this->replan_dist_thresh = cf->ReadLength(section,"replan_dist_thresh",2.0);
  this->replan_min_time = cf->ReadFloat(section,"replan_min_time",2.0);
  this->request_map = cf->ReadInt(section,"request_map",1);
  this->always_insert_rotational_waypoints =
          cf->ReadInt(section, "add_rotational_waypoints", 1);
  this->force_map_refresh = cf->ReadInt(section, "force_map_refresh", 0);
  this->cycletime = 1.0 / cf->ReadFloat(section, "update_rate", 10.0);

  this->velocity_control = cf->ReadInt(section, "velocity_control", 0);
  if(this->velocity_control)
  {
    this->tvmin = cf->ReadTupleLength(section, "control_tv", 0, 0.1);
    this->tvmax = cf->ReadTupleLength(section, "control_tv", 1, 0.5);
    this->avmin = cf->ReadTupleAngle(section, "control_av", 0, DTOR(10.0));
    this->avmax = cf->ReadTupleAngle(section, "control_av", 1, DTOR(90.0));
    this->amin = cf->ReadTupleAngle(section, "control_a", 0, DTOR(5.0));
    this->amax = cf->ReadTupleAngle(section, "control_a", 1, DTOR(20.0));
  }

  if(this->laser_id.interf)
  {
    this->scans_size = cf->ReadInt(section, "laser_buffer_size", 10);
    if(this->scans_size < 1)
    {
      PLAYER_WARN("must buffer at least one laser scan");
      this->scans_size = 1;
    }
    this->scan_maxrange = cf->ReadLength(section, "laser_maxrange", 6.0);
  }
  else
  {
    this->scans_size = 0;
    if(this->velocity_control)
      PLAYER_WARN("Wavefront doing direct velocity control, but without a laser for obstacle detection; this is not safe!");
  }
  memset((void*)(&this->offline_goal), 0, sizeof(player_pose2d_t));
  memset((void*)(&this->offline_start), 0, sizeof(player_pose2d_t));
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).
int
Wavefront::MainSetup()
{
  const player_pose2d_t zero = { 0.0, 0.0, 0.0 };

  this->have_map = false;
  this->new_map = false;
  this->new_map_available = false;
  this->stopped = true;
  this->atgoal = true;
  this->enable = true;
  this->target = zero;
  this->position = zero;
  this->localize = zero;
  this->waypoint = zero;
  this->waypoint_odom = zero;
  this->curr_waypoint = -1;

  this->new_goal = false;

  if(SetupPosition() < 0)
    return(-1);

  if(!(this->plan = new plan_t(this->robot_radius+this->safety_dist,
                               this->robot_radius+this->safety_dist,
                               this->max_radius,
                               this->dist_penalty,
                               0.5)))
  {
    PLAYER_ERROR("failed to allocate plan");
    return(-1);
  }
  this->offline_plan = NULL;
  if(SetupMap() < 0)
    return(-1);
  if(SetupLocalize() < 0)
    return(-1);

  if(this->laser_id.interf)
  {
    if(SetupLaser() < 0)
      return(-1);

    this->scans = new player_laser_data_scanpose_t[this->scans_size];

    this->scans_idx = 0;
    this->scans_count = 0;
    this->scan_points.clear();
    this->scan_points_size = 0;
    this->scan_points_count = 0;
  }

  if(this->graphics2d_id.interf)
  {
    if(SetupGraphics2d() < 0)
      return(-1);
  }
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).
void
Wavefront::MainQuit()
{
  if(this->plan) {
    delete(this->plan);
    this->plan = NULL;
  }

  if(this->offline_plan) {
    delete(this->offline_plan);
    this->offline_plan = NULL;
  }

  this->waypoints.clear();

  ShutdownPosition();
  ShutdownLocalize();
  ShutdownMap();
  if(this->laser_id.interf)
    ShutdownLaser();
  if(this->graphics2d_id.interf)
    ShutdownGraphics2d();

  if(this->scans) delete [] this->scans;
}

void
Wavefront::ProcessCommand(const player_planner_cmd_t & cmd)
{
  const player_pose2d_t & new_goal = cmd.goal;
  //double eps = 1e-3;

#if 0
  if((std::abs(new_x - this->target.px) > eps) ||
     (std::abs(new_y - this->target.py) > eps) ||
     (std::abs(this->angle_diff(new_a,this->target.pa)) > eps))
  {
#endif
    this->target = new_goal;
    printf("new goal: %f, %f, %f\n", target.px, target.py, target.pa);
    this->new_goal = true;
    this->atgoal = false;
#if 0
  }
#endif
}

void
Wavefront::ComputeOfflineWaypoints(player_planner_waypoints_req_t* req, player_planner_waypoints_req_t* reply)
{
  const player_pose2d_t & s = this->offline_start;
  const player_pose2d_t & g = this->offline_goal;

  // If there is no offline_plan, create by duplicating plan
  if(!this->offline_plan)
    this->offline_plan = new plan_t(*this->plan);

  // Compute path in offline plan
  if(this->offline_plan->do_global(s.px, s.py, g.px, g.py) < 0)
  {
    puts("Wavefront: offline path computation failed");
  }

  // Extract waypoints along the path to the goal from the start position
  this->offline_plan->update_waypoints(s.px, s.py);

  // Fill in reply
  // - waypoints
  if((reply->waypoints_count = this->offline_plan->waypoint_count))
  {
    reply->waypoints = new player_pose2d_t[reply->waypoints_count];

    double distance = 0.0;
    player_point_2d_t last_w;
    for(int i=0;i<(int)reply->waypoints_count;i++)
    {
      // Convert and copy waypoint
      player_point_2d_t w;
      this->offline_plan->convert_waypoint(this->offline_plan->waypoints[i],
                                           &w.px, &w.py);
      reply->waypoints[i].px = w.px;
      reply->waypoints[i].py = w.py;
      reply->waypoints[i].pa = 0.0;
      // Update path length
      if(i != 0)
      {
        distance += hypot(w.px-last_w.px,w.py-last_w.py);
      }
      last_w = w;
    }
    reply->waypoints_distance = distance;
  }
  else  // no waypoints
  {
    reply->waypoints = NULL;
    reply->waypoints_distance = 0.0;
  }
}


void
Wavefront::ProcessLaserScan(player_laser_data_scanpose_t* data)
{
  double t0 = get_time();

  // free up the old scan, if we're replacing one
  if(this->scans_idx < this->scans_count)
    playerxdr_cleanup_message(this->scans+this->scans_idx,
                              PLAYER_LASER_CODE, PLAYER_MSGTYPE_DATA,
                              PLAYER_LASER_DATA_SCANPOSE);
  // copy in the new scan
  playerxdr_deepcopy_message(data, this->scans+this->scans_idx,
                             PLAYER_LASER_CODE, PLAYER_MSGTYPE_DATA,
                             PLAYER_LASER_DATA_SCANPOSE);
  //memcpy(this->scans+this->scans_idx, data,
         //sizeof(player_laser_data_scanpose_t));

  // update counters
  this->scans_count++;
  this->scans_count = MIN(this->scans_count,this->scans_size);
  this->scans_idx = (this->scans_idx + 1) % this->scans_size;

  //printf("%d scans\n", this->scans_count);

  // run through the scans to see how much room we need to store all the
  // hitpoints
  size_t hitpt_cnt=0;
  player_laser_data_scanpose_t* scan = this->scans;
  for(int i=0;i<this->scans_count;i++,scan++)
    hitpt_cnt += scan->scan.ranges_count*2;

  // allocate more space as necessary
  if(this->scan_points_size < hitpt_cnt)
  {
    this->scan_points_size = hitpt_cnt;
    this->scan_points.clear();
    this->scan_points.reserve(this->scan_points_size);
  }

  // project hit points from each scan
  scan = this->scans;
  double* pts = this->scan_points.data();
  this->scan_points_count = 0;
  for(int i=0;i<this->scans_count;i++,scan++)
  {
    float b=scan->scan.min_angle;
    float* r=scan->scan.ranges;
    for(unsigned int j=0;
        j<scan->scan.ranges_count;
        j++,r++,b+=scan->scan.resolution)
    {
      if(((*r) >= this->scan_maxrange) || ((*r) >= scan->scan.max_range))
        continue;
      //double rx, ry;
      //rx = (*r)*cos(b);
      //ry = (*r)*sin(b);

      double cs,sn;
      cs = cos(scan->pose.pa+b);
      sn = sin(scan->pose.pa+b);

      double lx,ly;
      lx = scan->pose.px + (*r)*cs;
      ly = scan->pose.py + (*r)*sn;

      assert(this->scan_points_count*2 < this->scan_points_size);
      *(pts++) = lx;
      *(pts++) = ly;
      this->scan_points_count++;
    }
  }

  //printf("setting %d hit points\n", this->scan_points_count);
  plan->set_obstacles(this->scan_points.data(), this->scan_points_count);

  double t1 = get_time();
  //printf("ProcessLaserScan: %.6lf\n", t1-t0);

  if(this->graphics2d_id.interf)
  {
    // Draw the points
    player_graphics2d_cmd_points pts;
    pts.points = new player_point_2d_t[hitpt_cnt/2];

    pts.points_count = hitpt_cnt/2;
    pts.color.alpha = 0;
    pts.color.red = 255;
    pts.color.blue = 0;
    pts.color.green = 0;
    for(int i=0;i<hitpt_cnt/2;i++)
    {
      pts.points[i].px = this->scan_points[2*i];
      pts.points[i].py = this->scan_points[2*i+1];
    }

    this->graphics2d_dev->PutMsg(this->InQueue,
                             PLAYER_MSGTYPE_CMD,
                             PLAYER_GRAPHICS2D_CMD_POINTS,
                             (void*)&pts,0,NULL);
    delete [] pts.points;
  }
}

void
Wavefront::ProcessLocalizeData(const player_position2d_data_t & data)
{
  this->localize = data.pos;
}

void
Wavefront::ProcessPositionData(const player_position2d_data_t & data)
{
  this->position = data.pos;
}

void
Wavefront::ProcessMapInfo(const player_map_info_t & info)
{
  // Got new map info pushed to us.  We'll save this info and get the new
  // map.
  this->plan->scale = info.scale;
  this->plan->size_x = info.width;
  this->plan->size_y = info.height;
  this->plan->origin_x = info.origin.px;
  this->plan->origin_y = info.origin.py;

  // Now get the map data, possibly in separate tiles.
  if(this->GetMap(true) < 0)
  {
    this->have_map = false;
    this->StopPosition();
  }
  else
  {
    this->have_map = true;
    this->new_map = true;
    // force replanning
    if(this->curr_waypoint >= 0)
      this->new_goal = true;
  }
}

void
Wavefront::PutPlannerData()
{
  player_planner_data_t data;

  memset(&data,0,sizeof(data));

  if(this->waypoints.empty())
    data.valid = 0;
  else
    data.valid = 1;

  if((!this->waypoints.empty()) && (this->curr_waypoint < 0))
    data.done = 1;
  else
    data.done = 0;

  // put the current localize pose
  data.pos = this->localize;

  data.goal = this->target;

  if(data.valid && !data.done)
  {
    data.waypoint = this->waypoint;

    data.waypoint_idx = this->curr_waypoint;
    data.waypoints_count = this->waypoints.size();
  }

  this->Publish(this->device_addr,
                PLAYER_MSGTYPE_DATA,
                PLAYER_PLANNER_DATA_STATE,
                (void*)&data,sizeof(data),NULL);
}

void
Wavefront::PutPositionCommand(double x, double y, double a, POSITION_CMD_TYPE type)
{
  if(type == POSITION_CMD)
  {
	player_position2d_cmd_pos_t pos_cmd;
	memset(&pos_cmd,0,sizeof(pos_cmd));

    // position control
    pos_cmd.pos.px = x;
    pos_cmd.pos.py = y;
    pos_cmd.pos.pa = a;
    pos_cmd.state=1;
    this->position_dev->PutMsg(this->InQueue,
                         PLAYER_MSGTYPE_CMD,
                         PLAYER_POSITION2D_CMD_POS,
                         (void*)&pos_cmd,sizeof(pos_cmd),NULL);
  }
  else
  {
	player_position2d_cmd_vel_t vel_cmd;
	memset(&vel_cmd,0,sizeof(vel_cmd));

    // velocity control (used to stop the robot)
    vel_cmd.vel.px = x;
    vel_cmd.vel.py = y;
    vel_cmd.vel.pa = a;
    vel_cmd.state=1;
    this->position_dev->PutMsg(this->InQueue,
                         PLAYER_MSGTYPE_CMD,
                         PLAYER_POSITION2D_CMD_VEL,
                         (void*)&vel_cmd,sizeof(vel_cmd),NULL);
  }

  this->stopped = false;
}

void
Wavefront::LocalizeToPosition(player_pose2d_t * p,
		                      const player_pose2d_t & l)
{
  double offset_x, offset_y, offset_a;
  double lx_rot, ly_rot;

  offset_a = this->angle_diff(this->position.pa,this->localize.pa);
  lx_rot = this->localize.px * cos(offset_a) - this->localize.py * sin(offset_a);
  ly_rot = this->localize.px * sin(offset_a) + this->localize.py * cos(offset_a);

  offset_x = this->position.px - lx_rot;
  offset_y = this->position.py - ly_rot;

  //printf("offset: %f, %f, %f\n", offset_x, offset_y, RTOD(offset_a));

  p->px = l.px * cos(offset_a) - l.py * sin(offset_a) + offset_x;
  p->py = l.px * sin(offset_a) + l.py * cos(offset_a) + offset_y;
  p->pa = l.pa + offset_a;
}

void
Wavefront::StopPosition()
{
  if(!this->stopped)
  {
    //puts("stopping robot");
    PutPositionCommand(0.0,0.0,0.0,VELOCITY_CMD);
    this->stopped = true;
  }
}

void
Wavefront::SetWaypoint(const player_pose2d_t & w)
{
  player_pose2d_t w_odom;

  // transform to odometric frame
  LocalizeToPosition(&w_odom, w);

  // hand down waypoint
  printf("sending waypoint: %.3f %.3f %.3f\n",
         w_odom.px, w_odom.py, RTOD(w_odom.pa));
  PutPositionCommand(w_odom.px, w_odom.py, w_odom.pa, POSITION_CMD);

  // cache this waypoint, odometric coords
  this->waypoint_odom = w_odom;
}

void
Wavefront::Sleep(double loopstart)
{
  //GlobalTime->GetTimeDouble(&currt);
  double currt = get_time();

  //printf("cycle: %.6lf\n", currt-loopstart);

  double tdiff = MAX(0.0, this->cycletime - (currt-loopstart));

  if(tdiff == 0.0)
    PLAYER_WARN("Wavefront missed deadline and not sleeping; check machine load");

  usleep((unsigned int)rint(tdiff*1e6));
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void Wavefront::Main()
{
  player_point_2d_t last_replan = { 0.0, 0.0 };
  double last_replan_time = 0.0;
  double last_publish_time = 0.0;
  bool rotate_waypoint=false;
  int rotate_dir=0;
  bool printed_warning=false;

  pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);

  // block until we get initial data from underlying devices
  // TODO
  //this->position->Wait();
  this->StopPosition();

  for(;;)
  {
    //GlobalTime->GetTimeDouble(&t);
    double t = get_time();

    pthread_testcancel();

    ProcessMessages();

    if(!this->have_map && !this->new_map_available)
    {
      this->Sleep(t);
      continue;
    }

    if((t - last_publish_time) > 0.25)
    {
      this->PutPlannerData();
      last_publish_time = t;
    }

    if(!this->enable)
    {
      this->StopPosition();
      this->Sleep(t);
      continue;
    }

    // Is it time to replan?
    double replan_timediff = t - last_replan_time;
    double replan_dist = hypot(this->localize.px - last_replan.px,
                               this->localize.py - last_replan.py);
    bool replan = (this->replan_dist_thresh >= 0.0) &&
            (replan_dist > this->replan_dist_thresh) &&
            (this->replan_min_time >= 0.0) &&
            (replan_timediff > this->replan_min_time) &&
            !this->atgoal;

    // Did we get a new goal, or is it time to replan?
    if(this->new_goal || replan || (this->velocity_control && !this->atgoal))
    {
#if 0
      // Should we get a new map?
      if(this->new_map_available)
      {
        this->new_map_available = false;

        if(this->GetMapInfo(true) < 0)
          PLAYER_WARN("failed to get new map info");
        else
        {
          if(this->GetMap(true) < 0)
            PLAYER_WARN("failed to get new map data");
          else
          {
            this->new_map = true;
            this->have_map = true;
          }
        }
      }

      // We need to recompute the C-space if the map changed, or if the
      // goal or robot pose lie outside the bounds of the area we last
      // searched.
      if(this->new_map ||
         !plan_check_inbounds(plan,this->localize.px,this->localize.py) ||
          !plan_check_inbounds(plan,this->target.px,this->target.py))
      {
        // Unfortunately, this computation can take a while (e.g., 1-2
        // seconds).  So we'll stop the robot while it thinks.
        this->StopPosition();

        // Set the bounds to search only an axis-aligned bounding box
        // around the robot and the goal.
        plan_set_bbox(this->plan, 1.0, 3.0,
                      this->localize.px, this->localize.py,
                      this->target.px, this->target.py);

        double t0 = get_time();
        plan_update_cspace(this->plan,this->cspace_fname);
        double t1 = get_time();
        printf("time to update: %f\n", t1 - t0);
        this->new_map = false;
      }
#endif
      if(this->graphics2d_id.interf)
      {
        this->graphics2d_dev->PutMsg(this->InQueue,
                                 PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRAPHICS2D_CMD_CLEAR,
                                 NULL,0,NULL);
      }

      double t0 = get_time();

      // compute costs to the new goal.  Try local plan first
      if(new_goal ||
         (this->plan->path.empty()) ||
         (this->plan->do_local(this->localize.px,
                               this->localize.py, this->scan_maxrange) < 0))
      {
        if(!new_goal && (!this->plan->path.empty()))
          puts("Wavefront: local plan failed");

        // Create a global plan
        if(this->plan->do_global(this->localize.px, this->localize.py,
                                 this->target.px, this->target.py) < 0)
        {
          if(!printed_warning)
          {
            puts("Wavefront: global plan failed");
            printed_warning = true;
          }
        }
        else
        {
          this->new_goal = false;
          printed_warning = false;
        }
      }

      if(this->graphics2d_id.interf && !this->plan->lpath.empty())
      {
        player_graphics2d_cmd_polyline_t line;
        line.points_count = this->plan->lpath.size();
        line.points = new player_point_2d_t[line.points_count];
        line.color.alpha = 0;
        line.color.red = 0;
        line.color.green = 255;
        line.color.blue = 0;
        for(int i=0;i<this->plan->lpath.size();i++)
        {
          line.points[i].px = PLAN_WXGX(this->plan,this->plan->lpath[i]->ci);
          line.points[i].py = PLAN_WYGY(this->plan,this->plan->lpath[i]->cj);
        }
        this->graphics2d_dev->PutMsg(this->InQueue,
                                 PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRAPHICS2D_CMD_POLYLINE,
                                 (void*)&line,0,NULL);
        delete [] line.points;
      }

      if(this->graphics2d_id.interf && !this->plan->path.empty())
      {
        player_graphics2d_cmd_polyline_t line;
        line.points_count = this->plan->path.size();
        line.points = new player_point_2d_t[line.points_count];
        line.color.alpha = 0;
        line.color.red = 255;
        line.color.green = 0;
        line.color.blue = 0;
        for(int i=0;i<this->plan->path.size();i++)
        {
          line.points[i].px = PLAN_WXGX(this->plan,this->plan->path[i]->ci);
          line.points[i].py = PLAN_WYGY(this->plan,this->plan->path[i]->cj);
        }
        this->graphics2d_dev->PutMsg(this->InQueue,
                                 PLAYER_MSGTYPE_CMD,
                                 PLAYER_GRAPHICS2D_CMD_POLYLINE,
                                 (void*)&line,0,NULL);
        delete [] line.points;
      }

      double t1 = get_time();
      //printf("planning: %.6lf\n", t1-t0);

      if(!this->velocity_control)
      {
        // extract waypoints along the path to the goal from the current position
        this->plan->update_waypoints(this->localize.px, this->localize.py);

        if(this->plan->waypoint_count == 0)
        {
          fprintf(stderr, "Wavefront (port %d):\n  "
                  "No path from (%.3lf,%.3lf,%.3lf) to (%.3lf,%.3lf,%.3lf)\n",
                  this->device_addr.robot,
                  this->localize.px,
                  this->localize.py,
                  RTOD(this->localize.pa),
                  this->target.px,
                  this->target.py,
                  RTOD(this->target.pa));
          // Only fail here if this is our first try at making a plan;
          // if we're replanning and don't find a path then we'll just stick
          // with the old plan.
          if(this->curr_waypoint < 0)
          {
            //this->curr_waypoint = -1;
            this->new_goal=false;
            this->waypoints.clear();
          }
        }
        else
        {
          this->waypoints.clear();

          for(int i=0;i<this->plan->waypoint_count;i++)
          {
            player_point_2d_t w;
            this->plan->convert_waypoint(this->plan->waypoints[i],
                                         &w.px, &w.py);
            this->waypoints.push_back(w);
          }

          this->curr_waypoint = 0;
          // Why is this here?
          this->new_goal = true;
        }

        last_replan_time = t;
        last_replan.px = this->localize.px;
        last_replan.py = this->localize.py;
      }
    }


    if(this->velocity_control)
    {
      double t0 = get_time();
      if(!this->plan->path.empty() && !this->atgoal)
      {
        // Check doneness
        double dist = hypot(this->localize.px - this->target.px,
                            this->localize.py - this->target.py);
        double angle = std::abs(this->angle_diff(this->target.pa,this->localize.pa));
        if((dist < this->dist_eps) && (angle < this->ang_eps))
        {
          this->StopPosition();
          this->new_goal = false;
          this->curr_waypoint = -1;
          this->atgoal = true;
        }
        else
        {
          // Compute velocities
          double wx, wy;
          double maxd=2.0;
          double distweight=10.0;

          //printf("pose: (%.3lf,%.3lf,%.3lf)\n",
                 //this->localize.px, this->localize.py, RTOD(this->localize.pa));
          if(this->plan->get_carrot(&wx, &wy,
                                    this->localize.px, this->localize.py,
                                    maxd, distweight) < 0)
          {
            puts("Failed to find a carrot");
            //draw_cspace(this->plan, "debug.png");
            this->StopPosition();
            //exit(-1);
          }
          else
          {
            if(this->graphics2d_id.interf)
            {
              player_graphics2d_cmd_polyline_t line;
              line.points_count = 2;
              line.points = new player_point_2d_t[line.points_count];
              line.color.alpha = 0;
              line.color.red = 0;
              line.color.green = 0;
              line.color.blue = 255;

              line.points[0].px = this->localize.px;
              line.points[0].py = this->localize.py;
              line.points[1].px = wx;
              line.points[1].py = wy;
              this->graphics2d_dev->PutMsg(this->InQueue,
                                       PLAYER_MSGTYPE_CMD,
                                       PLAYER_GRAPHICS2D_CMD_POLYLINE,
                                       (void*)&line,0,NULL);
              delete [] line.points;
            }

            // Establish fake waypoints, for client-side visualization
            this->curr_waypoint = 0;
            this->waypoints.resize(2);
            this->waypoints[0].px = this->localize.px;
            this->waypoints[0].py = this->localize.py;
            this->waypoint.px = this->waypoints[1].px = wx;
            this->waypoint.py = this->waypoints[1].py = wy;
            this->waypoint.pa = 0.0;

            double goald = hypot(this->localize.px-this->target.px,
                                 this->localize.py-this->target.py);

            double d = hypot(this->localize.px-wx,this->localize.py-wy);
            double b = atan2(wy - this->localize.py, wx - this->localize.px);

            double av,tv;
            double a = this->amin + (d / maxd) * (this->amax-this->amin);
            double ad = angle_diff(b, this->localize.pa);

            // Are we on top of the goal?
            if(goald < this->dist_eps)
            {
              if(!rotate_dir)
              {
                if(ad < 0)
                  rotate_dir = -1;
                else
                  rotate_dir = 1;
              }

              tv = 0.0;
              av = rotate_dir * (this->avmin + (std::abs(ad)/M_PI) *
                                 (this->avmax-this->avmin));
            }
            else
            {
              rotate_dir = 0;

              if(std::abs(ad) > a)
                tv = 0.0;
              else
              {
                //tv = tvmin + (d / (M_SQRT2 * maxd)) * (tvmax-tvmin);
                tv = this->tvmin + (d / maxd) * (this->tvmax-this->tvmin);
              }

              av = this->avmin + (std::abs(ad)/M_PI) * (this->avmax-this->avmin);
              if(ad < 0)
                av = -av;
            }

            this->PutPositionCommand(tv,0.0,av,VELOCITY_CMD);
          }
        }
      }
      else
        this->StopPosition();

      double t1 = get_time();
      //printf("control: %.6lf\n", t1-t0);
    }
    else // !velocity_control
    {
      bool going_for_target = (this->curr_waypoint == this->plan->waypoint_count);
      double dist = hypot(this->localize.px - this->target.px,this->localize.py - this->target.py);
      // Note that we compare the current heading and waypoint heading in the
      // *odometric* frame.   We do this because comparing the current
      // heading and waypoint heading in the localization frame is unreliable
      // when making small adjustments to achieve a desired heading (i.e., the
      // robot gets there and VFH stops, but here we don't realize we're done
      // because the localization heading hasn't changed sufficiently).
      double angle = std::abs(this->angle_diff(this->waypoint_odom.pa,this->position.pa));
      if(going_for_target && dist < this->dist_eps && angle < this->ang_eps)
      {
        // we're at the final target, so stop
        StopPosition();
        this->curr_waypoint = -1;
        this->new_goal = false;
        this->atgoal = true;
      }
      else if(this->curr_waypoint < 0)
      {
        // no more waypoints, so stop
        StopPosition();
      }
      else
      {
        // are we there yet?  ignore angle, cause this is just a waypoint
        dist = hypot(this->localize.px - this->waypoint.px,
        		     this->localize.py - this->waypoint.py);
        // Note that we compare the current heading and waypoint heading in the
        // *odometric* frame.   We do this because comparing the current
        // heading and waypoint heading in the localization frame is unreliable
        // when making small adjustments to achieve a desired heading (i.e., the
        // robot gets there and VFH stops, but here we don't realize we're done
        // because the localization heading hasn't changed sufficiently).
        if(this->new_goal ||
           (rotate_waypoint &&
            (std::abs(this->angle_diff(this->waypoint_odom.pa,this->position.pa))
             < M_PI/4.0)) ||
           (!rotate_waypoint && (dist < this->dist_eps)))
        {
          if(this->curr_waypoint == this->waypoints.size())
          {
            // no more waypoints, so wait for target achievement

            //puts("waiting for goal achievement");
            this->Sleep(t);
            continue;
          }
          // get next waypoint
          this->waypoint.px = this->waypoints[this->curr_waypoint].px;
          this->waypoint.py = this->waypoints[this->curr_waypoint].py;
          this->curr_waypoint++;

          this->waypoint.pa = this->target.pa;
          if(this->always_insert_rotational_waypoints ||
             (this->curr_waypoint == 2))
          {
            dist = hypot(this->waypoint.px - this->localize.px,
            		     this->waypoint.py - this->localize.py);
            angle = atan2(this->waypoint.py - this->localize.py,
                          this->waypoint.px - this->localize.px);
            if((dist > this->dist_eps) &&
               std::abs(this->angle_diff(angle,this->localize.pa)) > M_PI/4.0)
            {
              this->waypoint.px = this->localize.px;
              this->waypoint.py = this->localize.py;
              this->waypoint.pa = angle;
              this->curr_waypoint--;
              rotate_waypoint=true;
            }
            else
              rotate_waypoint=false;
          }
          else
            rotate_waypoint=false;

          this->new_goal = false;
        }

        SetWaypoint(this->waypoint);
      }
    }

    this->Sleep(t);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Set up the underlying position device.
int
Wavefront::SetupPosition()
{
  // Subscribe to the position device.
  if(!(this->position_dev = deviceTable->GetDevice(this->position_id)))
  {
    PLAYER_ERROR("unable to locate suitable position device");
    return(-1);
  }
  if(this->position_dev->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to position device");
    return(-1);
  }

  // Enable the motors
  player_position2d_power_config_t motorconfig;
  motorconfig.state = 1;

  Message* msg;

  if(!(msg = this->position_dev->Request(this->InQueue,
                                     PLAYER_MSGTYPE_REQ,
                                     PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                     (void*)&motorconfig,
                                     sizeof(motorconfig), NULL, false)))
  {
    PLAYER_WARN("failed to enable motors");
  }
  else
    delete msg;

  // Get the robot's geometry
  if(!(msg = this->position_dev->Request(this->InQueue,
                                     PLAYER_MSGTYPE_REQ,
                                     PLAYER_POSITION2D_REQ_GET_GEOM,
                                     NULL, 0, NULL, false)) ||
     (msg->GetHeader()->size != sizeof(player_position2d_geom_t)))
  {
    PLAYER_ERROR("failed to get geometry of underlying position device");
    if(msg)
      delete msg;
    return(-1);
  }

  player_position2d_geom_t * geom = (player_position2d_geom_t*)msg->GetPayload();

  // take the bigger of the two dimensions, convert to meters, and halve
  // to get a radius
  //this->robot_radius = MAX(geom->size.sl, geom->size.sw);
  this->robot_radius = geom->size.sw;
  this->robot_radius /= 2.0;

  printf("robot radius: %.3lf\n", this->robot_radius);

  delete msg;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the underlying laser device.
int
Wavefront::SetupLaser()
{
  // Subscribe to the laser device.
  if(!(this->laser_dev = deviceTable->GetDevice(this->laser_id)))
  {
    PLAYER_ERROR("unable to locate suitable laser device");
    return(-1);
  }
  if(this->laser_dev->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to laser device");
    return(-1);
  }

  return(0);
}

int
Wavefront::SetupGraphics2d()
{
  // Subscribe to the graphics2d device.
  if(!(this->graphics2d_dev = deviceTable->GetDevice(this->graphics2d_id)))
  {
    PLAYER_ERROR("unable to locate suitable graphics2d device");
    return(-1);
  }
  if(this->graphics2d_dev->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to graphics2d device");
    return(-1);
  }

  return(0);
}

////////////////////////////////////////////////////////////////////////////////
// Set up the underlying localize device.
int
Wavefront::SetupLocalize()
{
  // Subscribe to the localize device.
  if(!(this->localize_dev = deviceTable->GetDevice(this->localize_id)))
  {
    PLAYER_ERROR("unable to locate suitable localize device");
    return(-1);
  }
  if(this->localize_dev->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to localize device");
    return(-1);
  }

  return(0);
}

// Retrieve the map data in tiles, assuming that the map info is already
// stored in this->plan.
int
Wavefront::GetMap(bool threaded)
{
  // allocate space for map cells
  this->plan->cells = (plan_cell_t*)realloc(this->plan->cells,
                                            (this->plan->size_x *
                                             this->plan->size_y *
                                             sizeof(plan_cell_t)));
  assert(this->plan->cells);

  // Reset the grid
  this->plan->reset();

  // now, get the map data
  player_map_data_t data_req;
  memset(&data_req,0,sizeof(player_map_data_t));
  int oi,oj;
  int sx,sy;
  int si,sj;

  // Grab 640x640 tiles
  sy = sx = 640;
  oi=oj=0;
  while((oi < this->plan->size_x) && (oj < this->plan->size_y))
  {
    si = MIN(sx, this->plan->size_x - oi);
    sj = MIN(sy, this->plan->size_y - oj);

    data_req.col = oi;
    data_req.row = oj;
    data_req.width = si;
    data_req.height = sj;

    Message* msg;
    if(!(msg = this->map_dev->Request(this->InQueue,
                                        PLAYER_MSGTYPE_REQ,
                                        PLAYER_MAP_REQ_GET_DATA,
                                        (void*)&data_req,0,NULL,
                                        threaded)))
    {
      PLAYER_ERROR("failed to get map data");
      // dont free plan->cells this here as it is realloced above and free'd on shutdown
      //free(this->plan->cells);
      return(-1);
    }

    player_map_data_t* mapdata = (player_map_data_t*)msg->GetPayload();
    plan_cell_t* cell;

    // copy the map data
    for(int j=0;j<sj;j++)
    {
      for(int i=0;i<si;i++)
      {
        cell = this->plan->cells + PLAN_INDEX(this->plan,oi+i,oj+j);
        cell->occ_dist = this->plan->max_radius;
        if((cell->occ_state = mapdata->data[j*si + i]) >= 0)
          cell->occ_dist = 0;
      }
    }

    delete msg;

    oi += si;
    if(oi >= this->plan->size_x)
    {
      oi = 0;
      oj += sj;
    }
  }

  this->plan->init();
  this->plan->compute_cspace();
  //draw_cspace(this->plan,"cspace.png");

  if (this->offline_plan) {
    delete this->offline_plan;
    this->offline_plan = NULL;
  }
  return(0);
}

int
Wavefront::GetMapInfo(bool threaded)
{
  Message* msg;
  if(!(msg = this->map_dev->Request(this->InQueue,
                                    PLAYER_MSGTYPE_REQ,
                                    PLAYER_MAP_REQ_GET_INFO,
                                    NULL, 0, NULL, threaded)))
  {
    PLAYER_WARN("failed to get map info");
    this->plan->scale = 0.1;
    this->plan->size_x = 0;
    this->plan->size_y = 0;
    this->plan->origin_x = 0.0;
    this->plan->origin_y = 0.0;
    return(-1);
  }

  player_map_info_t* info = (player_map_info_t*)msg->GetPayload();

  // copy in the map info
  this->plan->scale = info->scale;
  this->plan->size_x = info->width;
  this->plan->size_y = info->height;
  this->plan->origin_x = info->origin.px;
  this->plan->origin_y = info->origin.py;

  delete msg;
  return(0);
}

// setup the underlying map device (i.e., get the map)
int
Wavefront::SetupMap()
{
  // Subscribe to the map device
  if(!(this->map_dev = deviceTable->GetDevice(this->map_id)))
  {
    PLAYER_ERROR("unable to locate suitable map device");
    return(-1);
  }
  if(map_dev->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to map device");
    return(-1);
  }

  // should we get the map now?  if not, we'll wait for it to be pushed to
  // us as data later.
  if(!this->request_map)
    return(0);

  printf("Wavefront: Loading map from map:%d...\n", this->map_id.index);

  // Fill in the map structure

  // first, get the map info
  if(this->GetMapInfo(false) < 0)
    return(-1);
  // Now get the map data, possibly in separate tiles.
  if(this->GetMap(false) < 0)
    return(-1);

  this->have_map = true;
  this->new_map = true;

  puts("Done.");

  return(0);
}

int
Wavefront::ShutdownPosition()
{
  return(this->position_dev->Unsubscribe(this->InQueue));
}

int
Wavefront::ShutdownLocalize()
{
  return(this->localize_dev->Unsubscribe(this->InQueue));
}

int
Wavefront::ShutdownLaser()
{
  return(this->laser_dev->Unsubscribe(this->InQueue));
}

int
Wavefront::ShutdownGraphics2d()
{
  return(this->graphics2d_dev->Unsubscribe(this->InQueue));
}

int
Wavefront::ShutdownMap()
{
  return(this->map_dev->Unsubscribe(this->InQueue));
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message
int
Wavefront::ProcessMessage(QueuePointer & resp_queue,
                          player_msghdr * hdr,
                          void * data)
{
  HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
  HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_PLANNER_REQ_GET_WAYPOINTS);
  HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_PLANNER_REQ_ENABLE);
  HANDLE_CAPABILITY_REQUEST (device_addr, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_PLANNER_CMD_GOAL);
  if (this->have_offline_planner)
  {
    HANDLE_CAPABILITY_REQUEST (offline_planner_id, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_CAPABILITIES_REQ);
    HANDLE_CAPABILITY_REQUEST (offline_planner_id, resp_queue, hdr, data, PLAYER_MSGTYPE_REQ, PLAYER_PLANNER_REQ_GET_WAYPOINTS);
    HANDLE_CAPABILITY_REQUEST (offline_planner_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_PLANNER_CMD_GOAL);
    HANDLE_CAPABILITY_REQUEST (offline_planner_id, resp_queue, hdr, data, PLAYER_MSGTYPE_CMD, PLAYER_PLANNER_CMD_START);
  }
  // Is it new odometry data?
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                           PLAYER_POSITION2D_DATA_STATE,
                           this->position_id))
  {
    this->ProcessPositionData(*(player_position2d_data_t*)data);

    // In case localize_id and position_id are the same
    if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                             PLAYER_POSITION2D_DATA_STATE,
                             this->localize_id))
      this->ProcessLocalizeData(*(player_position2d_data_t*)data);
    return(0);
  }
  // Is it new localization data?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                                PLAYER_POSITION2D_DATA_STATE,
                                this->localize_id))
  {
    this->ProcessLocalizeData(*(player_position2d_data_t*)data);
    return(0);
  }
  // Is it a new goal for the planner?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                PLAYER_PLANNER_CMD_GOAL,
                                this->device_addr))
  {
    if (this->force_map_refresh)
    {
      PLAYER_WARN("requesting new map");

      if (this->plan) delete this->plan;
      this->plan = new plan_t(this->robot_radius+this->safety_dist,
                              this->robot_radius+this->safety_dist,
                              this->max_radius,
                              this->dist_penalty,
                              0.5);

      // Fill in the map structure

      // first, get the map info
      if(this->GetMapInfo(true) < 0) return -1;
      // Now get the map data, possibly in separate tiles.
      if(this->GetMap(true) < 0) return -1;

      this->have_map = true;
      this->new_map = true;
    }
    assert(data);
    this->ProcessCommand(*(player_planner_cmd_t*)data);
    return(0);
  }
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                PLAYER_PLANNER_REQ_GET_WAYPOINTS,
                                this->device_addr))
  {
    player_planner_waypoints_req_t reply;

    reply.waypoints_count = this->waypoints.size();
    reply.waypoints = new player_pose2d_t[this->waypoints.size()];
    double distance = 0.0;
    player_point_2d_t last_p;
    for(int i=0;i<(int)reply.waypoints_count;i++)
    {
      // copy waypoint for length computation
      double px = reply.waypoints[i].px = this->waypoints[i].px;
      double py = reply.waypoints[i].py = this->waypoints[i].py;
      reply.waypoints[i].pa = 0.0;
      if(i != 0) 
      {
        distance += hypot(px-last_p.px,py-last_p.py);
      }
      last_p.px = px;
      last_p.py = py;
    }
    reply.waypoints_distance = distance;

    this->Publish(this->device_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_PLANNER_REQ_GET_WAYPOINTS,
                  (void*)&reply);
    delete [] reply.waypoints;
    return(0);
  }
  // Is it a start position for an offline computed path?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                PLAYER_PLANNER_CMD_START,
                                this->offline_planner_id))
  {
    assert(data);
    this->offline_start = ((player_planner_cmd_t*)data)->goal;
    return(0);
  }
  // Is it a goal position for an offline computed path?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
                                PLAYER_PLANNER_CMD_GOAL,
                                this->offline_planner_id))
  {
    assert(data);
    this->offline_goal = ((player_planner_cmd_t*)data)->goal;
    return(0);
  }

  // Is it a request for an offline computed path?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                PLAYER_PLANNER_REQ_GET_WAYPOINTS,
                                this->offline_planner_id))
  {
    assert(data);

    player_planner_waypoints_req_t reply;

    this->ComputeOfflineWaypoints((player_planner_waypoints_req_t*)data, &reply);

    this->Publish(this->offline_planner_id, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_PLANNER_REQ_GET_WAYPOINTS,
                  (void*)&reply);
    if(reply.waypoints)
      delete [] reply.waypoints;
    return(0);
  }
  // Is it a request to enable or disable the planner?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                PLAYER_PLANNER_REQ_ENABLE,
                                this->device_addr))
  {
    if(hdr->size != sizeof(player_planner_enable_req_t))
    {
      PLAYER_ERROR("incorrect size for planner enable request");
      return(-1);
    }
    player_planner_enable_req_t* enable_req =
            (player_planner_enable_req_t*)data;

    if(enable_req->state)
    {
      this->enable = true;
      PLAYER_MSG0(2,"Robot enabled");
    }
    else
    {
      this->enable = false;
      PLAYER_MSG0(2,"Robot disabled");
    }
    this->Publish(this->device_addr,
                  resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_PLANNER_REQ_ENABLE);
    return(0);
  }
  // Is it new map metadata?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                                PLAYER_MAP_DATA_INFO,
                                this->map_id))
  {
    if(hdr->size != sizeof(player_map_info_t))
    {
      PLAYER_ERROR("incorrect size for map info");
      return(-1);
    }
    //this->ProcessMapInfo(*(player_map_info_t*)data);
    this->new_map_available = true;
    return(0);
  }
  // Is it a pose-stamped laser scan?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                                PLAYER_LASER_DATA_SCANPOSE,
                                this->laser_id))
  {
    player_laser_data_scanpose_t* pdata = (player_laser_data_scanpose_t*)data;
    this->ProcessLaserScan(pdata);
    return(0);
  }
  else
    return(-1);
}

// computes the signed minimum difference between the two angles.
double
Wavefront::angle_diff(double a, double b)
{
  double d1, d2;
  a = NORMALIZE(a);
  b = NORMALIZE(b);
  d1 = a-b;
  d2 = 2*M_PI - std::abs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(std::abs(d1) < std::abs(d2))
    return(d1);
  else
    return(d2);
}

double
static get_time(void)
{
  struct timeval curr;
  gettimeofday(&curr,NULL);
  return(curr.tv_sec + curr.tv_usec / 1e6);
}
