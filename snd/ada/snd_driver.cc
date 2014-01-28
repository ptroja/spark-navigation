/*  Smooth ND driver for Player/Stage
 *
 *  SND Authors:  Joey Durham (algorithm),
 *                Luca Invernizzi (driver implementation)
 *                Piotr Trojanek (code cleanup)
 *
 *  Implemented on top of Player - One Hell of a Robot Server
 *  Copyright (C) 2003  (Brian Gerkey, Andrew Howard)
 *
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_snd snd
 * @brief Smooth Nearness Diagram Navigation

This driver implements the Smooth Nearness Diagram Navigation algorithm.
This algorithm handles local reactive collision-avoidance and
goal-seeking and is designed for non-holonomic robots operating in
tight spaces.  The algorithm is in the following paper:

- J. W. Durham, F. Bullo. Smooth Nearness Diagram Navigation.  IEEE
International Conference on Intelligent Robots and Systems, p 690-695,
2008.  <a href="http://motion.mee.ucsb.edu/pdf/2008a-db.pdf">PDF</a>

SND is based off the Nearness Diagram Navigation method presented in:

- J. Minguez, L. Montano. Nearness Diagram Navigation (ND): Collision
Avoidance in Troublesome Scenarios. IEEE Transactions on Robotics and
Automation, pp 154, 2004.
<a href="http://webdiis.unizar.es/~jminguez/TRAND.pdf">PDF</a>

Briefly, like ND, SND is a gap or discontinuity-based method.  The main
differences are that SND uses a single motion law instead of picking
between six, and SND considers all close obstacle points instead of only the
closest point on each side.  Together these two changes reduce the jerky
turns which can occur with ND, producing smoother motion.

NOTE:  Like ND, SND is a local, reactive path planner.  There are
limits to what it can do on its own.  It can successfully move through
tight, cluttered environments with local traps, but will often get
stuck in larger traps or at ambiguous branch points if asked to
navigate long distances through complex environments.  It infers local
topological information from ranger scans but does not know or remember
global topological information.  For large, complex environments, SND
should be fed a series of waypoints to avoid larger traps.  It could be
combined with an implementation of D* using a coarse, static
representation of the environment.

This driver reads pose information from a @ref interface_position2d
device, sensor data from a @ref interface_ranger device, and writes
commands to a @ref interface_position2d device.  The two
@ref interface_position2d devices can be the same.

The driver itself supports the @interface_position2d interface.  Send
@ref PLAYER_POSITION2D_CMD_POS commands to set the goal pose.  The driver
also accepts @ref PLAYER_POSITION2D_CMD_VEL commands, simply passing them
through to the underlying output device.


@par Compile-time dependencies

- none

@par Provides

- @ref interface_position2d

@par Requires

- "input" @ref interface_position2d : source of pose and velocity information
- "output" @ref interface_position2d : sink for velocity commands to control the robot
- @ref interface_ranger : the ranger to read from

@par Configuration requests

- all @ref interface_position2d requests are passed through to the
underlying "output" @ref interface_position2d device.

@par Configuration file options

- robot_radius (length)
  - Default: 0.25 m
  - Radius of the robot.  In the driver the robot is assumed to be a
    circle of this radius centered around the ranger sensor.  In
    practice, navigation is quite forgiving of rectangular shapes and
    off-center sensors so long as the tightness of squeezes between
    clutter in the environment loosely matches the accuracy of the
    circular approximation.  The robot will stop moving if it believes
    there is an obstacle inside its radius.

- obstacle_avoid_dist (length)
  - Default: 4*robot_radius (m)
  - This is the main parameter of the navigation method, it controls
    the distance at which the robot slows down and begins directly
    avoiding obstacles.  Distance is taken from the boundary of the
    robot, so the robot will navigate away from any obstacle within
    (robot_radius + obstacle_avoid_dist).  4*robot_radius is
    a good starting point, if your robot is fast or your ranger sensor
    is slower than 10Hz or doesn't get reliable readings in the
    environment you may want to increase this to 6x or 8x.

- min_gap_width (length)
  - Default: 2.2*robot_radius (m)
  - The minimum width of gaps the robot will consider navigating.
    Should not be set less than 2*robot_radius, set a bit larger to
    avoid really tight passages.

- max_speed (length/sec)
  - Default: 0.3 m/sec
  - Maximum translational speed of the robot.  Actual speed at any
    instant is reduced based on proxmity to obstacles, rotational
    speed, proximity to goal location.  Maximum safe speed depends on
    the refresh rate and reliability of your ranger sensor, and on the
    choice of obstacle_avoid_dist.  Speeds up to 0.5 m/sec are safe for
    a Hokuyo URG-04LX and other default settings so long as the
    environment does not have many reflective surfaces or narrow
    obstacles.

- max_turn_rate (angle/sec)
  - Default: 60.0 deg/sec
  - Maximum rotational speed of the robot.

- goal_tol (tuple: [length angle])
  - Default: [robot_radius 30.0] (m deg)
  - Respectively, translational and rotational goal tolerance.  When the
    robot is within these bounds of the current target pose, it will
    stop.  If rotational pose is unimportant, set to 360.

@par Example

@verbatim
driver
(
  name "snd"
  provides ["position2d:1"]
  requires ["output:::position2d:0" "input:::position2d:0" "ranger:0"]

  robot_radius 0.25
  obstacle_avoid_dist 1.0
  min_gap_width 0.55

  max_speed 0.4
  max_turn_rate 60

  goal_tol [.3 360]
)
@endverbatim

@author Joseph W. Durham (underlying algorithm), Luca Invernizzi (driver integration)

*/
/** @} */

#include <cstring>
#include <iostream>

#include <unistd.h>
#include <pthread.h>

#include <libplayercore/playercore.h>

#include "snd_driver.h"
#include "clock.h"

////////////////////////////////////////////////////////////////////////////////
//                           BUILDING A SHARED OBJECT                         //
////////////////////////////////////////////////////////////////////////////////

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* SmoothND_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return ((Driver*) (new SmoothND(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void SmoothND_Register(DriverTable* table)
{
	table->AddDriver("snd", SmoothND_Init);
}

// Extra stuff for building a shared object.
/* need the extern to avoid C++ name-mangling  */
extern "C"
{

int player_driver_init(DriverTable* table)
{
	SmoothND_Register(table);
	return(0);
}

}


////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
SmoothND::SmoothND(ConfigFile* cf, int section) :
	ThreadedDriver(cf, section),
	next_goal_ready(false),
	data_odometry_ready(false),
	data_ranger_ready(false),
	config_ranger_ready(false),
	first_goal_has_been_set_to_init_position(false)
{
	statReset(&this->statistics);
	PLAYER_MSG0(1,"INITIALIZING INTERFACE ...");

	// My position interface
	player_devaddr_t m_position_addr;

	// Create my position interface
	if (cf->ReadDeviceAddr(&m_position_addr, section,
	                       "provides", PLAYER_POSITION2D_CODE, -1, NULL) != 0)
	{
		this->SetError(-1);
		PLAYER_MSG0(1,"servi della gleba");
		return;
	}
	if (this->AddInterface(m_position_addr))
	{
		this->SetError(-1);
		PLAYER_MSG0(1,"in una stanza");
		return;
	}
	// Ranger subscription address
	if (cf->ReadDeviceAddr(&(this->ranger_addr), section,
	                       "requires", PLAYER_RANGER_CODE, -1, NULL) != 0)
	{
		this->SetError(-1);
		return;
	}
	// Odometry subscription address
	if (cf->ReadDeviceAddr(&(this->odom_in_addr), section,
	                       "requires", PLAYER_POSITION2D_CODE, -1, "input") != 0)
	{
		this->SetError(-1);
		PLAYER_MSG0(1,"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
		return;
	}
	if (cf->ReadDeviceAddr(&(this->odom_out_addr), section,
	                       "requires", PLAYER_POSITION2D_CODE, -1, "output") != 0)
	{
		PLAYER_MSG0(1,"aaaaaaaaaaaaaaaaaaaaaaaBBBBBBBBBBBBBBBBBBBBaaaaaaaaaaaaa");
		this->SetError(-1);
		return;
	}
	PLAYER_MSG0(1,"INTERFACE INITIALIZED");

	robot_radius = cf->ReadLength(section, "robot_radius", 0.25);
	min_gap_width = cf->ReadLength(section, "min_gap_width", 2.2*robot_radius);
	min_gap_width = std::max(min_gap_width,2*robot_radius);
	obstacle_avoid_dist = cf->ReadLength(section, "obstacle_avoid_dist", 4*robot_radius);
	max_speed = cf->ReadLength(section, "max_speed", 0.3);
	max_turn_rate = DTOR(cf->ReadLength(section, "max_turn_rate", 60.0));
	goal_position_tol = cf->ReadTupleLength(section, "goal_tol", 0, robot_radius);
	goal_angle_tol = DTOR(cf->ReadTupleLength(section, "goal_tol", 1, 30.0));

	if( cf->ReadInt(section, "drive_mode", 0) == 1 )
	{
		drive_mode = DRIVE_OMNI;
	} else {
		drive_mode = DRIVE_DIFF;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.

int SmoothND::Odometry_Setup()
{
	PLAYER_MSG0(3,"SETTING UP THE ODOMETRY  1...");
	// Subscribe to the odometry device
	if(!(this->odom_in_dev = deviceTable->GetDevice(this->odom_in_addr)))
	{
		PLAYER_ERROR("unable to locate suitable odometry device");
		return(-1);
	}
	PLAYER_MSG0(3,"SETTING UP THE ODOMETRY 2 ...");
	if(this->odom_in_dev->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to odometry device");
		return(-1);
	}
	PLAYER_MSG0(3,"SETTING UP THE ODOMETRY 3 ...");
	if(!(this->odom_out_dev = deviceTable->GetDevice(this->odom_out_addr)))
	{
		PLAYER_ERROR("unable to locate suitable odometry device");
		return(-1);
	}
	PLAYER_MSG0(3,"SETTING UP THE ODOMETRY 4 ...");
	if(this->odom_out_dev->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to odometry device");
		return(-1);
	} else {
		PLAYER_MSG0(0, "ok");
	}
//  sleep(3);
	PLAYER_MSG0(3,"SETTING UP THE ODOMETRY 5 ...");

//  // Get the odometry geometry
//  Message* msg;
//  if(!(msg = this->odom_out_dev->Request(this->InQueue,
//                                  PLAYER_MSGTYPE_REQ,
//                                  PLAYER_POSITION2D_REQ_GET_GEOM,
//                                  NULL, 0, NULL,false)))
//  {
//    PLAYER_ERROR("failed to get odometry geometry");
//    if(msg)
//      delete msg;
//    return(-1);
//  }
//PLAYER_MSG0(3,"SETTING UP THE ODOMETRY 6 ...");
//
//  player_position2d_geom_t* geom = (player_position2d_geom_t*)msg->GetPayload();
//  this->robot_geom = *geom;
//  printf("robot geom: %.3f %.3f %.3f %.3f %.3f\n",
//         this->robot_geom.size.sl,
//         this->robot_geom.size.sw,
//         this->robot_geom.pose.px,
//         this->robot_geom.pose.py,
//         RTOD(this->robot_geom.pose.pyaw));
//  delete msg;
	return 0;
}


int SmoothND::Ranger_Setup()
{
	// Subscribe to the ranger device
	PLAYER_MSG0(3,"SETTING UP THE RANGER 1 ...");
	if(!(this->ranger_dev = deviceTable->GetDevice(this->ranger_addr)))
	{
		PLAYER_ERROR("unable to locate suitable ranger device");
		return(-1);
	}
	PLAYER_MSG0(3,"SETTING UP THE RANGER 2 ...");
	if(this->ranger_dev->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to ranger device");
		return(-1);
	}
	PLAYER_MSG0(3,"SETTING UP THE RANGER 3 ...");

	Message* msg;
	// Get the ranger config
	if(!(msg = this->ranger_dev->Request(this->InQueue,
                PLAYER_MSGTYPE_REQ,
                PLAYER_RANGER_REQ_GET_CONFIG,
                NULL, 0, NULL,false)))
	{
		PLAYER_ERROR("failed to get ranger configuration");
		return(-1);
	}

	// Store the ranger pose
	player_ranger_config_t * cfg = (player_ranger_config_t*) msg->GetPayload();
	this->ranger__resolution = cfg->angular_res;
	this->ranger__max_range = cfg->max_range;
	this->config_ranger_ready = true;

	delete msg;

	return 0;
}


int SmoothND::Setup()
{
	PLAYER_MSG0(1,"SETTING UP THE DRIVER ...");

	if (this->Odometry_Setup()!=0 ) return -1;
	if (this->Ranger_Setup()  !=0 ) return -1;

	// Here you do whatever else is necessary to setup the device, like open and
	// configure a serial port.

	// Initialize Ada subsystem
	ndinit();

	PLAYER_MSG0(1,"DRIVER READY");

	// Start the device thread; spawns a new thread and executes
	// SmoothND::Main(), which contains the main loop for the driver.
	this->StartThread();

	return(0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int SmoothND::Shutdown()
{
	// Stop and join the driver thread
	this->StopThread();

	// Finalize Ada subsystem
	ndfinal();

	// Unsubscribe from the ranger
	this->ranger_dev->Unsubscribe(this->InQueue);

	// Unsubscribe from the odometry
	// this->PutPositionCmd(0.0,0.0);
	this->odom_in_dev->Unsubscribe(this->InQueue);
	this->odom_out_dev->Unsubscribe(this->InQueue);

	// Here you would shut the device down by, for example, closing a
	// serial port.

	return(0);
}


// Interface functions

Robot_Proxy *
getProxy(void * arg)
{
  //proxy_c * p = (proxy_c *) arg;
  //Robot_Proxy * rp = (Robot_Proxy *) p->robot_proxy_ptr;
  Robot_Proxy * rp = (Robot_Proxy *) arg;

  return rp;
}

double _getScanRes(void * arg)
{
  return getProxy(arg)->GetScanRes();
}

double _getMaxRange(void * arg)
{
  return getProxy(arg)->GetMaxRange();
}

unsigned int _getScanCount(void * arg)
{
  return getProxy(arg)->GetCount();
}

double _getRange(void * arg, unsigned int i)
{
  return getProxy(arg)->GetRange(i);
}

double _getXPos(void * arg)
{
  return getProxy(arg)->GetXPos();
}

double _getYPos(void * arg)
{
  return getProxy(arg)->GetYPos();
}

double _getYaw(void * arg)
{
  return getProxy(arg)->GetYaw();
}

int _isNewGoalData(void * arg)
{
  return getProxy(arg)->isNewGoalData();
}

int _PeekInputData(void * arg)
{
  return getProxy(arg)->PeekInputData();
}

void _setSpeed(void * arg, double f, double a)
{
  getProxy(arg)->SetSpeed(f, a);
}

void _goalReached(void * arg)
{
  getProxy(arg)->GoalReached();
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void SmoothND::Main()
{
	// Wait for the first goal to be set to the initial position
	while (first_goal_has_been_set_to_init_position==false ||
		this->ranger__ranges_count > 100000 ||
		this->ranger__resolution <= M_PI/1001.0 ||
		this->ranger__resolution > 1.0)
	{
		PLAYER_MSG0(2,".");
		this->Wait();
		this->ProcessMessages();
	}

	//SND_algorithm algo = SND_algorithm(*(Robot_Proxy *) this);
	
	proxy_c p;

	p.robot_radius = this->robot_radius;
	p.min_gap_width = this->min_gap_width;
	p.obstacle_avoid_dist = this->obstacle_avoid_dist;
	p.max_speed = this->max_speed;
	p.max_turn_rate = this->max_turn_rate;
	p.goal_position_tol = this->goal_position_tol;
	p.goal_angle_tol = this->goal_angle_tol;
	p.robot_proxy_ptr = (Robot_Proxy *) this;
	p.getScanRes = _getScanRes;
	p.getMaxRange = _getMaxRange;
	p.getScanCount = _getScanCount;
	p.getRange = _getRange;
	p.getXPos = _getXPos;
	p.getYPos = _getYPos;
	p.getYaw = _getYaw;
	p.isNewGoalData = _isNewGoalData;
	p.PeekInputData = _PeekInputData;
	p.setSpeed = _setSpeed;
	p.goalReached = _goalReached;

	//void * controller = create_c(&p);

	// The main loop; interact with the device here
	for(;;)
	{
		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  Calls ProcessMessage() on each pending
		// message.
		this->ProcessMessages();

		p.goalX = this->goalX;
		p.goalY = this->goalY;
		p.goalA = this->goalA;

		statStart(&this->statistics);
		step_c(&p);
		statStop(&this->statistics);

		// Sleep (or you might, for example, block on a read() instead)
		usleep(50000);
	}
}

////////////////////////////////////////////////////////////////////////////////

void SmoothND::SetSpeedCmd(player_position2d_cmd_vel_t cmd)
{
    // cmd.state = 1;
	// cmd.type = 0;
	this->odom_out_dev->PutMsg(this->InQueue,
	                           PLAYER_MSGTYPE_CMD,
	                           PLAYER_POSITION2D_CMD_VEL,
	                           (void*)&cmd,sizeof(cmd),NULL);
}


////////////////////////////////////////////////////////////////////////////////


int SmoothND::ProcessMessage(QueuePointer & resp_queue,
                             player_msghdr * hdr,
                             void * data)
{
	// Handle new data from the ranger
	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_RANGER_DATA_RANGE,
	                         this->ranger_addr))
	{
		//PLAYER_MSG0(2,"INCOMING RANGER SCAN");
		this->ranger__ranges_count = ((player_ranger_data_range_t*) data)->ranges_count;
		this->ranger__ranges.clear();
		for (uint32_t index = 0; index < this->ranger__ranges_count; ++index)
		{
			this->ranger__ranges.push_back(((player_ranger_data_range_t*) data)->ranges[index]);
		}

		this->data_ranger_ready=true;

		return(0);
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_RANGER_DATA_INTNS,
	                         this->ranger_addr))
	{
		// We do not care about intensivity data.

		return(0);
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
	                         PLAYER_POSITION2D_DATA_STATE,
	                         this->odom_in_addr))
	{
		this->odom_pose = ((player_position2d_data_t*) data)->pos;
		if (this->first_goal_has_been_set_to_init_position == false) {
			this->NewGoalData(this->odom_pose.px, this->odom_pose.py, this->odom_pose.pa);
			this->first_goal_has_been_set_to_init_position=true;
		}

		this->data_odometry_ready=true;

		hdr->addr = this->device_addr;
		this->Publish(hdr,data);
		PLAYER_MSG3(2, "Here I am: (%.3f %.3f %.3f)",this->odom_pose.px,this->odom_pose.py,this->odom_pose.pa);

		return(0);
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
	                         PLAYER_POSITION2D_DATA_STATE,
	                         this->odom_out_addr)) {
		//hdr->addr = this->device_addr;
		//this->Publish(hdr,data);
		return(0);
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, -1, this->device_addr))
	{
		Message* msg;
		if(!(msg = this->odom_out_dev->Request(this->InQueue,
		                                       hdr->type,
		                                       hdr->subtype,
		                                       (void*)data,
		                                       hdr->size,
		                                       &hdr->timestamp)))
		{
			PLAYER_WARN1("failed to forward config request with subtype: %d\n", hdr->subtype);
			return(-1);
		}
		player_msghdr_t* rephdr = msg->GetHeader();
		void* repdata = msg->GetPayload();
		// Copy in our address and forward the response
		rephdr->addr = this->device_addr;
		this->Publish(resp_queue, rephdr, repdata);
		delete msg;
		return(0);
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
	                         PLAYER_POSITION2D_CMD_POS,
	                         this->device_addr))
	{
		player_position2d_cmd_pos_t* cmd = (player_position2d_cmd_pos_t *) data;
//    PLAYER_MSG3(2, "New goal (position): (%.3f %.3f %.3f)",
//              cmd->pos.px,
//              cmd->pos.py,
//              RTOD(cmd->pos.pa));
		this->NewGoalData(cmd->pos.px, cmd->pos.py, cmd->pos.pa);
		return 0;
	}

	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
	                         PLAYER_POSITION2D_CMD_VEL,
	                         this->device_addr))
	{
		const player_position2d_cmd_vel_t* cmd = (player_position2d_cmd_vel_t*)data;
		//PLAYER_MSG3(1, "New goal (speed): (%.3f %.3f %.3f)",
		//          cmd->vel.px,
		//          cmd->vel.py,
		//          RTOD(cmd->vel.pa));
		this->SetSpeedCmd(*cmd);
/*    if( cmd->vel.px < .05 && cmd->vel.pa < DTOR(3.0) )
	  {
        this->SignalNextGoal(this->odom_pose.px,
                                this->odom_pose.py,
                                this->odom_pose.pa);
    }
    else
    {
        double angle = ((cmd->vel.px < 0) ? M_PI : 0) + 2*cmd->vel.pa;
        this->SignalNextGoal(this->odom_pose.px + 10000,
                                this->odom_pose.py + 10000,
                                angle);
    }
*/
		//PLAYER_WARN1("cmd->state:%d\n",cmd->state);
		return 0;
	}

	// Tell the caller that you don't know how to handle this message
	return(-1);
}

//////////////////////////////////////////////////////7

Robot_Proxy::Robot_Proxy(ConfigFile* cf, int section) :
	SmoothND(cf, section)
{	
}

double Robot_Proxy::GetScanRes() const
{
	return this->ranger__resolution;
}

double Robot_Proxy::GetMaxRange() const
{
	return this->ranger__max_range;
}

uint32_t Robot_Proxy::GetCount() const
{
	return this->ranger__ranges_count;
}

double Robot_Proxy::GetRange(uint32_t index) const
{
	return this->ranger__ranges[index];
}

double Robot_Proxy::GetXPos() const
{
	return this->odom_pose.px;
}

double Robot_Proxy::GetYPos() const
{	
	return this->odom_pose.py;
}

double Robot_Proxy::GetYaw() const
{
	return this->odom_pose.pa;
}

void Robot_Proxy::SetSpeed(double velocity_modulus,
                           double velocity_angle)
{
	player_position2d_cmd_vel_t cmd;
	cmd.vel.px=velocity_modulus;
	cmd.vel.py=0.0;
	cmd.vel.pa=velocity_angle;
	this->SetSpeedCmd(cmd);
}

bool SmoothND::isNewGoalData() const
{	
	return next_goal_ready;
}

void SmoothND::NewGoalData(double goalX, double goalY, double goalA)
{
	this->goalX=goalX;
	this->goalY=goalY;
	this->goalA=goalA;

	// PLAYER_MSG0(2,"SIGNAL Goal");
	next_goal_ready = true;
}

void SmoothND::GoalReached()
{
	statPrint(&this->statistics);
	statReset(&this->statistics);
	next_goal_ready = false;
}

bool SmoothND::PeekInputData()
{
	if (this->data_odometry_ready && this->data_ranger_ready && this->config_ranger_ready)
	{
		// PLAYER_MSG0(2,"SIGNAL DATA READY");
		this->data_odometry_ready = false;
		this->data_ranger_ready = false;
		// Ranger's configuration remains ready.
		return true;
	}

	return false;
}
