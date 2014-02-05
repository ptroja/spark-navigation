/*  Smooth ND driver for Player/Stage
 *
 *  SND Authors:  Joey Durham (algorithm) ,
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
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */

#ifndef SND_DRIVER_H
#define SND_DRIVER_H

#include <vector>
#include <libplayercore/playercore.h>
#include "clock.h"

enum DriveTypes {
    DRIVE_DIFF,
    DRIVE_OMNI
};

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class SmoothND : public ThreadedDriver
{
public:

    // Constructor; need that
    SmoothND(ConfigFile* cf, int section);

    // Must implement the following methods.
    virtual int Setup();
    virtual int Shutdown();
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);

private:
    // Main function for device thread.
    virtual void Main();
    int Odometry_Setup();
    int Ranger_Setup();

    // Address of and pointer to the ranger device to which I'll subscribe
    player_devaddr_t ranger_addr;
    player_devaddr_t odom_in_addr;
    player_devaddr_t odom_out_addr;
    Device *ranger_dev;
    Device *odom_in_dev;
    Device *odom_out_dev;
    // player_position2d_geom_t robot_geom;
    bool first_goal_has_been_set_to_init_position;

    stat_t statistics;
protected:
    void SetSpeedCmd(player_position2d_cmd_vel_t cmd);

    player_pose2d_t odom_pose;

    std::vector<double> ranger__ranges;
    double ranger__resolution;
    double ranger__max_range;
    uint32_t ranger__ranges_count;
public:
    double robot_radius;
    double min_gap_width;
    double obstacle_avoid_dist;
    double max_speed;
    double max_turn_rate;
    double goal_position_tol;
    double goal_angle_tol;
    DriveTypes drive_mode;
    double goalX,goalY,goalA;

    bool data_odometry_ready;
    bool data_ranger_ready;
    bool config_ranger_ready;

    bool next_goal_ready;

    bool isNewGoalData() const;
    void NewGoalData(double goalX, double goalY, double goalA);
    void GoalReached();

    bool PeekInputData();
};

class Robot_Proxy : public SmoothND
{
public:
    Robot_Proxy(ConfigFile* cf, int section);

    double   GetScanRes() const;
    double   GetMaxRange() const;
    uint32_t GetCount() const;
    double   GetRange(uint32_t index) const;

    double GetXPos() const;
    double GetYPos() const;
    double GetYaw() const;
    void   SetSpeed(double velocity_modulus,
                    double velocity_angle);
};

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C" int player_driver_init(DriverTable* table);

#endif // SND_DRIVER_H
