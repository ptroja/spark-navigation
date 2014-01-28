/*
 *  Orca-Components: Components for robotics.
 *  
 *  Copyright (C) 2004
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
 */

#ifndef VFH_ALGORITHM_H
#define VFH_ALGORITHM_H

#include <vector>
#include <libplayercore/playercore.h>
//#include <libplayercore/playertime.h>

class VFH_Algorithm
{
public:
    VFH_Algorithm( double cell_size,
                   int window_diameter,
                   int sector_angle,
                   double safety_dist_0ms,
                   double safety_dist_1ms, 
                   int max_speed,
                   int max_speed_narrow_opening,
                   int max_speed_wide_opening,
                   int max_acceleration,
                   int min_turnrate,
                   int max_turnrate_0ms,
                   int max_turnrate_1ms,
                   double min_turn_radius_safety_factor,
                   double free_space_cutoff_0ms,
                   double obs_cutoff_0ms,
                   double free_space_cutoff_1ms,
                   double obs_cutoff_1ms,
                   double weight_desired_dir,
                   double weight_current_dir );

    ~VFH_Algorithm();

    void Init();
    
    // Choose a new speed and turnrate based on the given laser data and current speed.
    //
    // Units/Senses:
    //  - goal_direction in degrees, 0deg is to the right.
    //  - goal_distance  in mm.
    //  - goal_distance_tolerance in mm.
    //
    void Update_VFH(double laser_ranges[361][2], 
                    int current_speed,  
                    float goal_direction,
                    float goal_distance,
                    float goal_distance_tolerance,
                    int &chosen_speed, 
                    int &chosen_turnrate );

    // Get methods
    int   GetMinTurnrate() const { return MIN_TURNRATE; }

    // Max Turnrate depends on speed
    int GetMaxTurnrate( int speed ) const;
    int GetCurrentMaxSpeed() const { return Current_Max_Speed; }

    // Set methods
    void SetRobotRadius( float robot_radius ) { this->ROBOT_RADIUS = robot_radius; }
    void SetMinTurnrate( int min_turnrate ) { MIN_TURNRATE = min_turnrate; }
    void SetCurrentMaxSpeed( int Max_Speed );

    // The Histogram.
    // This is public so that monitoring tools can get at it; it shouldn't
    // be modified externally.
    // Sweeps in an anti-clockwise direction.
    float *Hist;

private:

    // Methods

    void VFH_Allocate();

    float Delta_Angle(int a1, int a2) const;
    float Delta_Angle(float a1, float a2) const;

    bool Cant_Turn_To_Goal() const;

    // Returns false if something got inside the safety distance, else true.
    bool Calculate_Cells_Mag( double laser_ranges[361][2], int speed );
    // Returns false if something got inside the safety distance, else true.
    bool Build_Primary_Polar_Histogram( double laser_ranges[361][2], int speed );
    void Build_Binary_Polar_Histogram(int speed);
    void Build_Masked_Polar_Histogram(int speed);
    void Select_Candidate_Angle();
    void Select_Direction();
    void Set_Motion( int &speed, int &turnrate, int current_speed );

    // AB: This doesn't seem to be implemented anywhere...
    // int Read_Min_Turning_Radius_From_File(char *filename);

    void Print_Cells_Dir() const;
    void Print_Cells_Mag() const;
    void Print_Cells_Dist() const;
    void Print_Cells_Sector() const;
    void Print_Cells_Enlargement_Angle() const;
    void Print_Hist() const;

    // Returns the speed index into Cell_Sector, for a given speed in mm/sec.
    // This exists so that only a few (potentially large) Cell_Sector tables must be stored.
    int Get_Speed_Index( int speed ) const;

    // Returns the safety dist in mm for this speed.
    int Get_Safety_Dist( int speed ) const;

    float Get_Binary_Hist_Low( int speed ) const;
    float Get_Binary_Hist_High( int speed ) const;

    // Data

    float ROBOT_RADIUS;                 // millimeters
    const int CENTER_X;                 // cells
    const int CENTER_Y;                 // cells

    const float CELL_WIDTH;             // millimeters
    const int WINDOW_DIAMETER;          // cells
    const int SECTOR_ANGLE;             // degrees
    const int HIST_SIZE;                // sectors (over 360deg)
    const float SAFETY_DIST_0MS;        // millimeters
    const float SAFETY_DIST_1MS;        // millimeters
    int Current_Max_Speed;              // mm/sec
    const int MAX_SPEED;                // mm/sec
    const int MAX_SPEED_NARROW_OPENING; // mm/sec
    const int MAX_SPEED_WIDE_OPENING;   // mm/sec
    const int MAX_ACCELERATION;   // mm/sec/sec
    int MIN_TURNRATE;             // deg/sec -- not actually used internally

    const int NUM_CELL_SECTOR_TABLES;

    // Scale turnrate linearly between these two
    int MAX_TURNRATE_0MS;       // deg/sec
    int MAX_TURNRATE_1MS;       // deg/sec
    double MIN_TURN_RADIUS_SAFETY_FACTOR;
    float Binary_Hist_Low_0ms, Binary_Hist_High_0ms;
    float Binary_Hist_Low_1ms, Binary_Hist_High_1ms;
    float U1, U2;
    float Desired_Angle, Dist_To_Goal, Goal_Distance_Tolerance;
    float Picked_Angle, Last_Picked_Angle;
    int   Max_Speed_For_Picked_Angle;

    // Radius of dis-allowed circles, either side of the robot, which
    // we can't enter due to our minimum turning radius.
    float Blocked_Circle_Radius;

    std::vector<std::vector<float> > Cell_Direction;
    std::vector<std::vector<float> > Cell_Base_Mag;
    std::vector<std::vector<float> > Cell_Mag;
    std::vector<std::vector<float> > Cell_Dist;      // millimetres
    std::vector<std::vector<float> > Cell_Enlarge;

    // Cell_Sector[x][y] is a vector of indices to sectors that are effected if cell (x,y) contains
    // an obstacle.  
    // Cell enlargement is taken into account.
    // Access as: Cell_Sector[speed_index][x][y][sector_index]
    std::vector<std::vector<std::vector<std::vector<int> > > > Cell_Sector;
    std::vector<float> Candidate_Angle;
    std::vector<int> Candidate_Speed;

    float *Last_Binary_Hist;

    // Minimum turning radius at different speeds, in millimeters
    std::vector<int> Min_Turning_Radius;

    // Keep track of last update, so we can monitor acceleration
    struct timeval last_update_time;

    int last_chosen_speed;
};

#endif
