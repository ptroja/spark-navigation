--
--  Orca-Components: Components for robotics.
--
--  Copyright (C) 2004
--
--  This program is free software; you can redistribute it and/or
--  modify it under the terms of the GNU General Public License
--  as published by the Free Software Foundation; either version 2
--  of the License, or (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
--

package Algorithm is

   -- Data

   -- FIXME: what is the real meaning of this range?
   subtype Hist_Index is Integer range 0 .. 360;

   subtype Hist_Size_Range is Natural range 1 .. 360;

   subtype Speed_Index is Integer range 0 .. 2000;
   subtype Max_Speed_Index is Speed_Index range 1 .. Speed_Index'Last;

   function Integer_Eq (X, Y : Integer) return Boolean;

   -- package Integer_Vector is
   --   new Ada.Containers.Formal_Vectors (Index_Type   => Hist_Index,
   --                                      Element_Type => Integer,
   --                                      "=" => Integer_Eq);

   -- package Speed_Vector is
   --   new Ada.Containers.Formal_Vectors (Index_Type   => Speed_Index,
   --                                      Element_Type => Integer,
   --                                      "=" => Integer_Eq);

   type Border_Pair is
      record
         first, second : Integer;
      end record;

   function Border_Pair_Eq (X, Y : Border_Pair) return Boolean;

     -- package Border_Pair_Vector is
     --   new Ada.Containers.Formal_Vectors (Index_Type   => Hist_Index,
     --                                      Element_Type => Border_Pair,
     --                                      "=" => Border_Pair_Eq
     --                                     );

   subtype Candidate_Index is Integer range 0 .. Hist_Index'Last * 4;

   type Candidate is
      record
         Angle : Float;
         Speed : Speed_Index;
      end record;

   function Candidate_Eq (X, Y : Candidate) return Boolean;

   -- package Candidate_Vector is
   --   new Ada.Containers.Formal_Vectors (Index_Type   => Candidate_Index,
   --                                      Element_Type => Candidate,
   --                                      "=" => Candidate_Eq
   --                                     );

   -- Make Ada.Containers.Count_Type operators visible.
   -- use type Ada.Containers.Count_Type;

   subtype Degree is Positive range 1 .. 360;

   MINIMAL_SECTOR_ANGLE : constant := 1;

   type Sectors_Vector is array (Hist_Index) of Integer;
      -- Integer_Vector.Vector (360 / MINIMAL_SECTOR_ANGLE);

   subtype Cell_Direction_t is Float range -180.0 .. +180.0;


   CELL_SIZE : constant Float := 0.1;
      -- in m, Local occupancy map grid size
   WINDOW_DIAMETER : constant := 61; 
      -- Dimensions of occupancy map (map consists of window_diameter X window_diameter cells
   SECTOR_ANGLE : constant := 5;
      -- Histogram angular resolution, in degrees.
   SAFETY_DIST_0MS : constant Float := 0.1;
      -- in m, The minimum distance the robot is allowed to get to obstacles when stopped.
   SAFETY_DIST_1MS : constant Float := SAFETY_DIST_0MS;
      -- in m, The minimum distance the robot is allowed to get to obstacles when travelling at 1 m/s.
   MAX_SPEED : constant Float := 0.2;
      -- in m/sec, The maximum allowable speed of the robot.
   MAX_SPEED_NARROW_OPENING : constant Float := MAX_SPEED;
      -- in m/sec, The maximum allowable speed of the robot through a narrow opening
   MAX_SPEED_WIDE_OPENING : constant Float := MAX_SPEED;
      -- in m/sec, The maximum allowable speed of the robot through a wide opening
   MAX_ACCELERATION : constant Float := 0.2;
      -- in m/sec/sec, The maximum allowable acceleration of the robot.
   MIN_TURNRATE : constant Float := 10.0;
      -- in deg / sec, The minimum allowable turnrate of the robot.
   MAX_TURNRATE_0MS : constant Float := 40.0;
      -- in deg/sec, The maximum allowable turnrate of the robot when stopped.
   MAX_TURNRATE_1MS : constant Float := MAX_TURNRATE_0MS;
      -- in deg/sec, The maximum allowable turnrate of the robot when travelling 1 m/s.
   MIN_TURN_RADIUS_SAFETY_FACTOR : constant Float := 1.0;
      -- in float
   FREE_SPACE_CUTOFF_0MS : constant Float := 2000000.0;
      -- Unitless value.  The higher the value, the closer the robot will get to obstacles before avoiding (while stopped).
   FREE_SPACE_CUTOFF_1MS : constant Float := FREE_SPACE_CUTOFF_0MS;
      -- Unitless value.  The higher the value, the closer the robot will get to obstacles before avoiding (while travelling at 1 m/s).
   OBS_CUTOFF_0MS : constant Float := FREE_SPACE_CUTOFF_0MS;
   OBS_CUTOFF_1MS : constant Float := FREE_SPACE_CUTOFF_1MS;
   WEIGHT_DESIRED_DIR : constant Float := 5.0;
      -- Bias for the robot to turn to move toward goal position.
   WEIGHT_CURRENT_DIR : constant Float := 3.0;
      -- Bias for the robot to continue moving in current direction of travel.
   DISTANCE_EPSILON : constant Float := 0.5;
      -- in m, Planar distance from the target position that will be considered acceptable.
   ANGLE_EPSILON : constant Float := 10.0;
      -- in deg, Angular difference from target angle that will considered acceptable.

   HIST_SIZE : constant Hist_Size_Range := 72; -- Integer (Float'Unbiased_Rounding (360.0 / Float (SECTOR_ANGLE)));
   HIST_COUNT : constant Integer := 72; -- Integer (Float'Unbiased_Rounding (360.0 / SECTOR_ANGLE));
   HIST_LAST : constant Natural := 71; -- Integer (Float'Unbiased_Rounding (360.0 / Float (SECTOR_ANGLE)) - 1);
   MIN_TURNING_VECTOR_CAPACITY : constant Integer := 2; -- Ada.Containers.Count_Type (MAX_SPEED) + 1;
   CELL_SECTOR_TABLES_LAST : constant Natural := 0; -- (if safety_dist_0ms = safety_dist_1ms then 0 else 19);
   WINDOW_DIAMETER_LAST : constant Natural := Natural (WINDOW_DIAMETER - 1);

   -- FIXME: this is HIST_SIZE in size.
   type History_Array is array (0 .. HIST_LAST) of Float;
   type Cell_Array is array (0 .. WINDOW_DIAMETER_LAST, 0 .. WINDOW_DIAMETER_LAST)
                             of Float;
   type Cell_Sectors is array (0 .. CELL_SECTOR_TABLES_LAST,   -- NUM_CELL_SECTOR_TABLES
                               0 .. WINDOW_DIAMETER_LAST,   -- WINDOW_DIAMETER
                               0 .. WINDOW_DIAMETER_LAST    -- WINDOW_DIAMETER
                              ) of Sectors_Vector; -- (360 / SECTOR_ANGLE)

   type VFH  is
      record

         -- The Histogram.
         -- This is public so that monitoring tools can get at it;
         -- it shouldn't be modified externally.
         -- Sweeps in an anti-clockwise direction.
         Hist : History_Array := (others => 0.0);

         ROBOT_RADIUS : Float;  -- millimeters
         CENTER_X : Natural;    -- cells
         CENTER_Y : Natural;    -- cells

         CELL_WIDTH : Float;    -- millimeters

         MAX_SPEED : Max_Speed_Index := 1;                  -- mm/sec

         SECTOR_ANGLE : Degree;                             -- degrees
         SAFETY_DIST_0MS : Float;                           -- millimeters
         SAFETY_DIST_1MS : Float;                           -- millimeters
         Current_Max_Speed : Max_Speed_Index;               -- mm/sec
         MAX_SPEED_NARROW_OPENING : Speed_Index;            -- mm/sec
         MAX_SPEED_WIDE_OPENING : Speed_Index;              -- mm/sec
         MAX_ACCELERATION : Integer;                        -- mm/sec/sec
         MIN_TURNRATE : Integer;                            -- deg/sec -- not actually used internally

         -- Scale turnrate linearly between these two
         MAX_TURNRATE_0MS : Integer;                        -- deg/sec
         MAX_TURNRATE_1MS : Integer;                        -- deg/sec
         MIN_TURN_RADIUS_SAFETY_FACTOR : Float;
         BINARY_HIST_LOW_0MS, BINARY_HIST_HIGH_0MS : Float;
         BINARY_HIST_LOW_1MS, BINARY_HIST_HIGH_1MS : Float;
         U1, U2 : Float;

         Desired_Angle : Float := 90.0;
         Dist_To_Goal : Float;
         Goal_Distance_Tolerance : Float;
         Picked_Angle, Last_Picked_Angle : Float := 90.0;
         Max_Speed_For_Picked_Angle : Speed_Index;

         -- Radius of dis-allowed circles, either side of the robot, which
         -- we can't enter due to our minimum turning radius.
         Blocked_Circle_Radius : Float;

         -- FIXME: these are Float(WINDOW_DIAMETER,WINDOW_DIAMETER).
         Cell_Direction : Cell_Array ;
         Cell_Base_Mag  : Cell_Array ;
         Cell_Mag       : Cell_Array ;
         Cell_Dist      : Cell_Array ; -- non-negative float.
         Cell_Enlarge   : Cell_Array ;

         -- Cell_Sector[x][y] is a vector of indices to sectors that are effected if cell (x,y) contains
         -- an obstacle.
         -- Cell enlargement is taken into account.
         -- Acess as: Cell_Sector[speed_index][x][y][sector_index]
         Cell_Sector : Cell_Sectors ;

         Last_Binary_Hist : History_Array := (others => 1.0);

         -- Minimum turning radius at different speeds, in millimeters
         Min_Turning_Radius : array (0 .. 2000) of Integer; -- Speed_Index
         Min_Turning_Radius_length : Integer := 0;
            -- Speed_Vector.Vector (MIN_TURNING_VECTOR_CAPACITY); -- MAX_SPEED+1 is size.

         -- Keep track of last update, so we can monitor acceleration
         -- last_update_time : Ada.Real_Time.Time := Ada.Real_Time.Clock;

         last_chosen_speed : Speed_Index := 0;
      end record;

   procedure Init (This : in out VFH);

   -- Choose a new speed and turnrate based on the given laser data and current speed.
   --
   -- Units/Senses:
   --  - goal_direction in degrees, 0deg is to the right.
   --  - goal_distance  in mm.
   --  - goal_distance_tolerance in mm.
   --

   type Laser_Range is array (0 .. 360,
                              0 .. 1) of Float;

   procedure Update (This : in out VFH;
                     laser_ranges : Laser_Range;
                     current_speed : Speed_Index;
                     goal_direction : Float;
                     goal_distance : Float;
                     goal_distance_tolerance : Float;
                     chosen_speed : out Integer;
                     chosen_turnrate : out Integer);
   -- with
   --   Pre => current_speed <= This.Current_Max_Speed and then
   --   This.last_chosen_speed <= This.Current_Max_Speed;

   -- Get methods
   function GetMinTurnrate (This : VFH) return Integer;

   -- Max Turnrate depends on speed
   function GetMaxTurnrate (This : VFH; speed : Integer) return Natural;
   function GetCurrentMaxSpeed (This : VFH) return Integer;

   -- Set methods
   procedure SetRobotRadius (This : in out VFH; robot_radius : Float);
   procedure SetMinTurnrate (This : in out VFH; min_turnrate : Integer);

   -- pragma Export (Cpp, Init, "Init");
   -- pragma Export (CPP, Update, "Update_VFH");
   -- pragma Export (CPP, GetMinTurnrate, "GetMinTurnrate");
   -- pragma Export (CPP, GetMaxTurnrate, "GetMaxTurnrate");
   -- pragma Export (CPP, GetCurrentMaxSpeed, "GetCurrentMaxSpeed");
   -- pragma Export (CPP, SetRobotRadius, "SetRobotRadius");
   -- pragma Export (CPP, SetMinTurnrate, "SetMinTurnrate");

   -- The Histogram.
   -- This is public so that monitoring tools can get at it; it shouldn't
   -- be modified externally.
   -- Sweeps in an anti-clockwise direction.
   -- float *Hist;
   -- function Get_Histogram (This : VFH) return array (<>) of Float;

-- private

   -- Methods

   function Delta_Angle (a1, a2 : Integer) return Float;
   function Delta_Angle (a1, a2 : Float) return Float;

   function Cant_Turn_To_Goal (This : VFH) return Boolean;

   -- Returns false if something got inside the safety distance, else true.
   procedure Calculate_Cells_Mag (This : in out VFH; laser_ranges : Laser_Range; speed : Integer; Ret : out Boolean);
   -- with
   --   Post => This.Current_Max_Speed = This.Current_Max_Speed'Old;
   -- Returns false if something got inside the safety distance, else true.

   procedure Build_Primary_Polar_Histogram (This : in out VFH; laser_ranges : Laser_Range; speed : Natural; Ret : out Boolean);
   -- with
   --   Post => This.Current_Max_Speed = This.Current_Max_Speed'Old;

   procedure Build_Binary_Polar_Histogram (This : in out VFH; speed : Integer);
   -- with
   --   Post => This.Current_Max_Speed = This.Current_Max_Speed'Old;

   procedure Build_Masked_Polar_Histogram (This : in out VFH; speed : Speed_Index);
   -- with
   --   Pre => speed <= Speed_Vector.Last_Index (This.Min_Turning_Radius); -- speed <= This.Current_Max_Speed

   procedure Select_Direction (This : in out VFH);

   procedure Set_Motion (This : VFH; speed : in out Integer; turnrate : out Integer; actual_speed : Integer);
   -- with
   --   Pre => speed <= Speed_Index'Last,
   --   Post => speed in Speed_Index'Range;

   -- AB: This doesn't seem to be implemented anywhere...
   -- int Read_Min_Turning_Radius_From_File(char *filename);

   -- procedure Print_Cells_Dir (This : VFH);
   -- procedure Print_Cells_Mag (This : VFH);
   -- procedure Print_Cells_Dist (This : VFH);
   -- procedure Print_Cells_Sector (This : VFH);
   -- procedure Print_Cells_Enlargement_Angle (This : VFH);
   -- procedure Print_Hist (This : VFH);

   -- Returns the speed index into Cell_Sector, for a given speed in mm/sec.
   -- This exists so that only a few (potentially large) Cell_Sector tables must be stored.
   function Get_Speed_Index (This : VFH; speed : Natural) return Natural;
   -- with
   --   Post => Get_Speed_Index'Result in This.Cell_Sector'Range (1);

   -- Returns the safety dist in mm for this speed.
   function Get_Safety_Dist (This : VFH; speed : Integer) return Integer;

   function Get_Binary_Hist_Low (This : VFH; speed : Integer) return Float;
   function Get_Binary_Hist_High (This : VFH; speed : Integer) return Float;

   function VFH_Predicate (This : VFH) return Boolean;
   -- with
   -- Convention => Ghost;

end Algorithm;
