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

with Ada.Real_Time;
with Ada.Containers.Formal_Vectors;

package Algorithm is

   -- Data
   
   subtype Hist_Index is Integer range 0 .. 360;
   
   subtype Speed_Index is Integer range 0 .. 2000;
   
   function Integer_Eq(X,Y : Integer) return Boolean is (X = Y);
   
   function Float_Eq(X,Y : Float) return Boolean is (X = Y);
         
   package Float_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Hist_Index,
                                                             Element_Type => Float,
                                                             "=" => Float_Eq);
     
   package Integer_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Hist_Index,
                                                               Element_Type => Integer,
                                                               "=" => Integer_Eq);

   package Speed_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Speed_Index,
                                                             Element_Type => Integer,
                                                             "=" => Integer_Eq);

   type Border_Pair is
      record
         first, second : Integer;
      end record;
   
   function Border_Pair_Eq(X,Y : Border_Pair) return Boolean is
     (X.first = Y.first and then X.second = Y.second);

   package Border_Pair_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Hist_Index,
                                                                   Element_Type => Border_Pair,
                                                                   "=" => Border_Pair_Eq
                                                                  );
   
   subtype Candidate_Index is Integer range 0 .. Hist_Index'Last*4;
   
   type Candidate is
      record
         Angle : Float;
         Speed : Integer;
      end record;
   
   function Candidate_Eq(X,Y : Candidate) return Boolean is
     (X.Angle = Y.Angle and then X.Speed = Y.Speed);

   package Candidate_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Candidate_Index,
                                                                 Element_Type => Candidate,
                                                                 "=" => Candidate_Eq
                                                                );   
   
   -- FIXME: this is HIST_SIZE in size.
   type History_Array is array (Integer range <>) of Float;
   
   -- Make Ada.Containers.Count_Type operators visible.
   use Ada.Containers;
   
   FIXED_SECTOR_ANGLE : constant := 1;
   
   type Cell_Sectors is array (Integer range <>,                 -- NUM_CELL_SECTOR_TABLES
                               Integer range <>,                 -- WINDOW_DIAMETER
                               Integer range <>                  -- WINDOW_DIAMETER
                              ) of Integer_Vector.Vector(360/FIXED_SECTOR_ANGLE); -- (360 / SECTOR_ANGLE)
   
   type Cell_Array is array (Integer range <>,
                             Integer range <>) of Float;
   
   type VFH(
            HIST_SIZE : Natural;                -- sectors (over 360deg)
            HIST_COUNT : Ada.Containers.Count_Type;
            HIST_LAST : Natural;
            MIN_TURNING_VECTOR_CAPACITY : Ada.Containers.Count_Type;
            CELL_SECTOR_TABLES_LAST : Natural;
            WINDOW_DIAMETER_LAST : Natural
           ) is
      record
            
         -- The Histogram.
         -- This is public so that monitoring tools can get at it;
         -- it shouldn't be modified externally.
         -- Sweeps in an anti-clockwise direction.
         Hist : History_Array(Natural range 0 .. HIST_LAST) := (others => 0.0);
         
         WINDOW_DIAMETER : Positive := WINDOW_DIAMETER_LAST+1;
      
         ROBOT_RADIUS : Float;               -- millimeters
         CENTER_X : Integer;                 -- cells
         CENTER_Y : Integer;                 -- cells

         CELL_WIDTH : Float;                 -- millimeters
         
         MAX_SPEED : Integer := 0;           -- mm/sec
      
         SECTOR_ANGLE : Positive;             -- degrees
         SAFETY_DIST_0MS : Float;            -- millimeters
         SAFETY_DIST_1MS : Float;            -- millimeters
         Current_Max_Speed : Positive;        -- mm/sec
         MAX_SPEED_NARROW_OPENING : Integer; -- mm/sec
         MAX_SPEED_WIDE_OPENING : Integer;   -- mm/sec
         MAX_ACCELERATION : Integer;         -- mm/sec/sec
         MIN_TURNRATE : Integer;             -- deg/sec -- not actually used internally

         -- Scale turnrate linearly between these two
         MAX_TURNRATE_0MS : Integer;         -- deg/sec
         MAX_TURNRATE_1MS : Integer;         -- deg/sec
         MIN_TURN_RADIUS_SAFETY_FACTOR : Float;
         Binary_Hist_Low_0ms, Binary_Hist_High_0ms : Float;
         Binary_Hist_Low_1ms, Binary_Hist_High_1ms : Float;
         U1, U2 : Float;
         Desired_Angle : Float := 90.0;
         Dist_To_Goal : Float;
         Goal_Distance_Tolerance : Float;
         Picked_Angle, Last_Picked_Angle : Float := 90.0;
         Max_Speed_For_Picked_Angle : Integer;

         -- Radius of dis-allowed circles, either side of the robot, which
         -- we can't enter due to our minimum turning radius.
         Blocked_Circle_Radius : Float;

         -- FIXME: these are Float(WINDOW_DIAMETER,WINDOW_DIAMETER).       
         Cell_Direction : Cell_Array(Integer range 0 .. WINDOW_DIAMETER_LAST,
                                     Integer range 0 .. WINDOW_DIAMETER_LAST);
         Cell_Base_Mag  : Cell_Array(Integer range 0 .. WINDOW_DIAMETER_LAST,
                                     Integer range 0 .. WINDOW_DIAMETER_LAST);
         Cell_Mag       : Cell_Array(Integer range 0 .. WINDOW_DIAMETER_LAST,
                                     Integer range 0 .. WINDOW_DIAMETER_LAST);
         Cell_Dist      : Cell_Array(Integer range 0 .. WINDOW_DIAMETER_LAST,
                                     Integer range 0 .. WINDOW_DIAMETER_LAST);
         Cell_Enlarge   : Cell_Array(Integer range 0 .. WINDOW_DIAMETER_LAST,
                                     Integer range 0 .. WINDOW_DIAMETER_LAST);
         
         -- Cell_Sector[x][y] is a vector of indices to sectors that are effected if cell (x,y) contains
         -- an obstacle.  
         -- Cell enlargement is taken into account.
         -- Acess as: Cell_Sector[speed_index][x][y][sector_index]
         Cell_Sector : Cell_Sectors(
                                    Integer range 0 .. CELL_SECTOR_TABLES_LAST,
                                    Integer range 0 .. WINDOW_DIAMETER_LAST,
                                    Integer range 0 .. WINDOW_DIAMETER_LAST
                                   );

         Last_Binary_Hist : History_Array(Integer range 0 .. HIST_LAST) := (others => 1.0);

         -- Minimum turning radius at different speeds, in millimeters
         Min_Turning_Radius : Speed_Vector.Vector(MIN_TURNING_VECTOR_CAPACITY); -- MAX_SPEED+1 is size.

         -- Keep track of last update, so we can monitor acceleration
         last_update_time : Ada.Real_Time.Time := Ada.Real_Time.Clock;

         last_chosen_speed : Integer := 0;
      end record;

   procedure Init(This : in out VFH);   

   -- Choose a new speed and turnrate based on the given laser data and current speed.
   --
   -- Units/Senses:
   --  - goal_direction in degrees, 0deg is to the right.
   --  - goal_distance  in mm.
   --  - goal_distance_tolerance in mm.
   --

   type Laser_Range is array (Integer range 0 .. 360,
                              Integer range 0 .. 1) of Float;

   procedure Update(This : in out VFH;
                    laser_ranges : Laser_Range; 
                    current_speed : Integer; 
                    goal_direction : Float;
                    goal_distance : Float;
                    goal_distance_tolerance : Float;
                    chosen_speed : out Integer; 
                    chosen_turnrate : out Integer);   

   -- Get methods
   function GetMinTurnrate(This : VFH) return Integer;

   -- Max Turnrate depends on speed
   function GetMaxTurnrate(This : VFH; speed : Integer ) return Natural;
   function GetCurrentMaxSpeed(This : VFH) return Integer;
   

   -- Set methods
   procedure SetRobotRadius( This : in out VFH; robot_radius : Float );
   procedure SetMinTurnrate( This : in out VFH; min_turnrate : Integer);
   
   pragma Export(Cpp, Init, "Init");
   pragma Export(CPP, Update, "Update_VFH");
   pragma Export(CPP, GetMinTurnrate, "GetMinTurnrate");
   pragma Export(CPP, GetMaxTurnrate, "GetMaxTurnrate");
   pragma Export(CPP, GetCurrentMaxSpeed, "GetCurrentMaxSpeed");
   pragma Export(CPP, SetRobotRadius, "SetRobotRadius");
   pragma Export(CPP, SetMinTurnrate, "SetMinTurnrate");

   -- The Histogram.
   -- This is public so that monitoring tools can get at it; it shouldn't
   -- be modified externally.
   -- Sweeps in an anti-clockwise direction.
   -- float *Hist;
   --function Get_Histogram (This : VFH) return array (<>) of Float;

private

   -- Methods

   function Delta_Angle(a1, a2 : Integer) return Float;
   function Delta_Angle(a1, a2 : Float) return Float;

   function Cant_Turn_To_Goal(This : VFH) return Boolean;

   -- Returns false if something got inside the safety distance, else true.
   procedure Calculate_Cells_Mag(This : in out VFH; laser_ranges : Laser_Range; speed : Integer; Ret : out Boolean);
   -- Returns false if something got inside the safety distance, else true.
   procedure Build_Primary_Polar_Histogram(This : in out VFH; laser_ranges : Laser_Range; speed : Natural; Ret : out Boolean);
   procedure Build_Binary_Polar_Histogram(This : in out VFH; speed : Integer);
   procedure Build_Masked_Polar_Histogram(This : in out VFH; speed : Speed_Index)
   with
     Pre => speed <= Speed_Vector.Last_Index(This.Min_Turning_Radius); -- speed <= This.Current_Max_Speed
   procedure Select_Direction(This : in out VFH);
   procedure Set_Motion(This : in out VFH; speed : in out Integer; turnrate : out Integer; actual_speed : Integer);

   -- AB: This doesn't seem to be implemented anywhere...
   -- int Read_Min_Turning_Radius_From_File(char *filename);

   procedure Print_Cells_Dir(This : VFH);
   procedure Print_Cells_Mag(This : VFH);
   procedure Print_Cells_Dist(This : VFH);
   procedure Print_Cells_Sector(This : VFH);
   procedure Print_Cells_Enlargement_Angle(This : VFH);
   procedure Print_Hist(This : VFH);

   -- Returns the speed index into Cell_Sector, for a given speed in mm/sec.
   -- This exists so that only a few (potentially large) Cell_Sector tables must be stored.
   function Get_Speed_Index( This : VFH; speed : Natural) return Natural
   with
     Post => Get_Speed_Index'Result in This.Cell_Sector'Range(1);

   -- Returns the safety dist in mm for this speed.
   function Get_Safety_Dist( This : VFH; speed : Integer ) return Integer;

   function Get_Binary_Hist_Low( This : VFH; speed : Integer ) return Float;
   function Get_Binary_Hist_High( This : VFH; speed : Integer ) return Float;
      
   function VFH_Predicate (This : VFH) return Boolean is
     (This.HIST_SIZE = Integer(This.HIST_COUNT) and then
      This.HIST_LAST = This.HIST_SIZE-1 and then
      This.Hist'Last = This.Last_Binary_Hist'Last and then
      This.Hist'Last = This.HIST_LAST and then
      
      Integer(This.MIN_TURNING_VECTOR_CAPACITY) - 1= This.MAX_SPEED and then
      
      This.WINDOW_DIAMETER - 1 = This.WINDOW_DIAMETER_LAST and then
      
      This.Cell_Direction'Last(1) = THIS.WINDOW_DIAMETER_LAST and then
      This.Cell_Direction'Last(1) = This.Cell_Base_Mag'Last(1) and then
      This.Cell_Direction'Last(1) = This.Cell_Mag'Last(1) and then
      This.Cell_Direction'Last(1) = This.Cell_Dist'Last(1) and then
      This.Cell_Direction'Last(1) = This.Cell_Enlarge'Last(1) and then
      
      This.Cell_Direction'Last(2) = THIS.WINDOW_DIAMETER_LAST and then
      This.Cell_Direction'Last(2) = This.Cell_Base_Mag'Last(2) and then
      This.Cell_Direction'Last(2) = This.Cell_Mag'Last(2) and then
      This.Cell_Direction'Last(2) = This.Cell_Dist'Last(2) and then
      This.Cell_Direction'Last(2) = This.Cell_Enlarge'Last(2) and then
      
      This.CELL_SECTOR_TABLES_LAST = This.Cell_Sector'Last(1) and then
      This.Cell_Direction'Last(1) = This.Cell_Sector'Last(2) and then
      This.Cell_Direction'Last(2) = This.Cell_Sector'Last(3))
   with
     Convention => Ghost;

end Algorithm;
