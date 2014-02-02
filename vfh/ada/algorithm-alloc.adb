with Ada.Unchecked_Deallocation;
with Interfaces.C; use Interfaces.C;
with Ada.Containers;
with Utils;

--with Ada.Text_IO;

package body Algorithm.Alloc is

   use Ada.Containers;

   function Create_VFH_Ptr
     (cell_size                     : C.C_float;
      window_diameter               : C.int;
      sector_angle                  : C.int;
      safety_dist_0ms               : C.C_float;
      safety_dist_1ms               : C.C_float;
      max_speed                     : C.int;
      max_speed_narrow_opening      : C.int;
      max_speed_wide_opening        : C.int;
      max_acceleration              : C.int;
      min_turnrate                  : C.int;
      max_turnrate_0ms              : C.int;
      max_turnrate_1ms              : C.int;
      min_turn_radius_safety_factor : C.C_float;
      free_space_cutoff_0ms         : C.C_float;
      obs_cutoff_0ms                : C.C_float;
      free_space_cutoff_1ms         : C.C_float;
      obs_cutoff_1ms                : C.C_float;
      weight_desired_dir            : C.C_float;
      weight_current_dir            : C.C_float)
      return                          VFH_Ptr
   is
      use Utils;

      pragma Assert (sector_angle = FIXED_SECTOR_ANGLE);
      This : constant VFH_Ptr := new
        Algorithm.VFH (
                      HIST_SIZE => rint (360.0/Float (sector_angle)),
                      HIST_COUNT => Ada.Containers.Count_Type (360/sector_angle),
                      HIST_LAST => rint (360.0/Float (sector_angle)) - 1,
                      MIN_TURNING_VECTOR_CAPACITY => Ada.Containers.Count_Type (max_speed) + 1,
                      CELL_SECTOR_TABLES_LAST =>
                        (if safety_dist_0ms = safety_dist_1ms
                         -- For the simple case of a fixed safety_dist, keep things simple.
                         then 0
                         -- AB: Made this number up...
                         else 19),
                      WINDOW_DIAMETER_LAST => Integer (window_diameter - 1)
                     );
--                -- These will be set later.
--                ROBOT_RADIUS => 0.0,
--                Dist_To_Goal => 0.0,
--                Goal_Distance_Tolerance => 0.0,
--                Max_Speed_For_Picked_Angle => 0,
--                Blocked_Circle_Radius => 0.0,
--                --Candidate_Angle => Candidate_Angle_Vec,
--                --Candidate_Speed => Integer_Vector.Vector(Ada.Containers.Count_Type(360/sector_angle)),
--                Min_Turning_Radius => Integer_Vector.Vector(MIN_TURNING_VECTOR_SIZE),
--                Cell_Sector => (others => (others => (others => Integer_Vector.Empty_Vector))),
--             );

   begin
      This.all.CENTER_X := Integer (window_diameter / 2);
      This.all.CENTER_Y := Integer (window_diameter / 2);  -- CENTER_X
      This.all.CELL_WIDTH := Float (cell_size);
      This.all.SECTOR_ANGLE := Integer (sector_angle);
      This.all.SAFETY_DIST_0MS := Float (safety_dist_0ms);
      This.all.SAFETY_DIST_1MS := Float (safety_dist_1ms);
      This.all.Current_Max_Speed := Positive (max_speed);
      This.all.MAX_SPEED := Speed_Index (max_speed);
      This.all.MAX_SPEED_NARROW_OPENING := Speed_Index (max_speed_narrow_opening);
      This.all.MAX_SPEED_WIDE_OPENING := Speed_Index (max_speed_wide_opening);
      This.all.MAX_ACCELERATION := Integer (max_acceleration);
      This.all.MIN_TURNRATE := Integer (min_turnrate);
      This.all.MAX_TURNRATE_0MS := Integer (max_turnrate_0ms);
      This.all.MAX_TURNRATE_1MS := Integer (max_turnrate_1ms);
      This.all.MIN_TURN_RADIUS_SAFETY_FACTOR := Float (min_turn_radius_safety_factor);
      This.all.BINARY_HIST_LOW_0MS := Float (free_space_cutoff_0ms);
      This.all.BINARY_HIST_HIGH_0MS := Float (obs_cutoff_0ms);
      This.all.BINARY_HIST_LOW_1MS := Float (free_space_cutoff_1ms);
      This.all.BINARY_HIST_HIGH_1MS := Float (obs_cutoff_1ms);
      This.all.U1 := Float (weight_desired_dir);
      This.all.U2 := Float (weight_current_dir);

      return This;
   end Create_VFH_Ptr;

   procedure Destroy_VFH_Ptr (This : in out VFH_Ptr) is
      procedure Deallocate is new Ada.Unchecked_Deallocation (
         Object => Algorithm.VFH,
         Name   => VFH_Ptr);
   begin
      Deallocate (This);
   end Destroy_VFH_Ptr;

end;
