with Formal.Numerics.Elementary_Functions;
with Ada.Text_IO;
with Ada.Float_Text_IO;
with Utils;

use Ada.Text_IO;
use Formal.Numerics.Elementary_Functions;
use Utils;

package body Algorithm is

   M_PI : constant := Formal.Numerics.Pi;

   use Integer_Vector;
   use Speed_Vector;

   procedure Tab is
   begin
      Put (ASCII.HT);
   end Tab;

   type Half_Option is (Inclusive, Exclusive);

   function Up_To_Half (Size : Positive; Half : Half_Option) return Natural
   with
     Post => Up_To_Half'Result < Size;

   function Up_To_Half (Size : Positive; Half : Half_Option) return Natural
   is
   begin
      case Half is
         when Inclusive =>
            return (if Size = 1 then 0 else (Size - 1)/2+1);
         when Exclusive =>
            return (Size - 1)/2;
      end case;
   end;

   ----------
   -- Init --
   ----------

   procedure Init (This : in out VFH) is
      pragma Spark_Mode (On);

      function Cell_Direction (X, Y : Integer) return Float is
         val : Float;
      begin
         if X < This.CENTER_X then
            if Y < This.CENTER_Y then
               val := Arctan (Float (This.CENTER_Y - Y) / Float (This.CENTER_X - X));
               val := val * (360.0 / (2.0*M_PI));
               val := 180.0 - val;
            elsif Y = This.CENTER_Y then
               val := 180.0;
            elsif Y > This.CENTER_Y then
               val := Arctan (Float (Y - This.CENTER_Y) / Float (This.CENTER_X - X));
               val := val * (360.0 / (2.0*M_PI));
               val := 180.0 + val;
            end if;
         elsif X = This.CENTER_X then
            if Y < This.CENTER_Y then
               val := 90.0;
            elsif Y = This.CENTER_Y then
               val := -1.0;
            elsif Y > This.CENTER_Y then
               val := 270.0;
            end if;
         elsif X > This.CENTER_X then
            if Y < This.CENTER_Y then
               val := Arctan (Float (This.CENTER_Y - Y) / Float (X - This.CENTER_X));
               val := val * (360.0 / (2.0*M_PI));
            elsif Y = This.CENTER_Y then
               val := 0.0;
            elsif Y > This.CENTER_Y then
               val := Arctan (Float (Y - This.CENTER_Y) / Float (X - This.CENTER_X));
               val := val * (360.0 / (2.0*M_PI));
               val := 360.0 - val;
            end if;
         end if;
         return val;
      end;

      procedure SetCurrentMaxSpeed is
      begin
         pragma Assert (VFH_Predicate (This));
         This.Current_Max_Speed := This.MAX_SPEED;

         -- This should always succeed.
         Reserve_Capacity (This.Min_Turning_Radius,
                           Ada.Containers.Count_Type (This.Current_Max_Speed) + 1);
         -- Instead of C++'s resize and set, in Ada we clear and append.
         Clear (This.Min_Turning_Radius);

         -- small chunks of forward movements and turns-in-place used to
         -- estimate turning radius, coz I'm too lazy to screw around with limits -> 0.

         --
         -- Calculate the turning radius, indexed by speed.
         -- Probably don't need it to be precise (changing in 1mm increments).
         --
         -- WARNING: This assumes that the max_turnrate that has been set for VFH is
         --          accurate.
         --
         pragma Assert (Capacity (This.Min_Turning_Radius) = This.MIN_TURNING_VECTOR_CAPACITY);
         for x in Integer range 0 .. This.Current_Max_Speed loop
            pragma Loop_Invariant (Length (This.Min_Turning_Radius) = Ada.Containers.Count_Type (x) and then
                                   Capacity (This.Min_Turning_Radius) = Capacity (This.Min_Turning_Radius)'Loop_Entry);
            declare
               dx : constant Float := Float (x) / 1.0e6; -- dx in m/millisec
               dtheta : constant Float := ((M_PI / 180.0)*Float (GetMaxTurnrate (This, x)))/1000.0; -- dTheta in radians/millisec
               val : constant Integer := Integer (((dx / Tan (dtheta))*1000.0) * This.MIN_TURN_RADIUS_SAFETY_FACTOR); -- in mm
            begin
               pragma Assert_And_Cut (True);
               Append (This.Min_Turning_Radius, val);
            end;
         end loop;
      end SetCurrentMaxSpeed;
   begin
      pragma Assert (VFH_Predicate (This));

      SetCurrentMaxSpeed;

      -- For the following calcs:
      --   - (x,y) = (0,0)   is to the front-left of the robot
      --   - (x,y) = (max,0) is to the front-right of the robot
      --
      for x in This.Cell_Mag'Range (1) loop
         for y in This.Cell_Mag'Range (2) loop
            This.Cell_Mag (x, y) := 0.0;
            This.Cell_Dist (x, y) := Hypot (Float (This.CENTER_X - x), Float (This.CENTER_Y - y)) * This.CELL_WIDTH;
            This.Cell_Base_Mag (x, y) := ((3000.0 - This.Cell_Dist (x, y))**4) / 100000000.0;

            This.Cell_Direction (x, y) := Cell_Direction (x, y);

            -- For the case where we have a speed-dependent safety_dist, calculate all tables
            for cell_sector_tablenum in This.Cell_Sector'Range (1) loop

               declare
                  max_speed_this_table : constant Integer :=
                    Integer ((Float (cell_sector_tablenum + 1) / Float (This.Cell_Sector'Length (1))) *
                               Float (This.MAX_SPEED));

                  Cell_Enlarge_OK : Boolean;
               begin
                  -- printf("cell_sector_tablenum: %d, max_speed: %d, safety_dist: %d\n",
                  -- cell_sector_tablenum,max_speed_this_table,Get_Safety_Dist(max_speed_this_table));

                  -- Set Cell_Enlarge to the _angle_ by which a an obstacle must be
                  -- enlarged for this cell, at this speed
                  if This.Cell_Dist (x, y) > 0.0 then
                     declare
                        r : constant Float := This.ROBOT_RADIUS + Float (Get_Safety_Dist (This, max_speed_this_table));
                        arg : constant Float := r / This.Cell_Dist (x, y);
                     begin
                        if abs (arg) <= 1.0 then
                           -- This.Cell_Enlarge(x,y) := arctan( r / This.Cell_Dist(x,y) ) * (180/M_PI);
                           This.Cell_Enlarge (x, y) := Arcsin (r / This.Cell_Dist (x, y)) * (180.0/M_PI);
                           Cell_Enlarge_OK := True;
                        else
                           Cell_Enlarge_OK := False;
                        end if;
                     end;
                  else
                     This.Cell_Enlarge (x, y) := 0.0;
                     Cell_Enlarge_OK := True;
                  end if;
                  pragma Assert_And_Cut (True);
                  Clear (This.Cell_Sector (cell_sector_tablenum, x, y)); -- FIXME: this seems unnecessary.

                  if Cell_Enlarge_OK then
                     declare
                        plus_dir : constant Float := This.Cell_Direction (x, y) + This.Cell_Enlarge (x, y);
                        neg_dir  : constant Float := This.Cell_Direction (x, y) - This.Cell_Enlarge (x, y);
                     begin
                        pragma Assert (Capacity (This.Cell_Sector (cell_sector_tablenum, x, y)) = 360);
                        for i in Integer range 0 .. (360 / This.SECTOR_ANGLE)-1 loop
                           pragma Loop_Invariant (Length (This.Cell_Sector (cell_sector_tablenum, x, y)) <= Ada.Containers.Count_Type (i) and then
                                                  Capacity (This.Cell_Sector (cell_sector_tablenum, x, y)) = 360);
                           -- Set plus_sector and neg_sector to the angles to the two adjacent sectors
                           declare
                              -- FIXME: I should be given with Global aspect and not a variable,
                              -- but GNATProve does not allow to use loop variables in this aspect.
                              function Append_Or_Not(I : Integer) return Boolean is
                                 plus_sector : constant Float := Float (i + 1) * Float (This.SECTOR_ANGLE);
                                 neg_sector : constant Float := Float (i) * Float (This.SECTOR_ANGLE);
                                 neg_sector_to_neg_dir, neg_sector_to_plus_dir : Float;
                                 plus_sector_to_neg_dir, plus_sector_to_plus_dir : Float;
                              begin
                                 if neg_sector - neg_dir > 180.0 then
                                    neg_sector_to_neg_dir := neg_dir - (neg_sector - 360.0);
                                 else
                                    if neg_dir - neg_sector > 180.0 then
                                       neg_sector_to_neg_dir := neg_sector - (neg_dir + 360.0);
                                    else
                                       neg_sector_to_neg_dir := neg_dir - neg_sector;
                                    end if;
                                 end if;

                                 if plus_sector - neg_dir > 180.0 then
                                    plus_sector_to_neg_dir := neg_dir - (plus_sector - 360.0);
                                 else
                                    if neg_dir - plus_sector > 180.0 then
                                       plus_sector_to_neg_dir := plus_sector - (neg_dir + 360.0);
                                    else
                                       plus_sector_to_neg_dir := neg_dir - plus_sector;
                                    end if;
                                 end if;

                                 if plus_sector - plus_dir > 180.0 then
                                    plus_sector_to_plus_dir := plus_dir - (plus_sector - 360.0);
                                 else
                                    if plus_dir - plus_sector > 180.0 then
                                       plus_sector_to_plus_dir := plus_sector - (plus_dir + 360.0);
                                    else
                                       plus_sector_to_plus_dir := plus_dir - plus_sector;
                                    end if;
                                 end if;

                                 if neg_sector - plus_dir > 180.0 then
                                    neg_sector_to_plus_dir := plus_dir - (neg_sector - 360.0);
                                 else
                                    if plus_dir - neg_sector > 180.0 then
                                       neg_sector_to_plus_dir := neg_sector - (plus_dir + 360.0);
                                    else
                                       neg_sector_to_plus_dir := plus_dir - neg_sector;
                                    end if;
                                 end if;

                                 declare
                                    neg_dir_bw : constant Boolean := neg_sector_to_neg_dir >= 0.0 and then plus_sector_to_neg_dir <= 0.0;
                                    dir_around_sector : constant Boolean := neg_sector_to_neg_dir <= 0.0 and then neg_sector_to_plus_dir >= 0.0;
                                    plus_dir_bw : constant Boolean :=
                                      (neg_sector_to_plus_dir >= 0.0 and then plus_sector_to_plus_dir <= 0.0)
                                      or else
                                        (plus_sector_to_neg_dir <= 0.0 and then plus_sector_to_plus_dir >= 0.0);
                                 begin
                                    return plus_dir_bw or else neg_dir_bw or else dir_around_sector;
                                 end;
                              end;

                           begin
                              if Append_Or_Not(I) then
                                 Append (This.Cell_Sector (cell_sector_tablenum, x, y), i);
                              end if;
                           end;
                        end loop;
                     end;

                  end if;
               end;
            end loop;
         end loop;
      end loop;

      This.last_update_time := Ada.Real_Time.Clock;

      -- Print_Cells_Sector(This);
   end Init;

   ------------
   -- Update --
   ------------

   procedure Update
     (This : in out VFH;
      laser_ranges : Laser_Range;
      current_speed : Speed_Index;
      goal_direction : Float;
      goal_distance : Float;
      goal_distance_tolerance : Float;
      chosen_speed : out Integer;
      chosen_turnrate : out Integer)
   is
      print : constant Boolean := false;
      current_pos_speed : constant Speed_Index :=
        (if current_speed < This.last_chosen_speed
         then This.last_chosen_speed
         else current_speed);

      Now : constant Ada.Real_Time.Time := Ada.Real_Time.Clock;
      diff : Ada.Real_Time.Time_Span;

      diffSeconds : Float;
      speed_incr : Integer;

      use Ada.Real_Time;
      Build_Primary_Polar_Histogram_Ret : Boolean;
   begin

      This.Desired_Angle := goal_direction;
      This.Dist_To_Goal  := goal_distance;
      This.Goal_Distance_Tolerance := goal_distance_tolerance;

      --
      -- Set current_pos_speed to the maximum of
      -- the set point (last_chosen_speed) and the current actual speed.
      -- This ensures conservative behaviour if the set point somehow ramps up beyond
      -- the actual speed.
      -- Ensure that this speed is non-negative.
      --
      pragma Assert_And_Cut (current_pos_speed <= This.Current_Max_Speed);

      -- printf("Update_VFH: current_pos_speed = %d\n",current_pos_speed);

      -- Work out how much time has elapsed since the last update,
      -- so we know how much to increase speed by, given MAX_ACCELERATION.

      diff := Now - This.last_update_time;
      diffSeconds := Float (Ada.Real_Time.To_Duration (diff));

      This.last_update_time := Now;

      Build_Primary_Polar_Histogram (This, laser_ranges, current_pos_speed, Build_Primary_Polar_Histogram_Ret);
      if not Build_Primary_Polar_Histogram_Ret then
         -- Something's inside our safety distance: brake hard and
         -- turn on the spot
         This.Picked_Angle := This.Last_Picked_Angle;
         This.Max_Speed_For_Picked_Angle := 0;
      else
         if print then
            Ada.Text_IO.Put_Line ("Primary Histogram");
            Print_Hist (This);
         end if;

         Build_Binary_Polar_Histogram (This, current_pos_speed);

         if print then
            Ada.Text_IO.Put_Line ("Binary Histogram");
            Print_Hist (This);
         end if;

         pragma Assert (Last_Index (This.Min_Turning_Radius) = This.Current_Max_Speed);
         Build_Masked_Polar_Histogram (This, current_pos_speed);

         if print then
            Ada.Text_IO.Put_Line ("Masked Histogram");
            Print_Hist (This);
         end if;

         -- Sets Picked_Angle, Last_Picked_Angle, and Max_Speed_For_Picked_Angle.
         Select_Direction (This);
      end if;

      --  ("Picked Angle: %f\n", Picked_Angle);

      --
      -- OK, so now we've chosen a direction.  Time to choose a speed.
      --

      -- How much can we change our speed by?
      if diffSeconds > 0.3 or else diffSeconds < 0.0 then
         -- Either this is the first time we've been updated, or something's a bit screwy and
         -- update hasn't been called for a while.  Don't want a sudden burst of acceleration,
         -- so better to just pick a small value this time, calculate properly next time.
         speed_incr := 10;
      else
         speed_incr := Integer (Float (This.MAX_ACCELERATION) * diffSeconds);
      end if;

      if Cant_Turn_To_Goal (This) then
         -- The goal's too close -- we can't turn tightly enough to get to it,
         -- so slow down.
         speed_incr := -speed_incr;
      end if;

      -- Accelerate (if we're not already at Max_Speed_For_Picked_Angle).
      chosen_speed := Integer'Min (This.last_chosen_speed + speed_incr, This.Max_Speed_For_Picked_Angle);

      -- printf("Max Speed for picked angle: %d\n",Max_Speed_For_Picked_Angle);

      -- Set the chosen_turnrate, and possibly modify the chosen_speed
      Set_Motion (This, chosen_speed, chosen_turnrate, current_pos_speed);

      This.last_chosen_speed := chosen_speed;

      if print then
         Ada.Text_IO.Put ("CHOSEN: SPEED: ");
         Ada.Text_IO.Put (Integer'Image (chosen_speed)); Tab;
         Ada.Text_IO.Put (" TURNRATE: ");
         Ada.Text_IO.Put (Integer'Image (chosen_turnrate));
         Ada.Text_IO.New_Line;
      end if;
   end Update;

   --------------------
   -- GetMinTurnrate --
   --------------------

   function GetMinTurnrate (This : VFH) return Integer is
   begin
      return This.MIN_TURNRATE;
   end GetMinTurnrate;

   --------------------
   -- GetMaxTurnrate --
   --------------------

   function GetMaxTurnrate (This : VFH; speed : Integer) return Natural is
      val : Integer :=
        This.MAX_TURNRATE_0MS -
          Integer (Float (speed * (This.MAX_TURNRATE_0MS - This.MAX_TURNRATE_1MS)) / 1000.0);
   begin
      if val < 0 then
         val := 0;
      end if;

      return val;
   end GetMaxTurnrate;

   ------------------------
   -- GetCurrentMaxSpeed --
   ------------------------

   function GetCurrentMaxSpeed (This : VFH) return Integer is
   begin
      return This.Current_Max_Speed;
   end GetCurrentMaxSpeed;

   --------------------
   -- SetRobotRadius --
   --------------------

   procedure SetRobotRadius (This : in out VFH; robot_radius : Float) is
   begin
      This.ROBOT_RADIUS := robot_radius;
   end SetRobotRadius;

   --------------------
   -- SetMinTurnrate --
   --------------------

   procedure SetMinTurnrate (This : in out VFH; min_turnrate : Integer) is
   begin
      This.MIN_TURNRATE := min_turnrate;
   end SetMinTurnrate;

   -----------------
   -- Delta_Angle --
   -----------------

   function Delta_Angle (a1, a2 : Integer) return Float is
   begin
      return Delta_Angle (Float (a1), Float (a2));
   end Delta_Angle;

   -----------------
   -- Delta_Angle --
   -----------------

   function Delta_Angle (a1, a2 : Float) return Float is
      diff : Float := a2 - a1;
   begin
      if diff > 180.0 then
         diff := diff - 360.0;
      elsif diff < -180.0 then
         diff := diff + 360.0;
      end if;

      return diff;
   end Delta_Angle;

   -----------------------
   -- Cant_Turn_To_Goal --
   -----------------------

   --
   -- Are we going too fast, such that we'll overshoot before we can turn to the goal?
   --

   function Cant_Turn_To_Goal (This : VFH) return Boolean is
      -- Coords of goal in local coord system:
      goal_x : constant Float := This.Dist_To_Goal * Cos ((DTOR (This.Desired_Angle)));
      goal_y : constant Float := This.Dist_To_Goal * Sin ((DTOR (This.Desired_Angle)));
   begin
      -- Calculate this by seeing if the goal is inside the blocked circles
      -- (circles we can't enter because we're going too fast).  Radii set
      -- by Build_Masked_Polar_Histogram.

      -- AlexB: Is this useful?
      --     if ( goal_y < 0 )
      --     {
      --         Put_Line("Goal behind");
      --         return true;
      --     }

      -- This is the distance between the centre of the goal and
      -- the centre of the blocked circle
      declare
         dist_between_centres : Float;
      begin

         --     printf("Cant_Turn_To_Goal: Dist_To_Goal = %f\n",Dist_To_Goal);
         --     printf("Cant_Turn_To_Goal: Angle_To_Goal = %f\n",Desired_Angle);
         --     printf("Cant_Turn_To_Goal: Blocked_Circle_Radius = %f\n",Blocked_Circle_Radius);

         -- right circle
         dist_between_centres := Hypot (goal_x - This.Blocked_Circle_Radius, goal_y);
         if dist_between_centres + This.Goal_Distance_Tolerance < This.Blocked_Circle_Radius then
            -- Put_Line("Goal close & right");
            return true;
         end if;

         -- left circle
         dist_between_centres := Hypot (-goal_x - This.Blocked_Circle_Radius, goal_y);
         if dist_between_centres + This.Goal_Distance_Tolerance < This.Blocked_Circle_Radius then
            -- Put_Line("Goal close & left.");
            return true;
         end if;
      end;

      return false;
   end Cant_Turn_To_Goal;

   -------------------------
   -- Calculate_Cells_Mag --
   -------------------------

   procedure Calculate_Cells_Mag
     (This : in out VFH;
      laser_ranges : Laser_Range;
      speed : Integer;
      Ret : out Boolean)
   is
      -- AB: This is a bit dodgy...  Makes it possible to miss really skinny obstacles, since if the
      --     resolution of the cells is finer than the resolution of laser_ranges, some ranges might be missed.
      --     Rather than looping over the cells, should perhaps loop over the laser_ranges.

      r : constant Float := This.ROBOT_RADIUS + Float (Get_Safety_Dist (This, speed));
   begin
      --Put_Line("Laser Ranges");
      --Put_Line("************");
      --for x in range laser_ranges'Range(1) loop
      --Put_Line(Integer'Image(x) & ": " & Float'Image(laser_ranges(x,0)));
      --end loop;
      pragma Assert (VFH_Predicate (This));
      -- Only deal with the cells in front of the robot, since we can't sense behind.
      for x in Integer range 0 .. This.WINDOW_DIAMETER - 1 loop pragma Loop_Invariant (VFH_Predicate (This));
         for y in Integer range
           0 .. Up_To_Half (This.Cell_Direction'Length (2), Exclusive) loop pragma Loop_Invariant (VFH_Predicate (This));
            --Put(Integer'Image(x) & " x " & Integer'Image(y) & ": ");
            --Put(Float'Image(This.Cell_Dist(x,y)) & ", " & Float'Image(This.Cell_Direction(x,y)));
            --New_Line;
            if This.Cell_Direction (x, y) >= 0.0 and then
              This.Cell_Dist (x, y) + This.CELL_WIDTH / 2.0 >
              laser_ranges (rint (This.Cell_Direction (x, y) * 2.0), 0)
            then
               if This.Cell_Dist (x, y) < r and then (not (x = This.CENTER_X and then y = This.CENTER_Y)) then
                  -- printf("Cell %d,%d: Cell_Dist is %f, range is %f (minimum is %f): too close...\n",
                  --        x,
                  --        y,
                  --        Cell_Dist[x][y] + CELL_WIDTH / 2.0,
                  --        laser_ranges[(int)rint(Cell_Direction[x][y] * 2.0)][0],
                  --        r);

                  -- Damn, something got inside our safety_distance...
                  -- Short-circuit this process.
                  Ret := false; return;
               else
                  This.Cell_Mag (x, y) := This.Cell_Base_Mag (x, y);
               end if;
            else
               This.Cell_Mag (x, y) := 0.0;
            end if;
         end loop;
      end loop;

      Ret := true;
   end Calculate_Cells_Mag;

   -----------------------------------
   -- Build_Primary_Polar_Histogram --
   -----------------------------------

   procedure Build_Primary_Polar_Histogram
     (This : in out VFH;
      laser_ranges : Laser_Range;
      speed : Natural;
      Ret : out Boolean)
   is
      -- index into the vector of Cell_Sector tables
      speed_idx : constant Integer := Get_Speed_Index (This, speed);
      Calculate_Cells_Mag_Ret : Boolean;
   begin
      Calculate_Cells_Mag (This, laser_ranges, speed, Calculate_Cells_Mag_Ret);
      if not Calculate_Cells_Mag_Ret then
         -- set Hist to all blocked
         This.Hist := (others => 1.0);
         Ret := False; return;
      end if;

      -- FIXED: we do this in the 'else' branch of the proceeding test.
      This.Hist := (others => 0.0);

      --  Print_Cells_Dist(This);
      --  Print_Cells_Dir(This);
      --  Print_Cells_Mag(This);
      --  Print_Cells_Sector(This);
      --  Print_Cells_Enlargement_Angle(This);
      pragma Assert_And_Cut (VFH_Predicate (This));

      -- Use a local procedure with parameters mode to avoid loop invariants,
      -- which allocates huge local copies of This.Cell_Sector:
      -- pragma Loop_Invariant (This.Cell_Sector = This.Cell_Sector'Loop_Entry);
      declare
         subtype Hist_t is History_Array (Natural range 0 .. This.HIST_LAST);
         subtype Cell_Sector_t is Cell_Sectors (
                                                Integer range 0 .. This.CELL_SECTOR_TABLES_LAST,
                                                Integer range 0 .. This.WINDOW_DIAMETER_LAST,
                                                Integer range 0 .. This.WINDOW_DIAMETER_LAST
                                               );
         subtype Cell_Mag_t is Cell_Array (Integer range 0 .. This.WINDOW_DIAMETER_LAST,
                                           Integer range 0 .. This.WINDOW_DIAMETER_LAST);

         procedure Update_Hist (Hist : in out Hist_t;
                                Cell_Mag : Cell_Mag_t;
                                Cell_Sector : Cell_Sector_t) is
         begin
            -- Only have to go through the cells in front.
            for y in Integer range 0 .. Up_To_Half (Cell_Sector'Length (3), Inclusive) loop
               for x in Cell_Sector'Range (2) loop
                  declare
                     Cell_Max_x_y : constant Float := Cell_Mag (x, y);
                  begin
                     for i in Integer range
                       First_Index (Cell_Sector (speed_idx, x, y)) ..
                       Last_Index (Cell_Sector (speed_idx, x, y))
                     loop
                        declare
                           idx : constant Integer := Element (Cell_Sector (speed_idx, x, y), i);
                        begin
                           Hist (idx) := Hist (idx) + Cell_Max_x_y;
                        end;
                     end loop;
                  end;
               end loop;
            end loop;
         end;
      begin
         Update_Hist (Hist        => This.Hist,
                      Cell_Mag    => This.Cell_Mag,
                      Cell_Sector => This.Cell_Sector);
      end;

      Ret := true;
   end Build_Primary_Polar_Histogram;

   ----------------------------------
   -- Build_Binary_Polar_Histogram --
   ----------------------------------

   procedure Build_Binary_Polar_Histogram
     (This : in out VFH;
      speed : Integer)
   is
   begin pragma Assert (VFH_Predicate (This));
      for x in This.Hist'Range loop
         if This.Hist (x) > Get_Binary_Hist_High (This, speed) then
            This.Hist (x) := 1.0;
         elsif This.Hist (x) < Get_Binary_Hist_Low (This, speed) then
            This.Hist (x) := 0.0;
         else
            This.Hist (x) := This.Last_Binary_Hist (x);
         end if;
      end loop;

      This.Last_Binary_Hist := This.Hist;
   end Build_Binary_Polar_Histogram;

   ----------------------------------
   -- Build_Masked_Polar_Histogram --
   ----------------------------------

   procedure Build_Masked_Polar_Histogram
     (This : in out VFH;
      speed : Speed_Index)
   is
      -- center_x_[left|right] is the centre of the circles on either side that
      -- are blocked due to the robot's dynamics.  Units are in cells, in the robot's
      -- local coordinate system (+y is forward).
      center_x_right : constant Float := Float (This.CENTER_X) + (Float (Element (This.Min_Turning_Radius, speed)) / This.CELL_WIDTH);
      center_x_left  : constant Float := Float (This.CENTER_X) - (Float (Element (This.Min_Turning_Radius, speed)) / This.CELL_WIDTH);
      center_y       : constant Float := Float (This.CENTER_Y);

      dist_r, dist_l : Float;

      angle_ahead : constant := 90.0;
      phi_left  : Float := 180.0;
      phi_right : Float := 0.0;

      angle : Float;
   begin
      pragma Assert (VFH_Predicate (This));
      This.Blocked_Circle_Radius := Float (Element (This.Min_Turning_Radius, speed)) + This.ROBOT_RADIUS + Float (Get_Safety_Dist (This, speed));
      --
      -- This loop fixes phi_left and phi_right so that they go through the inside-most
      -- occupied cells inside the left/right circles.  These circles are centred at the
      -- left/right centres of rotation, and are of radius Blocked_Circle_Radius.
      --
      -- We have to go between phi_left and phi_right, due to our minimum turning radius.
      --

      --
      -- Only loop through the cells in front of us.
      -- FIXME: range 0 .. (This.WINDOW_DIAMETER+1)/2-1
      for y in Integer range 0 .. Up_To_Half (This.Cell_Direction'Length (2), Exclusive) loop
         for x in This.Cell_Direction'Range (1) loop
            if This.Cell_Mag (x, y) = 0.0 then
               null;
            elsif Delta_Angle (This.Cell_Direction (x, y), angle_ahead) > 0.0 and then
              Delta_Angle (This.Cell_Direction (x, y), phi_right) <= 0.0
            then
               -- The cell is between phi_right and angle_ahead

               dist_r := Hypot (center_x_right - Float (x), center_y - Float (y)) * This.CELL_WIDTH;
               if dist_r < This.Blocked_Circle_Radius then
                  phi_right := This.Cell_Direction (x, y);
               end if;

            elsif Delta_Angle (This.Cell_Direction (x, y), angle_ahead) <= 0.0 and then
              Delta_Angle (This.Cell_Direction (x, y), phi_left) > 0.0
            then
               -- The cell is between phi_left and angle_ahead

               dist_l := Hypot (center_x_left - Float (x), center_y - Float (y)) * This.CELL_WIDTH;
               if dist_l < This.Blocked_Circle_Radius then
                  phi_left := This.Cell_Direction (x, y);
               end if;
            end if;
         end loop;
      end loop;

      --
      -- Mask out everything outside phi_left and phi_right
      --
      for x in This.Hist'Range loop
         angle := Float (x * This.SECTOR_ANGLE);
         if This.Hist (x) = 0.0 and then (
                                          (Delta_Angle (angle, phi_right) <= 0.0 and then
                                           Delta_Angle (angle, angle_ahead) >= 0.0)
                                          or else
                                            (Delta_Angle (angle, phi_left) >= 0.0 and then
                                             Delta_Angle (angle, angle_ahead) <= 0.0)
                                         )
         then
            null; -- This.Hist(x) := 0.0;
         else
            This.Hist (x) := 1.0;
         end if;
      end loop;
   end Build_Masked_Polar_Histogram;

   ----------------------
   -- Select_Direction --
   ----------------------

   procedure Select_Direction (This : in out VFH) is
      subtype Start_Range is Integer range -1 .. This.HIST_SIZE / 2 - 1;
      start : Start_Range;
      left : Boolean;
      angle, new_angle : Float;

      use Border_Pair_Vector;

      border : Border_Pair_Vector.Vector (This.HIST_COUNT + 1);
      new_border : Border_Pair;
   begin
      pragma Assert (VFH_Predicate (This));

      --
      -- set start to sector of first obstacle
      --
      start := -1;

      -- only look at the forward 180deg for first obstacle.
      for i in Integer range 0 .. This.HIST_SIZE / 2 - 1 loop
         if This.Hist (i) = 1.0 then
            start := i;
            exit;
         end if;
      end loop;

      if start = -1 then
         -- No obstacles detected in front of us: full speed towards goal
         This.Picked_Angle := This.Desired_Angle;
         This.Last_Picked_Angle := This.Picked_Angle;
         This.Max_Speed_For_Picked_Angle := This.Current_Max_Speed;

         return;
      end if;

      --
      -- Find the left and right borders of each opening
      pragma Assert (Is_Empty (border)); pragma Assert (Capacity (border) = This.HIST_COUNT + 1);
      pragma Assert_And_Cut (VFH_Predicate (This) and then Is_Empty (border) and then start >= 0 and then Capacity (border) = This.HIST_COUNT + 1);
      --Put("Start: ");
      --Put_Line(Integer'Image(start));
      left := True;
      for i in Integer range start .. start + This.HIST_SIZE loop
         pragma Loop_Invariant (Length (border) <= Ada.Containers.Count_Type (i - start) and then Capacity (border) = This.HIST_COUNT + 1);
         if This.Hist (i mod This.HIST_SIZE) = 0.0 and then left then
            new_border.first := (i mod This.HIST_SIZE) * This.SECTOR_ANGLE;
            left := False;
         end if;

         if This.Hist (i mod This.HIST_SIZE) = 1.0 and then not left then
            new_border.second := ((i mod This.HIST_SIZE) - 1) * This.SECTOR_ANGLE;
            if new_border.second < 0 then
               new_border.second := new_border.second + 360;
            end if;
            Append (border, new_border);
            left := True;
         end if;
      end loop;
      pragma Assert_And_Cut (VFH_Predicate (This));
      --
      -- Consider each opening
      --
      declare
         CANDIDATE_CAPACITY : constant Ada.Containers.Count_Type := Length (border) * 4;

         Candidates : Candidate_Vector.Vector (CANDIDATE_CAPACITY);
         use Candidate_Vector;

         ----------------------------
         -- Select_Candidate_Angle --
         ----------------------------

         procedure Select_Candidate_Angle is
         begin
            if Is_Empty (Candidates) then
               -- We're hemmed in by obstacles -- nowhere to go,
               -- so brake hard and turn on the spot.
               This.Picked_Angle := This.Last_Picked_Angle;
               This.Max_Speed_For_Picked_Angle := 0;
               return;
            end if;

            This.Picked_Angle := 90.0;
            declare
               min_weight : Float := Float'Last; -- 10000000;
            begin
               pragma Assert (VFH_Predicate (This));
               for i in Integer range First_Index (Candidates) .. Last_Index (Candidates) loop
                  --printf("CANDIDATE: %f\n", Candidate_Angle[i]);
                  pragma Loop_Invariant (Candidates = Candidates'Loop_Entry);
                  declare
                     weight : constant Float :=
                       This.U1 * abs (Delta_Angle (This.Desired_Angle, Element (Candidates, i).Angle)) +
                       This.U2 * abs (Delta_Angle (This.Last_Picked_Angle, Element (Candidates, i).Angle));
                  begin
                     if weight < min_weight then
                        min_weight := weight;
                        This.Picked_Angle := Element (Candidates, i).Angle;
                        This.Max_Speed_For_Picked_Angle := Element (Candidates, i).Speed;
                     end if;
                  end;
               end loop;
            end;

            This.Last_Picked_Angle := This.Picked_Angle;
         end Select_Candidate_Angle;
      begin
         pragma Assert (Is_Empty (Candidates));
         pragma Assert (Capacity (Candidates) = CANDIDATE_CAPACITY);
         for i in First_Index (border) .. Last_Index (border) loop
            pragma Loop_Invariant (Capacity (Candidates) = CANDIDATE_CAPACITY and then Length (Candidates) <= Ada.Containers.Count_Type (i) * 4);
            --printf("BORDER: %f %f\n", border[i].first, border[i].second);
            angle := Delta_Angle (Element (border, i).first, Element (border, i).second);

            if abs (angle) < 10.0 then
               -- ignore very narrow openings
               null;
            elsif abs (angle) < 80.0 then
               -- narrow opening: aim for the centre

               new_angle := Float (Element (border, i).first) + Float (Element (border, i).second - Element (border, i).first) / 2.0;

               Append (Candidates, (Angle => new_angle, Speed => Integer'Min (This.Current_Max_Speed, This.MAX_SPEED_NARROW_OPENING)));
            else
               -- wide opening: consider the centre, and 40deg from each border

               new_angle := Float (Element (border, i).first) + Float (Element (border, i).second - Element (border, i).first) / 2.0;

               Append (Candidates, (Angle => new_angle, Speed => This.Current_Max_Speed));

               new_angle := Float ((Element (border, i).first + 40) mod 360);
               Append (Candidates, (Angle => new_angle, Speed => Integer'Min (This.Current_Max_Speed, This.MAX_SPEED_WIDE_OPENING)));

               new_angle := Float (Element (border, i).second - 40);
               if new_angle < 0.0 then
                  new_angle := new_angle + 360.0;
               end if;
               Append (Candidates, (Angle => new_angle, Speed => Integer'Min (This.Current_Max_Speed, This.MAX_SPEED_WIDE_OPENING)));

               -- See if candidate dir is in this opening
               if Delta_Angle (This.Desired_Angle, Element (Candidates, Last_Index (Candidates) - 1).Angle) < 0.0 and then
                 Delta_Angle (This.Desired_Angle, Last_Element (Candidates).Angle) > 0.0
               then
                  Append (Candidates, (Angle => This.Desired_Angle, Speed => Integer'Min (This.Current_Max_Speed, This.MAX_SPEED_WIDE_OPENING)));
               end if;
            end if;
         end loop;

         Select_Candidate_Angle;
      end;
   end Select_Direction;

   ----------------
   -- Set_Motion --
   ----------------

   procedure Set_Motion
     (This : VFH;
      speed : in out Integer;
      turnrate : out Integer;
      actual_speed : Integer)
   is
   begin
      -- This happens if all directions blocked, so just spin in place
      if speed <= 0 then
         -- Put_Line("stop");
         turnrate := GetMaxTurnrate (This, actual_speed);
         speed := 0;
      else
         --printf("Picked %f\n", Picked_Angle);
         if This.Picked_Angle > 270.0 and then This.Picked_Angle < 360.0 then
            turnrate := -GetMaxTurnrate (This, actual_speed);
         elsif This.Picked_Angle < 270.0 and then This.Picked_Angle > 180.0 then
            turnrate := GetMaxTurnrate (This, actual_speed);
         else
            --turnrate := (int)rint(((float)(Picked_Angle - 90) / 75.0) * GetMaxTurnrate( actual_speed ));
            turnrate := rint (((This.Picked_Angle - 90.0) / 75.0) * Float (GetMaxTurnrate (This, actual_speed)));

            if turnrate > GetMaxTurnrate (This, actual_speed) then
               turnrate := GetMaxTurnrate (This, actual_speed);
            elsif turnrate < -GetMaxTurnrate (This, actual_speed) then
               turnrate := -GetMaxTurnrate (This, actual_speed);
            end if;

            --      if abs(turnrate) > (0.9 * GetMaxTurnrate( actual_speed )) then
            --        speed := 0;
            --      end if;
         end if;
      end if;

      --  speed and turnrate have been set for the calling function -- return.

      return;
   end Set_Motion;

   ---------------------
   -- Print_Cells_Dir --
   ---------------------

   procedure Print_Cells_Dir (This : VFH) is
   begin
      New_Line;
      Put_Line ("Cell Directions:");
      Put_Line ("****************");
      for y in This.Cell_Direction'Range (2) loop
         for x in This.Cell_Direction'Range (1) loop
            --Put(Float'Image(This.Cell_Direction(x,y)));
            Ada.Float_Text_IO.Put (Item => This.Cell_Direction (x, y),
                                   Fore => 3,
                                   Aft  => 1,
                                   Exp  => 0);
            Tab;
         end loop;
         New_Line;
      end loop;
   end Print_Cells_Dir;

   ---------------------
   -- Print_Cells_Mag --
   ---------------------

   procedure Print_Cells_Mag (This : VFH) is
   begin
      New_Line;
      Put_Line ("Cell Magnitudes:");
      Put_Line ("****************");
      for y in This.Cell_Mag'Range (2) loop
         for x in This.Cell_Mag'Range (1) loop
            Put (Float'Image (This.Cell_Mag (x, y)));
            Tab;
         end loop;
         New_Line;
      end loop;
   end Print_Cells_Mag;

   ----------------------
   -- Print_Cells_Dist --
   ----------------------

   procedure Print_Cells_Dist (This : VFH) is
   begin
      New_Line;
      Put_Line ("Cell Distances:");
      Put_Line ("****************");
      for y in This.Cell_Dist'Range (2) loop
         for x in This.Cell_Dist'Range (1) loop
            Put (Float'Image (This.Cell_Dist (x, y)));
            Tab;
         end loop;
         New_Line;
      end loop;
   end Print_Cells_Dist;

   ------------------------
   -- Print_Cells_Sector --
   ------------------------

   procedure Print_Cells_Sector (This : VFH) is
   begin
      New_Line;
      Put_Line ("Cell Sectors for table 0:");
      Put_Line ("***************************");

      --        for y in This.Cell_Sector'Range(3) loop
      --           for x in This.Cell_Sector'Range(2) loop
      --              for i in Integer range 0 .. 1 loop -- i<Cell_Sector[0][x][y].size();i++
      --                 if i < This.Cell_Sector[0][x][y].size() - 1 then
      --                    Put(Integer'Image(Cell_Sector(0,x,y,i)));
      --                    Put(",");
      --                 else
      --                    Put(Integer'Image(Cell_Sector(0,x,y,i)));
      --                    Tab;
      --                 end if;
      --              end loop;
      --           end loop;
      --           New_Line;
      --        end loop;
   end Print_Cells_Sector;

   -----------------------------------
   -- Print_Cells_Enlargement_Angle --
   -----------------------------------

   procedure Print_Cells_Enlargement_Angle (This : VFH) is
   begin
      New_Line;
      Put_Line ("Enlargement Angles:");
      Put_Line ("****************");
      for y in This.Cell_Enlarge'Range (2) loop
         for x in This.Cell_Enlarge'Range (1) loop
            Put (Float'Image (This.Cell_Enlarge (x, y)));
            Tab;
         end loop;
         New_Line;
      end loop;
   end Print_Cells_Enlargement_Angle;

   ----------------
   -- Print_Hist --
   ----------------

   procedure Print_Hist (This : VFH) is
   begin
      Put_Line ("Histogram:");
      Put_Line ("****************");

      pragma Assert (VFH_Predicate (This));
      for x in 0 .. This.HIST_SIZE / 2 loop
         --printf("%d:\t%1.1f\n", (x * SECTOR_ANGLE), Hist[x]);
         Put (Integer'Image (x * This.SECTOR_ANGLE));
         Tab;
         Put (Float'Image (This.Hist (x)));
         New_Line;
      end loop;
      New_Line;
      New_Line;
   end Print_Hist;

   ---------------------
   -- Get_Speed_Index --
   ---------------------

   function Get_Speed_Index (This : VFH; speed : Natural) return Natural is
      val : Natural := Integer (Float'Floor (
                                (Float (speed) / Float (This.Current_Max_Speed))*Float (This.Cell_Sector'Length (1))));
   begin
      pragma Assert (VFH_Predicate (This));
      if val > This.Cell_Sector'Last (1) then
         val := This.Cell_Sector'Last (1);
      end if;

      -- printf("Speed_Index at %dmm/s: %d\n",speed,val);

      return val;
   end Get_Speed_Index;

   ---------------------
   -- Get_Safety_Dist --
   ---------------------

   -- Doesn't need optimization: only gets called on init plus once per update.

   function Get_Safety_Dist (This : VFH; speed : Integer) return Integer is
      val : Integer := Integer (
                                This.SAFETY_DIST_0MS + Float (speed)*(This.SAFETY_DIST_1MS - This.SAFETY_DIST_0MS)/1000.0);
   begin

      if val < 0 then
         val := 0;
      end if;

      -- printf("Safety_Dist at %dmm/s: %d\n",speed,val);

      return val;
   end Get_Safety_Dist;

   -------------------------
   -- Get_Binary_Hist_Low --
   -------------------------

   -- AB: Could optimize this with a look-up table, but it shouldn't make much
   -- difference: only gets called once per sector per update.

   function Get_Binary_Hist_Low (This : VFH; speed : Integer) return Float is
   begin
      return This.BINARY_HIST_LOW_0MS - Float (speed)*(This.BINARY_HIST_LOW_0MS - This.BINARY_HIST_LOW_1MS)/1000.0;
   end Get_Binary_Hist_Low;

   --------------------------
   -- Get_Binary_Hist_High --
   --------------------------

   -- AB: Could optimize this with a look-up table, but it shouldn't make much
   -- difference: only gets called once per sector per update.

   function Get_Binary_Hist_High
     (This : VFH;
      speed : Integer)
      return Float
   is
   begin
      return This.BINARY_HIST_HIGH_0MS - Float (speed)*(This.BINARY_HIST_HIGH_0MS - This.BINARY_HIST_HIGH_1MS)/1000.0;
   end Get_Binary_Hist_High;

end Algorithm;
