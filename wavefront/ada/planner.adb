with Ada.Numerics.Elementary_Functions;

package body Planner is

   MAX_COST : constant := 1.0e9;

   ------------
   -- Create --
   ------------

   function Create
     (size_x, size_y : Positive;
      abs_min_radius,
      max_radius, dist_penalty,
      hysteresis_factor : Float)
      return Plan_Ptr
   is
      This : Plan_Ptr := new Plan
        (Last_X => size_x-1, Last_Y => size_y-1,
         Number_Of_Cells => Ada.Containers.Count_Type(size_x*size_y),
         kernel_index_min => -1, kernel_index_max => +1
        );
   begin
      This.abs_min_radius := abs_min_radius;
      This.max_radius := max_radius;
      This.dist_penalty := dist_penalty;
      This.hysteresis_factor := hysteresis_factor;
      This.min_x := 0;
      This.min_y := 0;
      This.max_x := 0;
      This.max_y := 0;
      This.origin_x := 0.0;
      This.origin_y := 0.0;
      This.scale := 0.0;
      This.Dist_Kernel_3x3 := (others => (others => 0.0));

      Plan_Paths.Reserve_Capacity(This.path, 1000);
      Plan_Paths.Reserve_Capacity(This.lpath, 1000);
      Plan_Paths.Reserve_Capacity(This.waypoints, 1000);

      return This;
   end Create;

   ----------
   -- Init --
   ----------

   procedure Init (This : in out Plan) is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "Init unimplemented");
      raise Program_Error;
   end Init;

   -----------
   -- Reset --
   -----------

   procedure Reset (This : in out Plan) is
   begin
      for j in Integer range This.min_y .. This.max_y loop
         for i in Integer range This.min_x .. This.max_x loop
            This.cells(i,j).plan_cost := MAX_COST;
            This.cells(i,j).plan_next := (i => -1, j => -1);
            This.cells(i,j).mark := false;
         end loop;
      end loop;

      Plan_Paths.Clear(This.waypoints);
   end Reset;

   --------------
   -- set_bbox --
   --------------

   procedure set_bbox
     (This : in out Plan;
      padding : Float;
      min_size : Float;
      x0, y0, x1, y1 : Float)
   is
      gx0, gy0, gx1, gy1 : Integer;
      min_x, min_y, max_x, max_y : Integer;
      sx, sy : Integer;
      dx, dy : Integer;
      gmin_size : Integer;
      gpadding : Integer;
   begin

      gx0 := GXWX(this, x0);
      gy0 := GYWY(this, y0);
      gx1 := GXWX(this, x1);
      gy1 := GYWY(this, y1);

      -- Make a bounding box to include both points.
      min_x := Integer'Min(gx0, gx1);
      min_y := Integer'Min(gy0, gy1);
      max_x := Integer'Max(gx0, gx1);
      max_y := Integer'Max(gy0, gy1);

      -- Make sure the min_size is achievable
      gmin_size := Integer(Float'Ceiling(min_size / This.scale));
      gmin_size := Integer'Min(gmin_size, Integer'Min(This.Last_X, This.Last_Y));

      -- Add padding
      gpadding := Integer(Float'Ceiling(padding / This.scale));
      min_x := min_x - gpadding / 2;
      min_x := Integer'Max(min_x, 0);
      max_x := max_x + gpadding / 2;
      max_x := Integer'Min(max_x, This.Last_X);
      min_y := min_y - gpadding / 2;
      min_y := Integer'Max(min_y, 0);
      max_y := max_y + gpadding / 2;
      max_y := Integer'Min(max_y, This.Last_Y);

      -- Grow the box if necessary to achieve the min_size
      sx := max_x - min_x;
      while sx < gmin_size loop
         dx := gmin_size - sx;
         min_x := min_x - Integer(Float'Ceiling(Float(dx) / 2.0)); -- TODO: this can be computed without floating-point
         max_x := max_x + Integer(Float'Ceiling(Float(dx) / 2.0));

         min_x := Integer'Max(min_x, 0);
         max_x := Integer'Min(max_x, This.Last_X);

         sx := max_x - min_x;
      end loop;
      sy := max_y - min_y;
      while sy < gmin_size loop
         dy := gmin_size - sy;
         min_y := min_y - Integer(Float'Ceiling(Float(dy) / 2.0));
         max_y := max_y + Integer(Float'Ceiling(Float(dy) / 2.0));

         min_y := Integer'Max(min_y, 0);
         max_y := Integer'Min(max_y, This.Last_Y);

         sy := max_y - min_y;
      end loop;

      set_bounds(This, min_x, min_y, max_x, max_y);
   end set_bbox;

   --------------------
   -- check_inbounds --
   --------------------

   function check_inbounds (This : in Plan; x, y : Float) return Boolean is
      gx : constant Integer := GXWX(this, x);
      gy : constant Integer := GYWY(this, y);
   begin
      return
        gx in This.min_x .. This.max_x and then
        gy in This.min_y .. This.max_y;
   end check_inbounds;

   --------------------
   -- compute_cspace --
   --------------------

   procedure compute_cspace (This : in out Plan) is
   begin
      -- Ada.Text_IO.Put_Line("Generating C-space...");

      for j in Integer range This.min_y .. This.Max_Y loop
         for i in Integer range This.min_x .. This.max_x loop

            if not (This.cells(i,j).occ_state = Free) then

               for dj in Integer range This.dist_kernel'Range(2) loop
                  for di in Integer range This.dist_kernel'Range(1) loop

                     if VALID_BOUNDS(This, i+di,j+dj) and then
                       This.dist_kernel(di,dj) < This.cells(i+di,j+dj).occ_dist then
                        This.cells(i+di,j+dj).occ_dist_dyn := This.dist_kernel(di,dj);
                        This.cells(i+di,j+dj).occ_dist := This.dist_kernel(di,dj);
                     end if;

                  end loop;
               end loop;
            end if;

         end loop;
      end loop;

   end compute_cspace;

   ---------------
   -- do_global --
   ---------------

   function do_global
     (This : in out Plan;
      lx, ly : Float;
      gx, gy : Float)
      return Status
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "do_global unimplemented");
      raise Program_Error;
      return do_global (This, lx, ly, gx, gy);
   end do_global;

   --------------
   -- do_local --
   --------------

   function do_local
     (This : in out Plan;
      lx, ly : Float;
      plan_halfwidth : Float)
      return Status
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "do_local unimplemented");
      raise Program_Error;
      return do_local (This, lx, ly, plan_halfwidth);
   end do_local;

   ----------------------
   -- update_waypoints --
   ----------------------

   procedure update_waypoints
     (This : in out Plan;
      px, py : Float)
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "update_waypoints unimplemented");
      raise Program_Error;
   end update_waypoints;

   ----------------------
   -- convert_waypoint --
   ----------------------

   procedure convert_waypoint
     (This : Plan;
      Waypoint : Cell;
      px, py : out Float)
   is
   begin
      px := WXGX(This, waypoint.c.i);
      py := WYGY(This, waypoint.c.j);
   end convert_waypoint;

   ----------------
   -- get_carrot --
   ----------------

   function get_carrot
     (px, py : out Float;
      lx, ly : Float;
      maxdist : Float;
      distweight : Float)
      return Float
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "get_carrot unimplemented");
      raise Program_Error;
      return get_carrot (px, py, lx, ly, maxdist, distweight);
   end get_carrot;

   ----------------------------
   -- compute_diffdrive_cmds --
   ----------------------------

   function compute_diffdrive_cmds
     (vx, va : out Float;
      rotate_dir : Integer;
      lx, ly, la : Float;
      gx, gy, ga : Float;
      goal_d, goal_a : Float;
      maxd : Float;
      dweight : Float;
      txmin, tvmax : Float;
      avmin, avmax : Float;
      amin, amax : Float)
      return Integer
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "compute_diffdrive_cmds unimplemented");
      raise Program_Error;
      return compute_diffdrive_cmds (vx, va, rotate_dir, lx, ly, la, gx, gy,
         ga, goal_d, goal_a, maxd, dweight, txmin, tvmax, avmin, avmax, amin,
         amax);
   end compute_diffdrive_cmds;

   function PLAN_GXWX(This : Plan; X : Float) return Integer
   is
   begin
      return Integer((X - This.origin_x) / This.scale + 0.5);
   end;

   function PLAN_GYWY(This : Plan; Y : Float) return Integer
   is
   begin
      return Integer((Y - This.origin_y) / This.scale + 0.5);
   end;

   procedure set_bounds(This : in out Plan;
                        min_x, min_y, max_x, max_y : Integer)
   is
   begin
      This.min_x := Integer'Min(This.Last_X, Integer'Max(0, min_x));
      This.min_y := Integer'Min(This.Last_Y, Integer'Max(0, min_y));
      This.max_x := Integer'Min(This.Last_X, Integer'Max(0, max_x));
      This.max_y := Integer'Min(This.Last_Y, Integer'Max(0, max_y));

      pragma Assert (This.min_x <= This.max_x);
      pragma Assert (This.min_y <= This.max_y);

      --printf("new bounds: (%d,%d) -> (%d,%d)\n",
      --plan->min_x, plan->min_y,
      --plan->max_x, plan->max_y);
   end;

   function hypot (X, Y : Float) return Float is
   begin
      return Ada.Numerics.Elementary_Functions.Sqrt(X*X + Y*Y);
   end;

   function Get_Variable_Dist_Kernel(This : Plan) return Dist_Kernel is
      dist_kernel_width : constant Positive := 1 + 2 * Integer(Float'Ceiling(This.max_radius / This.scale));
      Kernel : Dist_Kernel(-dist_kernel_width/2 .. +dist_kernel_width/2,
                           -dist_kernel_width/2 .. +dist_kernel_width/2);
   begin
      for j in Kernel'Range(1) loop
         for i in Kernel'Range(2) loop
            Kernel(i,j) := hypot(Float(i),Float(j)) * This.scale;
         end loop;
      end loop;

      return Kernel;
   end;

   procedure Compute_Dist_Kernel(This : in out Plan) is
   begin
      -- also compute a 3x3 kernel, used when propagating distance from goal
      for j in Integer range This.Dist_Kernel_3x3'Range(2) loop
         for i in Integer range This.Dist_Kernel_3x3'Range(1) loop
            This.Dist_Kernel_3x3(i,j) := hypot(Float(i),Float(j)) * This.scale;
         end loop;
      end loop;
   end;

   function WXGX(This : Plan; i : Integer) return Float is
   begin
      return This.origin_x + Float(i) * This.Scale;
   end;

   function WYGY(This : Plan; j : Integer) return Float is
   begin
      return This.origin_y + Float(j) * This.Scale;
   end;

   function GXWX(This : Plan; x : Float) return Integer is
   begin
      return Integer((x - this.origin_x) / this.scale + 0.5);
   end;

   function GYWY(This : Plan; Y : Float) return Integer is
   begin
      return Integer((y - this.origin_Y) / this.scale + 0.5);
   end;

   function VALID_BOUNDS(This : Plan; I, J : Integer) return Boolean is
   begin
      return
        i in this.min_x .. this.max_x and then
        j in this.min_y .. this.max_y;
   end;

end Planner;
