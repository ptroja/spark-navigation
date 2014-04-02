with Ada.Numerics.Elementary_Functions;
use Ada.Numerics.Elementary_Functions;

with Priority_Queue;

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
      --printf("scale: %.3lf\n", scale);

      for j in This.cells'Range(2) loop
         for i in This.cells'Range(1) loop
            This.cells(i,j).C := (i => i, j => j);
            This.cells(i,j).occ_state_dyn := This.cells(i,j).occ_state;
            This.cells(i,j).occ_dist_dyn := This.cells(i,j).occ_dist;
            This.cells(i,j).plan_cost := MAX_COST;
            This.cells(i,j).plan_next := (Opt => O_NONE);
            This.cells(i,j).lpathmark := false;
         end loop;
      end loop;

      Plan_Paths.Clear(This.waypoints);

      compute_dist_kernel(This);

      set_bounds(This, 0, 0, This.Last_X, This.Last_Y);
   end Init;

   -----------
   -- Reset --
   -----------

   procedure Reset (This : in out Plan) is
   begin
      for j in Integer range This.min_y .. This.max_y loop
         for i in Integer range This.min_x .. This.max_x loop
            This.cells(i,j).plan_cost := MAX_COST;
            This.cells(i,j).plan_next := (Opt => O_NONE);
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

   procedure update_plan(This : in out Plan;
                         lx, ly, gx, gy : Float;
                         Result : out Status)
   is
      -- Initialize the goal cell
      gi : constant Integer := GXWX(This, gx);
      gj : constant Integer := GYWY(This, gy);

      -- Initialize the start cell
      -- FIXME: this may be not needed, can be done after the first 'return'.
      li : constant Integer := GXWX(This, lx);
      lj : constant Integer := GYWY(This, ly);

      old_occ_state : Occupancy;
      old_occ_dist : Float;

      function Priority_Of(Element : Cell_Index) return Float is (This.cells(Element.i,Element.j).Plan_Cost);

      package Cell_Priority_Queue is new Priority_Queue(Element_Type  => Cell_Index,
                                                        Priority_Type => Float,
                                                        Priority_Of => Priority_Of,
                                                        ">" => ">");

      heap : Cell_Priority_Queue.Queue_Type(100);

      procedure Push(This : Plan; Queue : in out Cell_Priority_Queue.Queue_Type; Element : Cell_Index);

      procedure Pop(Queue : in out Cell_Priority_Queue.Queue_Type; Element : out Cell_Ptr);

      procedure Push(This : Plan; Queue : in out Cell_Priority_Queue.Queue_Type; Element : Cell_Index) is
      begin
         Cell_Priority_Queue.Enqueue(Queue, Element);
      end;

      procedure Pop(Queue : in out Cell_Priority_Queue.Queue_Type;  Element : out Cell_Ptr) is
      begin
         if Cell_Priority_Queue.Empty(Queue) then
            Element := (Opt => O_NONE);
         else
            declare
               Top : Cell_Index;
            begin
               Cell_Priority_Queue.Dequeue(Queue, Top);
               Element := (Opt => O_SOME, C => Top);
            end;
         end if;
      end;

   begin
      -- Reset the queue
      -- TODO: use C++11 swap with empty heap.
      -- while(!heap.empty()) heap.pop();

      --printf("planning from %d,%d to %d,%d\n", li,lj,gi,gj);

      if not VALID_BOUNDS(This, gi, gj) then
         --puts("goal out of bounds");
         Result := Failure;
         return;
      end if;

      if not VALID_BOUNDS(This, li, lj) then
         --puts("start out of bounds");
         Result := Failure;
         return;
      end if;

      -- Latch and clear the obstacle state for the cell I'm in
      old_occ_state := This.cells(li,lj).Occ_State_Dyn;
      old_occ_dist := This.cells(li,lj).Occ_Dist_Dyn;

      This.cells(li,lj).Occ_State_Dyn := Free;
      This.cells(li,lj).Occ_Dist_Dyn := This.Max_Radius;

      This.cells(gi,gj).Plan_Cost := 0.0;

      -- Are we done?
      if li = gi and then lj = gj then
         result := Success;
         return;
      end if;

      This.cells(gi,gj).Mark := True;
      push(This, heap, (i => gi, j => gj));

      loop
         declare
            cell : Cell_Ptr;
         begin

            Pop(heap, cell);

            if cell.Opt = O_NONE then
               exit;
            end if;

            --printf("pop %d %d %f\n", cell->ci, cell->cj, cell->plan_cost);

            for dj in Integer range -1 .. +1 loop
               for di in Integer range -1 .. +1 loop

                  declare
                     ni : constant Integer := cell.C.i + di;
                     nj : constant Integer := cell.C.j + dj;
                  begin
                     if (di = 0 and then dj = 0) or else
                       (not VALID_BOUNDS(This, ni, nj)) or else
                       This.cells(ni,nj).Mark or else
                       This.cells(ni,nj).Occ_Dist_Dyn < This.Abs_Min_Radius then
                        null;
                     else
                        declare
                           cost : Float := This.cells(cell.C.i,cell.C.j).Plan_Cost;
                        begin
                           if This.cells(ni,nj).Lpathmark then
                              cost := cost + This.Dist_Kernel_3x3(di,dj) * This.hysteresis_factor;
                           else
                              cost := cost + This.Dist_Kernel_3x3(di,dj);
                           end if;

                           if This.cells(ni,nj).Occ_Dist_Dyn < This.max_radius then
                              cost := cost + This.dist_penalty * (This.max_radius - This.cells(ni,nj).Occ_Dist_Dyn);
                           end if;

                           if cost < This.cells(ni,nj).Plan_Cost then
                              This.cells(ni,nj).plan_cost := cost;
                              This.cells(ni,nj).plan_next := cell;

                              This.cells(ni,nj).Mark := True;
                              push(This, heap, (i => ni, j => nj));
                           end if;
                        end;
                     end if;
                  end;
               end loop;
            end loop;
         end;
      end loop;

      -- Restore the obstacle state for the cell I'm in
      This.cells(li,lj).occ_state_dyn := old_occ_state;
      This.cells(li,lj).occ_dist_dyn := old_occ_dist;

      --puts("start was found");
      Result := (if This.cells(li,lj).plan_next.Opt = O_SOME
                 then Success
                 else Failure);
   end;

   procedure find_local_goal(This : in out Plan;
                             gx, gy : out Float;
                             lx, ly : Float;
                             Result : out Status)
   is
   begin
      --  Generated stub: replace with real body!
      pragma Compile_Time_Warning (Standard.True, "find_local_goal unimplemented");
      raise Program_Error;
      find_local_goal (This, gx, gy, lx, ly, Result);
   end;

   ---------------
   -- do_global --
   ---------------

   procedure do_global
     (This : in out Plan;
      lx, ly : Float;
      gx, gy : Float;
      Result : out Status)
   is
   begin
      -- Set bounds to look over the entire grid
      set_bounds(This, 0, 0, This.Last_X, This.Last_Y);

      -- Reset plan costs
      reset(This);

      Plan_Paths.Clear(This.path);

      update_plan(This, lx, ly, gx, gy, Result);
      if Result = Failure then
         return;
      end if;

      declare
         li : constant Integer := GXWX(This, lx);
         lj : constant Integer := GYWY(This, ly);

         cell : Cell_Ptr := (Opt => O_SOME, C => (i => li, j => lj));
      begin

         -- Cache the path
         while cell.Opt = O_SOME loop
            Plan_Paths.Append(This.path, cell.C);
            cell := This.cells(cell.C.i,cell.C.j).plan_next;
         end loop;
      end;

      Result := Success;
   end do_global;

   --------------
   -- do_local --
   --------------

   procedure do_local
     (This : in out Plan;
      lx, ly : Float;
      plan_halfwidth : Float;
      Result : out Status)
   is
   begin
      declare
         -- Set bounds as directed
         xmin : constant Integer := GXWX(This, lx - plan_halfwidth);
         ymin : constant Integer := GYWY(This, ly - plan_halfwidth);
         xmax : constant Integer := GXWX(This, lx + plan_halfwidth);
         ymax : constant Integer := GYWY(This, ly + plan_halfwidth);
      begin
         set_bounds(This, xmin, ymin, xmax, ymax);
      end;

      -- Reset plan costs (within the local patch)
      reset(This);

      declare
         -- Find a local goal to pursue
         gx, gy : Float;
      begin

         find_local_goal(This, gx, gy, lx, ly, Result);
         if Result = Failure then
            -- puts("no local goal");
            return;
         end if;

         -- printf("local goal: %.3lf, %.3lf\n", gx, gy);

         Plan_Paths.Clear(This.lpath);

         update_plan(This, lx, ly, gx, gy, Result);
         if Result = Failure then
            return;
         end if;
      end;

      -- Reset path marks (TODO: find a smarter place to do this)
      for j in This.cells'Range(2) loop
         for i in This.cells'Range(1) loop
            This.cells(i,j).Lpathmark := False;
         end loop;
      end loop;

      declare
         li : constant Integer := GXWX(This, lx);
         lj : constant Integer := GYWY(This, ly);
         cell : Cell_Ptr := (Opt => O_SOME, C => (i => li, j => lj));
      begin

         -- Cache the path
         while cell.Opt = O_SOME loop
            This.cells(cell.C.i,cell.C.j).Lpathmark := true;
            Plan_Paths.Append(This.lpath,cell.C);

            cell := This.cells(cell.C.i,cell.C.j).plan_next;
         end loop;
      end;

      -- printf("computed local path: %.6lf\n", t1-t0);
      Result := Success;
   end do_local;

   function VALID(This : Plan; I, J : Integer) return Boolean is
   begin
      return
        i in 0 .. this.Last_X and then
        j in 0 .. this.Last_Y;
   end;

   -- See if once cell is reachable from another.
   function test_reachable(This : Plan; A, B : Cell_Index) return Boolean is
      theta : constant Float := Arctan(Float(B.j - A.j),
                                       Float(B.i - A.i));

      -- FIXME: use sincos where available
      sinth : constant Float := Sin(Theta);
      costh : constant Float := Cos(Theta);

      i : Float := Float(A.i);
      j : Float := Float(A.j);

      lasti : Integer := A.i;
      lastj : Integer := A.j;
   begin

      while not (lasti = B.i and then lastj = B.j) loop
         if lasti /= Integer(Float'Floor(i)) or else lastj /= Integer(Float'Floor(j)) then
            lasti := Integer(Float'Floor(i));
            lastj := Integer(Float'Floor(j));
            if (not VALID(This,lasti,lastj)) or else -- stepped out of map
              This.cells(lasti,lastj).occ_dist < This.abs_min_radius then
               return false;
            end if;
         end if;

         if lasti /= B.i then
            i := i + costh;
         end if;
         if lastj /= B.j then
            j := j + sinth;
         end if;
      end loop;

      return true;
   end;

   ----------------------
   -- update_waypoints --
   ----------------------

   procedure update_waypoints
     (This : in out Plan;
      px, py : Float)
   is
      ni : constant Integer := GXWX(This, px);
      nj : constant Integer := GYWY(This, py);

      Cell : Cell_Ptr;
   begin

      Plan_Paths.Clear(This.waypoints);

      -- Can't plan a path if we're off the map
      if not VALID(This,ni,nj) then
         return;
      end if;

      cell := (Opt => O_SOME, C => (i => ni, j => nj));

      while Cell.Opt /= O_NONE loop

         Plan_Paths.Append(This.waypoints, cell.C);

         if This.cells(cell.C.i,cell.C.j).plan_next.Opt = O_NONE then
            -- done
            exit;
         end if;

         -- Find the farthest cell in the path that is reachable from the
         -- current cell.
         declare
            dist : Float := 0.0;
            ncell : Cell_Ptr := Cell;
         begin
            while This.cells(ncell.C.i,ncell.C.j).plan_next.Opt /= O_NONE loop
               if dist > 0.50 then
                  if not test_reachable(This, cell.C, This.cells(ncell.C.i,ncell.C.j).plan_next.C) then
                     exit;
                  end if;
               end if;
               dist := dist + This.scale;

               ncell := This.cells(ncell.C.i,ncell.C.j).plan_next;
            end loop;

            if ncell = cell then
               exit;
            end if;

            cell := ncell;
         end;
      end loop;

      if cell.Opt = O_SOME and then This.cells(cell.C.i,cell.C.j).Plan_Cost > 0.0 then
         -- no path
         Plan_Paths.Clear(This.waypoints);
      end if;

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
      pragma Assert (min_x <= max_x);
      pragma Assert (min_y <= max_y);

      This.min_x := Integer'Min(This.Last_X, Integer'Max(0, min_x));
      This.min_y := Integer'Min(This.Last_Y, Integer'Max(0, min_y));
      This.max_x := Integer'Min(This.Last_X, Integer'Max(0, max_x));
      This.max_y := Integer'Min(This.Last_Y, Integer'Max(0, max_y));

      --printf("new bounds: (%d,%d) -> (%d,%d)\n",
      --plan->min_x, plan->min_y,
      --plan->max_x, plan->max_y);
   end;

   function hypot (X, Y : Float) return Float is
   begin
      return Sqrt(X*X + Y*Y);
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
