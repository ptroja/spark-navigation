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

      procedure Push(Queue : in out Cell_Priority_Queue.Queue_Type; Element : Cell_Index);

      procedure Pop(Queue : in out Cell_Priority_Queue.Queue_Type; Element : out Cell_Ptr);

      procedure Push(Queue : in out Cell_Priority_Queue.Queue_Type; Element : Cell_Index) is
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
      push(heap, (i => gi, j => gj));

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
                           cost := cost + This.Dist_Kernel_3x3(di,dj) *
                             (if This.cells(ni,nj).Lpathmark
                              then This.hysteresis_factor
                              else 1.0);

                           if This.cells(ni,nj).Occ_Dist_Dyn < This.max_radius then
                              cost := cost + This.dist_penalty * (This.max_radius - This.cells(ni,nj).Occ_Dist_Dyn);
                           end if;

                           if cost < This.cells(ni,nj).Plan_Cost then
                              This.cells(ni,nj).Plan_Cost := cost;
                              This.cells(ni,nj).plan_next := cell;

                              This.cells(ni,nj).Mark := True;
                              push(heap, (i => ni, j => nj));
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

      -- puts("start was found");
      Result := (if This.cells(li,lj).plan_next.Opt = O_SOME
                 then Success
                 else Failure);
   end;

   procedure find_local_goal(This : in out Plan;
                             gx, gy : out Float;
                             lx, ly : Float;
                             Result : out Status)
   is
      use Plan_Paths;
      use type Cell_Count;

      c_min : Cell_Count;
   begin
      -- FIXME: Result should be a discriminated record.
      gx := 0.0;
      gy := 0.0;

      -- Must already have computed a global goal
      if Plan_Paths.Is_Empty(This.path) then
         -- Put_Line ("no global path");

         Result := Failure;
         return;
      end if;

      declare
         li : constant Integer := GXWX(This, lx);
         lj : constant Integer := GYWY(This, ly);
      begin
         if not VALID_BOUNDS(This, li, lj) then
            Result := Failure;
            return;
         end if;

         -- Find the closest place to jump on the global path
         declare
            -- FIXME: this initialization is not needed.
            squared_d_min : Integer := Integer'Last;
            Found : Boolean := False;
         begin

            for i in Cell_Count range First_Index(This.path) .. Last_Index(This.path) loop
               declare
                  idx : constant Cell_Index := Element(This.path, i);
                  squared_d : constant Integer :=
                    (idx.i - li) * (idx.i - li) +
                    (idx.j - lj) * (idx.j - lj);
               begin
                  if (not Found) or else squared_d < squared_d_min then
                     squared_d_min := squared_d;
                     c_min := i;
                     Found := True;
                  end if;
               end;
            end loop;

            pragma Assert (Found);
         end;
      end;

      -- Follow the path to find the last cell that's inside the local planning
      -- area
      declare
         last_inside : Cell_Count;
      begin
         for i in Cell_Count range c_min .. Last_Index(This.path) loop
            declare
               cell : constant Cell_Index := Element(This.path, i);
            begin
               -- printf("step %d: (%d,%d)\n", c, cell->ci, cell->cj);

               if not (cell.i in This.min_x .. This.max_x and then
                       cell.j in This.min_y .. This.max_y) then
                  -- Did we move at least one cell along the path?
                  if i = c_min then
                     -- nope; the entire global path is outside the local region;
                     -- can't fix that here
                     -- Put_Line("global path not in local region");
                     Result := Failure;
                     return;
                  else
                     last_inside := i-1;
                     exit;
                  end if;
               end if;
            end;
         end loop;

         declare
            idx : Cell_Index := Element(This.path, last_inside);
         begin
            -- printf("ci: %d cj: %d\n", cell->ci, cell->cj);
            gx := WXGX(This, idx.i);
            gy := WYGY(This, idx.j);
         end;

         Result := Success;
         return;
      end;
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

      Update_Plan(This, lx, ly, gx, gy, Result);
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
            -- Put_Line ("no local goal");
            return;
         end if;

         -- printf("local goal: %.3lf, %.3lf\n", gx, gy);

         Plan_Paths.Clear(This.lpath);

         Update_Plan(This, lx, ly, gx, gy, Result);
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
   function reachable(This : Plan; A, B : Cell_Index) return Boolean is
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
               if dist > 0.50 and then
                 not reachable(This, cell.C, This.cells(ncell.C.i,ncell.C.j).plan_next.C) then
                  exit;
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
