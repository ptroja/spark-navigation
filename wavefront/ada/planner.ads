with Ada.Containers.Vectors;
with Ada.Containers.Unbounded_Priority_Queues;
with Ada.Containers.Synchronized_Queue_Interfaces;

package Planner is

   type Occupancy is (Free, Unknown, Occ);

   -- FIXME: discriminated record.
   type Cell_Index is
      record
         i, j : Integer;
      end record;

   type Cell is
      record
         -- Cell index in grid map
         C : Cell_Index;

         -- Occupancy state (-1 = free, 0 = unknown, +1 = occ)
         Occ_State : Occupancy;
         Occ_State_Dyn : Occupancy;

         -- Distance to the nearest occupied cell
         Occ_Dist : Float;
         Occ_Dist_Dyn : Float;

         -- Distance (cost) to the goal
         Plan_Cost : Float;

         -- Mark used in dynamic programming
         Mark : Boolean;

         -- Mark used in path hysteresis
         Lpathmark : Boolean;

         -- The next cell in the plan
         plan_next : Cell_Index;

      end record;

   function Cell_Eq (Left, Right : Cell) return Boolean is (False);

   type Fixed_Dist_Kernel is array (-1 .. +1, -1 .. +1) of Float;
   type Variable_Dist_Kernel is array (Integer range <>, Integer range <>) of Float;

   type Cell_Grid is array (Positive range <>, Positive range <>) of Cell;

   package Plan_Paths is new Ada.Containers.Vectors(Index_Type   => Natural,
                                               Element_Type => Cell,
                                               "="          => Cell_Eq);

   subtype Plan_Path is Plan_Paths.Vector;

   function Get_Priority(Element : Cell) return Float
   is (Element.Plan_Cost);

   function Before (Left, Right : Float) return Boolean renames "<";

   package Cell_Queues is
     new Ada.Containers.Synchronized_Queue_Interfaces(Element_Type => Cell);
   package Cell_Priority_Queues is
     new Ada.Containers.Unbounded_Priority_Queues(Queue_Interfaces => Cell_Queues,
                                                  Queue_Priority   => Float);

   -- Grid dimensions (number of cells)
   type Plan (Last_X, Last_Y : Positive;
              Number_Of_Cells : Ada.Containers.Count_Type;
              kernel_index_min, kernel_index_max : Integer) is
      record
         -- Grid origin (real-world coords, in meters, of the lower-left grid cell)
         Origin_X, Origin_Y : Float;

         -- Grid scale (m/cell)
         Scale : Float;

         -- Max radius we will consider
         Max_Radius : Float;

         -- The grid data
         cells : Cell_Grid (0 .. Last_X, 0 .. Last_Y);

         -- The global path
         path : Plan_Path;

         -- The local path (mainly for debugging)
         lpath : Plan_Path;

         -- Waypoints extracted from global path
         waypoints : Plan_Path;

         -- Priority queue of cells to update
         heap : Cell_Priority_Queues.Queue;

         -- Distance penalty kernel, pre-computed in plan_compute_dist_kernel();
         dist_kernel : Variable_Dist_Kernel(kernel_index_min .. kernel_index_max,
                                            kernel_index_min .. kernel_index_max);

         Dist_Kernel_3x3 : Fixed_Dist_Kernel;

         -- Penalty factor for cells inside the max radius
         Dist_Penalty : Float;

         -- Cost multiplier for cells on the previous local path
         Hysteresis_Factor : Float;

         -- Grid bounds (for limiting the search).
         Min_X, Min_Y, Max_X, Max_Y : Integer;

         -- Effective robot radius
         Abs_Min_Radius : Float;
      end record;

   type Plan_Ptr is access Plan;

   function Create(size_x, size_y : Positive;
                   abs_min_radius,
                   max_radius, dist_penalty,
                   hysteresis_factor : Float)
                   return Plan_Ptr;

   -- Initialize the plan
   procedure Init(This : in out Plan);

   -- Reset the plan
   procedure Reset(This : in out Plan);

   -- Load the occupancy values from an image file
   --int load_occ(const char *filename, double scale);

   procedure set_bounds(This : in out Plan;
                        min_x, min_y, max_x, max_y : Integer);

   procedure set_bbox(This : in out Plan;
                      padding : Float;
                      min_size : Float;
                      x0, y0, x1, y1 : Float);

   function check_inbounds(This : in Plan; x, y : Float) return Boolean;

   -- Construct the configuration space from the occupancy grid.
   --void plan_update_cspace(plan_t *plan, const char* cachefile);
   procedure compute_cspace(This : in out Plan);

   type Status is (Success, Error);

   function do_global(This : in out Plan;
                      lx, ly : Float;
                      gx, gy : Float) return Status;

   function do_local(This : in out Plan;
                     lx, ly : Float;
                     plan_halfwidth : Float) return Status;

   -- Generate a path to the goal
   procedure update_waypoints(This : in out Plan;
                              px, py : Float);

   -- Convert given waypoint cell to global x,y
   procedure convert_waypoint(This : Plan;
                              Waypoint : Cell;
                              px, py : out Float);

   function get_carrot(px, py : out Float;
                       lx, ly : Float;
                       maxdist : Float;
                       distweight : Float) return Float;

   function compute_diffdrive_cmds(vx, va : out Float;
                                   rotate_dir : Integer;
                                   lx, ly, la : Float;
                                   gx, gy, ga : Float;
                                   goal_d, goal_a : Float;
                                   maxd : Float;
                                   dweight : Float;
                                   txmin, tvmax : Float;
                                   avmin, avmax : Float;
                                   amin, amax : Float) return Integer;

   --procedure set_obstacles(); --double* obs, size_t num);

private

   function GXWX(This : Plan; X : Float) return Integer;
   function GYWY(This : Plan; Y : Float) return Integer;

   function WXGX(This : Plan; i : Integer) return Float;
   function WYGY(This : Plan; j : Integer) return Float;

   type Dist_Kernel is array (Integer range <>, Integer range <>) of Float;

   -- Compute variable sized kernel, for use in propagating distance from obstacles
   function Get_Variable_Dist_Kernel(This : Plan) return Dist_Kernel;

   procedure Compute_Dist_Kernel(This : in out Plan);

   function VALID_BOUNDS(This : Plan; I, J : Integer) return Boolean;

end Planner;
