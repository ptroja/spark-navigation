with interface_h;
with Interfaces.C;

with Algorithm;

package body Robot_Iface is

   use type interfaces.C.int;

   procedure step_c (arg1 : not null access interface_h.proxy_c);
   pragma Export(CPP, step_c);

   procedure step_c(arg1 : not null access interface_h.proxy_c) is
      C : Algorithm.Controller;
   begin
      if arg1.PeekInputData(arg1.robot_proxy_ptr) = 0 or else
        arg1.isNewGoalData(arg1.robot_proxy_ptr) = 0
      then
         return;
      end if;

      C.robot.robot_radius := Float(arg1.robot_radius);
      C.robot.min_gap_width := Float(arg1.min_gap_width);
      C.robot.obstacle_avoid_dist := Float(arg1.obstacle_avoid_dist);
      C.robot.max_speed := Float(arg1.max_speed);
      C.robot.max_turn_rate := Float(arg1.max_turn_rate);
      C.robot.goal_position_tol := Float(arg1.goal_position_tol);
      C.robot.goal_angle_tol := Float(arg1.goal_angle_tol);
      C.robot.goalX := Float(arg1.goalX);
      C.robot.goalY := Float(arg1.goalY);
      C.robot.goalA := Float(arg1.goalA);

      C.robot.scan_Count := Natural(arg1.getScanCount(arg1.robot_proxy_ptr));
      C.robot.scan_Res := Float(arg1.getScanRes(arg1.robot_proxy_ptr));
      C.robot.max_Range := Float(arg1.getMaxRange(arg1.robot_proxy_ptr));

      pragma Assert(C.robot.scan_Count = Laser_Scans'Length);

      for I in Laser_Scans'Range loop
         C.robot.scans(I) := Float(
                             arg1.getRange(
                               arg1.robot_proxy_ptr,
                               Interfaces.C.unsigned(I-Laser_Scans'First))
                            );
      end loop;
      C.robot.X := Float(arg1.getXPos(arg1.robot_proxy_ptr));
      C.robot.Y := Float(arg1.getYPos(arg1.robot_proxy_ptr));
      C.robot.Yaw := Float(arg1.getYaw(arg1.robot_proxy_ptr));
      C.robot.goal_reached := False;

      --  Create, Step;
      Algorithm.Create(Robot => C.robot);
      Algorithm.Step(This => C);

      if C.robot.speed.Opt = O_SOME then
         arg1.setSpeed(
                       arg1.robot_proxy_ptr,
                       Interfaces.C.double(C.robot.speed.modulus),
                       Interfaces.C.double(C.robot.speed.angle)
                      );
      end if;

      if C.robot.goal_reached then
         arg1.goalReached(arg1.robot_proxy_ptr);
      end if;
   end step_c;

   function GetScanCount (This : Proxy) return Natural is
   begin
      return This.scan_Count;
   end GetScanCount;

   function GetRange
     (This : Proxy;
      index : Laser_Scan_ID)
      return Formal.Numerics.NonNegative_Float
   is
   begin
      return This.scans(index);
   end GetRange;

   function GetXPos (This : Proxy) return Float is
   begin
      return This.X;
   end GetXPos;

   function GetYPos (This : Proxy) return Float is
   begin
      return This.Y;
   end GetYPos;

   function GetYaw (This : Proxy) return Float is
   begin
      return This.Yaw;
   end GetYaw;

   procedure SetSpeed (This : in out Proxy; modulus, angle : Float) is
   begin
      This.speed := (Opt => O_SOME,
                     modulus => modulus,
                     angle => angle);
   end SetSpeed;

   procedure GoalReached (This : in out Proxy) is
   begin
      This.goal_reached := True;
   end GoalReached;

end Robot_Iface;
