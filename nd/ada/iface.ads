with nd;
with Interfaces.C;

package Iface
is
   type TParametersND is record
      geometryRect : Interfaces.C.short;
      front, back, left : Interfaces.C.C_float;
      R : Interfaces.C.C_float;
      holonomic : Interfaces.C.short;
      vlmax, vamax : Interfaces.C.C_float;
      almax, aamax : Interfaces.C.C_float;
      dsmax, dsmin : Interfaces.C.C_float;
      enlarge : Interfaces.C.C_float;
      discontinuity : Interfaces.C.C_float;
      T : Interfaces.C.C_float;
      -- float laser;
   end record;
   pragma Convention (C, TParametersND);

   -- Information of linear v, and angular velocities w.
   type TVelocities is record
      v       : Interfaces.C.C_float;   -- linear velocity
      w       : Interfaces.C.C_float;   -- angular velocity
      v_theta : Interfaces.C.C_float;   -- velocity angle (just if holonomous vehicle)
   end record;
   pragma Convention (C, TVelocities);

   type TCoordenadas is
      record
         x, y : Interfaces.C.C_float;
      end record;
   pragma Convention (C, TCoordenadas);

   type TSR is
      record
         posicion : TCoordenadas;
         orientacion : Interfaces.C.C_float;
      end record;
   pragma Convention (C, TSR);

   -- (information of the robot)
   type TInfoMovimiento is record
      SR1         : TSR;
      velocidades : TVelocities;
   end record;
   pragma Convention (C, TInfoMovimiento);

   type Obstacle_Points is
     array (nd.POINT_ID) of TCoordenadas;
   pragma Convention (C, Obstacle_Points);

   -- List of obstacle points.
   type TInfoEntorno is record
      longitud : Interfaces.C.int;
      punto    : Obstacle_Points;
   end record;
   pragma Convention (C, TInfoEntorno);

   type TDimensiones is
      record
         Rear : Interfaces.C.C_float;
         Left : Interfaces.C.C_float;
         Front : Interfaces.C.C_float;
         Right : Interfaces.C.C_float;
      end record;
   pragma Convention (C, TDimensiones);

   procedure InicializarND (parametros : TParametersND);
   pragma Export (C, InicializarND, "InicializarND");
   pragma Export_Procedure (Internal => InicializarND,
                           External => "InicializarND",
                           Mechanism => Reference);

   function IterarND
     (objetivo   : TCoordenadas;
      goal_tol   : Interfaces.C.C_float;
      movimiento : TInfoMovimiento;
      mapa       : TInfoEntorno)
      return       access TVelocities;
   pragma Export (C, IterarND, "IterarND");
   pragma Export_Function (Internal => IterarND,
                          External => "IterarND",
                          Mechanism => (objetivo => Value,
                                        goal_tol => Value,
                                        movimiento => Reference,
                                        mapa => Reference)
                         );
end Iface;
