with geometria;

with Formal.Numerics; use Formal.Numerics;

package nd
is
   -- Information of the robot and laser for the ND.
   type TParametersND is record

      -- GEOMETRY
      -- The vehicle is considered to be symetric at both sides of the X axis.
      -- The flag is 1 if the robot is resctangular, 0 if it is circular
      geometryRect : Boolean; -- FIXME: enum

      -- --- RECTANGULAR ---
      -- distance (m) from the wheels to the:
      -- front: frontal part
      -- back: back part
      -- left: left side. Notice that the vehicle is symetric
      front, back, left : Positive_Float; -- FIXME: 0..Inf[m]

      -- --- CIRCULAR ---
      -- radius of the robot is is circular
      R : Positive_Float; -- FIXME: 0..Inf[m]

      -- MOTION
      -- The falg is 1 if the robot is holonomous, or 0 is diff-drive or syncro
      holonomic : Boolean; -- FIXME: enum

      -- Maximum linear and angular velocities
      vlmax, vamax : Positive_Float; -- FIXME: 0..Inf[m/s], 0..Inf[rad/s]

      -- Maximum linear and angular acelerations
      almax, aamax : Positive_Float; -- FIXME: 0..Inf[m/s^2], 0..Inf[rad/s^2]

      -- OTHER STUFF

      -- -- SECURITY DISTANCE ---
      -- Distance to consider an obstacle dangerous (i.e. to start the
      --avoidance maneouvre)
      -- dsmax: Distance from the frontal robot bounds.
      -- dsmin: Distance from the back robot bounds.
      -- enlarge: Inner value. The suggestion is 20% of the dsmin (i.e.
      --0.2*dsmin)
      dsmax, dsmin : Positive_Float;
      enlarge : NonNegative_Float;

      -- -- DISCONTINUITY --
      -- Minimum space where the robot fits. I suggest same value than
      --"izquierda" value.
      discontinuity : Positive_Float;

      -- -- SAMPLING PERIOD --
      T : Positive_Float; -- FIXME: (0..Inf[s])

      -- LASER
      -- Distance from the wheels axis to the laser, X axis.
      --float laser;

   end record;

   -- Information of linear v, and angular velocities w.
   type TVelocities is record
      v       : Float;   -- linear velocity
      w       : Float;   -- angular velocity
      v_theta : Float;   -- velocity angle (just if holonomous vehicle)
   end record;

   -- (information of the robot)
   type TInfoMovimiento is record
      SR1         : geometria.TSR;  -- Current vehicle location in GLOBAL
                                    -- coordinates
      velocidades : TVelocities;    -- Current vehicle velocities
   end record;

   -- Maximum number of points of the environment
   -- This number depends on the maximum number of obstacle points that
   -- you want to give to the ND

   -- #define MAX_POINTS_SCENARIO 1440
   MAX_POINTS_SCENARIO : constant := 1440;

   subtype POINT_ID is Natural range 0 .. MAX_POINTS_SCENARIO-1;

   type Obstacle_Points is
     array (POINT_ID) of geometria.TCoordenadas;

   -- List of obstacle points.
   type TInfoEntorno is record
      longitud : POINT_ID;
      punto    : Obstacle_Points;
   end record;

   -- package nd2;

   -- -------------------------------------------------------------------------
   -- CONSTANTES.
   -- -------------------------------------------------------------------------

   -- Numero de sectores: multiplo de 4.
   SECTORES : constant := 180; -- Positive
--      with Static_Predicate => SECTORES mod 4 = 0;

   subtype SECTOR_ID is Natural range 0 .. SECTORES - 1;

   subtype SECTOR_ID_Optional is Integer range -1 .. SECTOR_ID'Last;

   VERDADERO : constant Boolean := True;

   FALSO : constant Boolean := False;

   -- -------------------------------------------------------------------------
   -- TIPOS.
   -- -------------------------------------------------------------------------

   -- Informaci�n acerca del robot.

   -- Dimensiones del robot.
   --   Consideramos el robot definido por un rect�ngulo. Numeramos sus
   --   dimensiones, medidas a partir de su centro en las direcciones
   --principales,
   --   siguiendo la misma convenci�n que para los sectores:
   --     Dimension[0]: distance from the center to the rear of the robot..
   --     Dimension[1]: distance from the center to the left of the robot.
   --     Dimension[2]: distance from the center to the front of the robot.
   --     Dimension[3]: distance from the center to the right of the robot.

   -- type TDimension_Index is (Rear, Left, Front, Right);
   -- type TDimensiones is array (TDimension_Index) of NonZero_Float;
   type TDimensiones is
      record
         Rear : Negative_Float;
         Left : Positive_Float;
         Front : Positive_Float;
         Right : Negative_Float;
      end record;

--     type TDimensiones is record
--        rear, left, front, right : NonNegative_Float;
--     end TDimensiones;

   subtype TMatriz2x2_Index is Natural range 0 .. 1;

   -- FIXME: static predicates are not allowed for non-scalar types;
   -- FIXME: dynamic predicates are not yet implemented.
   -- with Predicate => TMatriz2x2(0,0) /= 0.0 and TMatriz2x2(1,1) /= 0.0;
   type TMatriz2x2 is array (TMatriz2x2_Index, TMatriz2x2_Index) of Float;

   type Sector_Ranges is array (SECTOR_ID) of Positive_Float;

   type TInfoRobot is record
      Dimensiones : TDimensiones;
      enlarge     : Float;

      geometriaRect : Boolean; -- Si es cuadrado o no

      R : NonNegative_Float; -- radio del robot por si es circular

      holonomo : Boolean;

      E  : Sector_Ranges;  -- Distancia desde el origen de SR2 al per�metro
                           --del robot.
      ds : Sector_Ranges;  -- Distancia de seguridad: desde el per�metro del
                           --robot al per�metro de seguridad.

      velocidad_lineal_maxima  : NonNegative_Float;
      velocidad_angular_maxima : NonNegative_Float;

      aceleracion_lineal_maxima  : NonNegative_Float;
      aceleracion_angular_maxima : NonNegative_Float;

      discontinuidad : Positive_Float; -- Espacio m�nimo por el que cabe el robot.

      T : NonNegative_Float; -- Periodo.

      H : TMatriz2x2; -- Generador de movimientos: "Inercia" del robot.
      G : TMatriz2x2;   -- Generador de movimientos: "Fuerza" aplicada sobre
                        --el robot.

   end record;

   -- Informaci�n acerca del objetivo.
   type TObjetivo is record
      c0, c1 : geometria.TCoordenadas;
      p1     : geometria.TCoordenadasPolares;
      s      : SECTOR_ID;
   end record;

   -- Informaci�n acerca de la regi�n escogida.

   type DIRECTION_TYPE is (
     DIRECCION_OBJETIVO,
     DIRECCION_DISCONTINUIDAD_INICIAL,
     DIRECCION_DISCONTINUIDAD_FINAL);

   type TRegion is record
      principio : SECTOR_ID_Optional;
      final     : SECTOR_ID_Optional;

      principio_ascendente : Boolean;
      final_ascendente     : Boolean;

      descartada : Boolean;

      direccion_tipo   : DIRECTION_TYPE;
      direccion_sector : SECTOR_ID_Optional;
      direccion_angulo : Float;
   end record;

   type TRegion_Vector is array (SECTOR_ID) of aliased TRegion;

   type TVRegiones is record
      longitud : Integer;
      vector   : TRegion_Vector;
   end record;

   type d_t is array (SECTOR_ID) of geometria.TCoordenadasPolares;
   type dr_t is array (SECTOR_ID) of Float;

   type Situation_TYPE is (
     HSGR,
     HSWR,
     HSNR,
     LS2,
     LSG,
     LS1);

   type CUTTING_TYPE is (
     CUTTING_NINGUNO,
     CUTTING_IZQUIERDA,
     CUTTING_DERECHA,
     CUTTING_AMBOS);

   -- Informaci�n interna del m�todo de navegaci�n.
   type TInfoND is record
      objetivo : TObjetivo;

      SR1         : geometria.TSR;                    -- Estado actual del
                                                      --robot: posici�n y
                                                      --orientaci�n.
      velocidades : TVelocities; -- Estado actual del robot: velocidades
                                 --lineal y angular.

      --TCoordenadasPolares d[SECTORES]; -- Distancia desde el centro del
      --robot al obst�culo m�s pr�ximo en cada sector (con �ngulos).
      d : d_t;
      --float dr[SECTORES]; -- Distancia desde el per�metro del robot al
      --obst�culo m�s pr�ximo en cada sector.
      dr : dr_t;

      regiones : TVRegiones;  -- S�lo como informaci�n de cara al
                              --exterior: Lista de todas las regiones
                              --encontradas en el proceso de selecci�n.
      region   : SECTOR_ID_Optional;-- Como almacenamos m�s de una regi�n
                                    --debemos indicar cu�l es la escogida.

      -- Should be -1 \/ SECTOR_ID;
      obstaculo_izquierda, obstaculo_derecha : SECTOR_ID_Optional;

      angulosin : Float;      -- S�lo como informaci�n de cara al
                              --exterior: �ngulo antes de tener en cuenta
                              --los obst�culos m�s pr�ximos.
      angulocon : Float;      -- S�lo como informaci�n de cara al
                              --exterior: �ngulo despu�s de tener en
                              --cuenta los obst�culos m�s pr�ximos.
      situacion : Situation_TYPE;   -- S�lo como informaci�n de cara al
                                    --exterior: Situaci�n en la que se
                                    --encuentra el robot.
      cutting   : CUTTING_TYPE;     -- S�lo como informaci�n de cara al
                                    --exterior: Cutting aplicado al movimiento
                                    --del robot.

      angulo    : Float;      -- Salida del algoritmo de navegaci�n y
                              --entrada al generador de movimientos:
                              --direcci�n de movimiento deseada.
      velocidad : Float;   -- Salida del algoritmo de navegaci�n y entrada
                           --al generador de movimientos: velocidad lineal
                           --deseada.

   end record;

   -- -------------------------------------------------------------------------
   -- VARIABLES.
   -- -------------------------------------------------------------------------

   --robot : TInfoRobot;

   -- end nd2;

   -- -------------------------------------------------------------------------
   -- FUNCTIONS
   -- -------------------------------------------------------------------------

   -- Itialization of the ND.
   -- Input--
   --		parametros:: information of the robot and laser used by the ND

   procedure InicializarND (parametros : TParametersND);

   -- This runs the ND. The input is the current obstacle list and the goal
   --location
   -- and the output the motion command.
   -- Input--
   --		objetivo::  current objective in GLOBAL coordinates. Notice that this
   --					location can change each time you call ND.
   --		movimiento:: this is the current velocity of the robot.
   --		mapa::  this is a list of the obstacle points in global coordinates.
   --				You can use the current sensor reading or implement a kind of memory
   --				to remember last scans. Whatever, ND wants a list of points in GLOBAL
   --coordinates.
   --		information:: variable for debug.
   --
   -- Ouput--
   --		movimiento:: this is the output of the ND.
   --					 * Linear and angular velocities (and direction if holonomic).
   --					 * NULL an emergency stop is required
   --					 * pointer to (0,0) goal reached.

   type Option is (O_NONE, O_SOME);

   type TVelocities_Option(Opt : Option := O_NONE) is
      record
         case Opt is
            when O_NONE =>
               null;
            when O_SOME =>
               value : TVelocities;
         end case;
      end record;

   function IterarND
     (objetivo   : geometria.TCoordenadas;
      goal_tol   : Float;
      movimiento : TInfoMovimiento;
      mapa       : TInfoEntorno)
   --;void *informacion
   -- if you do not want to see the internal information in nh2.h informacion
   --= NULL
      return TVelocities_Option;
end nd;
