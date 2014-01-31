with nd;
with geometria;
with Formal.Numerics; use Formal.Numerics;

package body Iface is

   use type Interfaces.C.short;

   procedure InicializarND (parametros : TParametersND) is
      p : constant nd.TParametersND :=
        (geometryRect => (parametros.geometryRect > 0),

         front => Positive_Float(parametros.front),
         back => Positive_Float(parametros.back),
         left => Positive_Float(parametros.left),

         R => Positive_Float(parametros.R),

         holonomic => (parametros.holonomic > 0),

         vlmax => Positive_Float(parametros.vlmax),
         vamax => Positive_Float(parametros.vamax),

         almax => Positive_Float(parametros.almax),
         aamax => Positive_Float(parametros.aamax),

         dsmax => Positive_Float(parametros.dsmax),
         dsmin => Positive_Float(parametros.dsmin),
         enlarge => NonNegative_Float(parametros.enlarge),

         discontinuity => Positive_Float(parametros.discontinuity),

         T => Positive_Float(parametros.T)
        );
   begin
      nd.InicializarND(p);
   end InicializarND;

   velocidades : aliased TVelocities;

   Null_TVelocities : constant access TVelocities := null;

   function IterarND
     (objetivo   : TCoordenadas;
      goal_tol   : Interfaces.C.C_float;
      movimiento : TInfoMovimiento;
      mapa       : TInfoEntorno)
      return access TVelocities
   is
      -- Input parameters.
      obj : constant geometria.TCoordenadas :=
        (x => Float(objetivo.x), y => Float(objetivo.y));
      gol : constant Float := Float(goal_tol);
      mov : constant Nd.TInfoMovimiento :=
        (SR1 => (posicion => (x => Float(movimiento.SR1.posicion.x),
                              y => Float(movimiento.SR1.posicion.y)),
                 orientacion => Float(movimiento.SR1.orientacion)),
         velocidades => (v => Float(movimiento.velocidades.v),
                         w => Float(movimiento.velocidades.w),
                         v_theta => Float(movimiento.velocidades.v_theta)));
      map : Nd.TInfoEntorno;

      -- Output parameters.
      ret : Nd.TVelocities_Option;

      use type Nd.Option;
   begin
      map.longitud := Nd.POINT_ID(mapa.longitud);
      for i in map.punto'Range loop
         map.punto(i) := (x => Float(mapa.punto(i).x),
                          y => Float(mapa.punto(i).y));
      end loop;

      ret := Nd.IterarND(obj, gol, mov, map);

      if ret.Opt = Nd.O_NONE then
         return Null_TVelocities;
      else
         velocidades := (v => Interfaces.C.C_float(ret.value.v),
                         w => Interfaces.C.C_float(ret.value.w),
                         v_theta => Interfaces.C.C_float(ret.value.v_theta)
                        );
         return velocidades'Access;
      end if;
   end IterarND;

end Iface;
