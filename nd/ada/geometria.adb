package body geometria is

   function Atan2(x, y : Unbounded_Float) return Unbounded_Float
   with
     Pre => X /= 0.0 or else Y /= 0.0,
     Post => Atan2'Result in -Pi .. Pi;

   function Atan2(x, y : Unbounded_Float) return Unbounded_Float renames Arctan;

   -- Note: We must swap the arguments.
--     function ARCOTANGENTE(x,y : Unbounded_Float) return Unbounded_Float
--     is
--     begin
--        return Arctan (X => x, Y => y);
--     end;

   ---------------------------
   -- TransformacionDirecta --
   ---------------------------

   procedure TransformacionDirecta (SR : TSR; p : in out TCoordenadas) is
      Dx : constant Unbounded_Float := p.x - SR.posicion.x;
      Dy : constant Unbounded_Float := p.y - SR.posicion.y;

      seno   : constant Unbounded_Float := Sin (SR.orientacion);
      coseno : constant Unbounded_Float := Cos (SR.orientacion);
   begin
      p.x := Dx * coseno + Dy * seno;
      p.y := -Dx * seno + Dy * coseno;
   end;
   pragma Inline(TransformacionDirecta);

   procedure TRANSFORMACION01(SR1 : TSR; p : in out TCoordenadas) renames TransformacionDirecta;
   procedure TRANSFORMACION12(SR2 : TSR; p : in out TCoordenadas) renames TransformacionDirecta;
   procedure TRANSFORMACION23(SR3 : TSR; p : in out TCoordenadas) renames TransformacionDirecta;

   ------------------
   -- AplicarCotas --
   ------------------

   procedure AplicarCotas (n : in out Unbounded_Float; i, s : Unbounded_Float) is
   begin
      if n < i then
         n := i;
      elsif n > s then
         n := s;
      end if;
   end;

   -------------------------
   -- DISTANCIA_CUADRADO2 --
   -------------------------

   function DISTANCIA_CUADRADO2 (p, q : TCoordenadas) return Unbounded_Float is
   begin
      return hypot(p.x - q.x, p.y - q.y);
   end;

   ----------------------------
   -- ConstruirCoordenadasCP --
   ----------------------------

   procedure ConstruirCoordenadasCP
     (p : out TCoordenadas;
      q : TCoordenadasPolares)
   is
   begin
      p.x := q.r * Cos (q.a);
      p.y := q.r * Sin (q.a);
   end;

   -----------------------------
   -- ConstruirCoordenadasCxy --
   -----------------------------

   procedure ConstruirCoordenadasCxy
     (p    : out TCoordenadas;
      x, y : Unbounded_Float)
   is
   begin
      p.x := x;
      p.y := y;
   end;

   -----------------------------
   -- ConstruirCoordenadasCra --
   -----------------------------

   procedure ConstruirCoordenadasCra
     (p    : out TCoordenadas;
      r    : Unbounded_Float;
      a    : Unbounded_Float)
   is
   begin
      p.x := r * Cos (a);
      p.y := r * Sin (a);
   end;

   ----------------------------
   -- ConstruirCoordenadasPC --
   ----------------------------

   procedure ConstruirCoordenadasPC
     (p : out TCoordenadasPolares;
      q : TCoordenadas)
   is
   begin
      p.r := Hypot (q.x, q.y);
      p.a := Atan2 (q.y, q.x);
   end;

   -----------------------------
   -- ConstruirCoordenadasPxy --
   -----------------------------

   procedure ConstruirCoordenadasPxy
     (p    : out TCoordenadasPolares;
      x, y : Unbounded_Float)
   is
   begin
      p.r := Hypot (x, y);
      p.a := Atan2 (y => x, x => y);
   end;

   -----------------------------
   -- ConstruirCoordenadasPra --
   -----------------------------

   procedure ConstruirCoordenadasPra
     (p    : out TCoordenadasPolares;
      r    : Unbounded_Float;
      a    : Unbounded_Float)
   is
   begin
      p.r := r;
      p.a := a;
   end;

   -----------------------------
   -- ConstruirCoordenadasPcC --
   -----------------------------

   procedure ConstruirCoordenadasPcC
     (p : out TCoordenadasPolares;
      q : TCoordenadas)
   is
   begin
      p.r := q.x * q.x + q.y * q.y; -- yes, square (?!).
      p.a := Atan2 (q.y, q.x);
   end;

   -------------------------
   -- SumarCoordenadasCxy --
   -------------------------

   procedure SumarCoordenadasCxy (p : in out TCoordenadas; x, y : Unbounded_Float) is
   begin
      p.x := p.x + x;
      p.y := p.y + y;
   end;

   --------------------------
   -- SumarCoordenadasCxyC --
   --------------------------

   procedure SumarCoordenadasCxyC
     (p    : TCoordenadas;
      x, y : Unbounded_Float;
      q    : out TCoordenadas)
   is
   begin
      q.x := p.x + x;
      q.y := p.y + y;
   end;

   -------------------------
   -- SumarCoordenadasCra --
   -------------------------

   procedure SumarCoordenadasCra
     (p : in out TCoordenadas;
      r : Unbounded_Float;
      a : Unbounded_Float) is
   begin
      p.x := p.x + r * Cos (a);
      p.y := p.y + r * Sin (a);
   end;

   --------------------------
   -- SumarCoordenadasCraC --
   --------------------------

   procedure SumarCoordenadasCraC
     (p    : TCoordenadas;
      r    : Unbounded_Float;
      a    : Unbounded_Float;
      q    : out TCoordenadas)
   is
   begin
      q.x := p.x + r * Cos (a);
      q.y := p.y + r * Sin (a);
   end;

   ----------------------
   -- TRANSFORMACION02 --
   ----------------------

   procedure TRANSFORMACION02 (SR1, SR2 : TSR; p : in out TCoordenadas) is
   begin
      TRANSFORMACION01 (SR1, p);
      TRANSFORMACION12 (SR2, p);
   end;

   ---------------------------
   -- TransformacionInversa --
   ---------------------------

   procedure TransformacionInversa (SR : TSR; p : in out TCoordenadas) is
      seno   : constant Unbounded_Float := Sin (SR.orientacion);
      coseno : constant Unbounded_Float := Cos (SR.orientacion);

      x : constant Unbounded_Float := SR.posicion.x + p.x * coseno - p.y * seno;
   begin
      p.y := SR.posicion.y + p.x * seno + p.y * coseno;
      p.x := x;
   end;

   -----------------------
   -- AnguloNormalizado --
   -----------------------

   function AnguloNormalizado (angulo : Unbounded_Float) return Unbounded_Float is
   begin
      -- Debe pertenecer a (-PI,PI].
      -- Todos los �ngulos que proceden de un "atan2" pertenecen a ese
      --intervalo.
      return Atan2 (Sin (angulo), Cos (angulo));
   end;

   ----------------------------------------------
   -- AnguloPerteneceIntervaloOrientadoCerrado --
   ----------------------------------------------

   function AnguloPerteneceIntervaloOrientadoCerrado
     (angulo, limite1, limite2 : Unbounded_Float)
      return                     Boolean
   is
   begin
      -- Intervalo orientado.
      -- Esta funci�n devuelve 1 si el �ngulo est� entre los l�mites;
      --0 en caso contrario.
      -- Todos los par�metros deben pertenecer al intervalo (-PI,PI].
      -- Si limite1==limite2, entonces el intervalo es de longitud 0.

      if limite2 >= limite1 then
         return angulo >= limite1 and then angulo <= limite2;
      else
         return angulo >= limite1 or else angulo <= limite2;
      end if;
   end;

   ------------------------------
   -- BisectrizAnguloOrientado --
   ------------------------------

   function BisectrizAnguloOrientado
     (limite1, limite2 : Unbounded_Float)
      return             Unbounded_Float
   is
      resultado : constant Unbounded_Float := (limite1 + limite2) / 2.0;
   begin
      -- Devuelve la bisectriz del �ngulo de "limite1" a "limite2" en
      -- sentido contrario a las agujas del reloj.

      if limite1 <= limite2 then
         return resultado;
      else
         return AnguloNormalizado(resultado + Pi);
      end if;
   end;

   --------------------------------
   -- BisectrizAnguloNoOrientado --
   --------------------------------

   function BisectrizAnguloNoOrientado
     (limite1, limite2 : Unbounded_Float)
      return             Unbounded_Float
   is
      resultado : constant Unbounded_Float := (limite1 + limite2) / 2.0;
   begin
      -- Devuelve la bisectriz del menor �ngulo formado por "limite1" y
      --"limite2", ya sea en el sentido de las agujas del reloj o en el
      --opuesto.

      if abs (limite1 - limite2) <= Pi then
         return resultado;
      else
         return AnguloNormalizado (resultado + Pi);
      end if;
   end;

   -----------------------------
   -- AmplitudAnguloOrientado --
   -----------------------------

   function AmplitudAnguloOrientado
     (limite1, limite2 : Unbounded_Float)
      return             Unbounded_Float
   is
      amplitud : constant Unbounded_Float := limite2 - limite1;
   begin
      -- Devuelve la amplitud del �ngulo de "limite1" a "limite2" en sentido
      --contrario a las agujas del reloj.

      if limite1 <= limite2 then
         return amplitud;
      else
         return 2.0 * Pi - amplitud;
      end if;
   end;

   -------------------------------
   -- AmplitudAnguloNoOrientado --
   -------------------------------

   function AmplitudAnguloNoOrientado
     (limite1, limite2 : Unbounded_Float)
      return             Unbounded_Float
   is
      amplitud : constant Unbounded_Float := abs (limite1 - limite2);
   begin
      -- Devuelve la amplitud del menor �ngulo formado por "limite1" y
      --"limite2", ya sea en el sentido de las agujas del reloj o en el
      --opuesto.

      if amplitud <= Pi then
         return amplitud;
      else
         return 2.0 * Pi - amplitud;
      end if;
   end;

   ----------------------------------
   -- MinimaDistanciaCuadradoCorte --
   ----------------------------------

   procedure MinimaDistanciaCuadradoCorte
     (pp1, pp2  : TCoordenadasPolares;
      angulo    : Unbounded_Float;
      distancia : in out Unbounded_Float)
   is
      pp1temp : TCoordenadasPolares := pp1;
      pp2temp : TCoordenadasPolares := pp2;
      p1, p2  : TCoordenadas;
      x       : Unbounded_Float;
   begin
      -- Mediante su aplicaci�n reiterada obtenemos el m�s pr�ximo de
      --entre los puntos de corte de un
      -- grupo de segmentos con una direcci�n determinada.
      -- "p1" y "p2" son los extremos de un segmento.
      -- "angulo" es la direcci�n de corte (desde el origen).
      -- "distancia" es la menor distancia obtenida hasta el momento.

      pp1temp.a := AnguloNormalizado (pp1.a - angulo);
      pp2temp.a := AnguloNormalizado (pp2.a - angulo);

      ConstruirCoordenadasCP (p1, pp1temp);
      ConstruirCoordenadasCP (p2, pp2temp);

      if p1.y * p2.y > 0.0 or else (p1.y = p2.y and then p1.y /= 0.0)  -- No hay
                                                                       --punto
                                                                       --de
                                                                       --corte.
      then
         return;
      end if;

      if p1.y = 0.0 and then p2.y = 0.0 and then p1.x * p2.x <= 0.0 then
         distancia := 0.0;
         return;
      end if;

      if p1.y = 0.0 or else p2.y = 0.0 then
         if p1.y = 0.0 and then p1.x >= 0.0 and then p1.x <= distancia then
            distancia := p1.x;
         end if;

         if p2.y = 0.0 and then p2.x >= 0.0 and then p2.x <= distancia then
            distancia := p2.x;
         end if;

         return;
      end if;

      x := (p1.x * p2.y - p2.x * p1.y) / (p2.y - p1.y);
      if x >= 0.0 and then x < distancia then
         distancia := x;
      end if;
   end;

end geometria;
