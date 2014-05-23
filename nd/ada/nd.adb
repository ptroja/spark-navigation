with Formal.Numerics.Elementary_Functions;
with geometria;

use Formal.Numerics.Elementary_Functions;
use geometria;

package body nd
is
   -- ----------------------------------------------------------------------------
   -- CONSTANTES.
   -- ----------------------------------------------------------------------------

   DISTANCIA_INFINITO : constant := 1_000_000.0;

   -- ----------------------------------------------------------------------------
   -- VARIABLES.
   -- ----------------------------------------------------------------------------
   -- FILE *depuracion;
   --   iteracion : Integer := 0;

   robot : TInfoRobot;

   -- ----------------------------------------------------------------------------
   -- FUNCIONES.
   -- ----------------------------------------------------------------------------

   -- ----------------------------------------------------------------------------
   -- Operaciones con sectores.
   -- ----------------------------------------------------------------------------

   function MINIMO (a, b : Float) return Float renames Float'Min;
   function MAXIMO (a, b : Float) return Float renames Float'Max;

   function INCREMENTAR_SECTOR (s : SECTOR_ID) return SECTOR_ID is
     ((s + 1) mod SECTORES);

   function DECREMENTAR_SECTOR (s : SECTOR_ID) return SECTOR_ID is
     ((s + (SECTORES - 1)) mod SECTORES);

   function sector2angulo (sector : SECTOR_ID) return Float
     is (Pi * (1.0 - 2.0 * (Float (sector) / Float (SECTORES))));
--     with
--       Post => sector2angulo'Result in -Pi .. Pi and
--     (if sector > SECTOR_ID'First then sector2angulo'Result <  Pi ) and
--     (if sector < SECTOR_ID'Last  then sector2angulo'Result > -Pi );

--     -- Esta funci�n NO debe declararse como est�tica.
--     function sector2angulo (sector : in SECTOR_ID) return Float is
--        FACTOR : constant := -2.0*PI/Float(SECTORES);
--        SUMANDO : constant := -Float(SECTORES)/2.0;
--     begin
--        return FACTOR*(Float(sector)+SUMANDO);
--     end;

   function angulo2sector (angulo : Unbounded_Float) return SECTOR_ID
     is (Integer ((-Float (SECTORES) / (2.0 * Pi)) * angulo + ((Float (SECTORES + 1)) / 2.0)))
   with
     Pre => angulo in -Pi .. Pi;

   function ObtenerSectorP (p : TCoordenadasPolares) return SECTOR_ID is
      FACTOR : constant := -Float (SECTORES) / (2.0 * Pi);
      SUMANDO : constant := (Float (SECTORES) + 1.0) / 2.0;
      -- FIXME: Ada casting give different result from C, but we do not care.
   begin
      return Integer (FACTOR * p.a + SUMANDO) mod SECTORES;
   end ObtenerSectorP;
   pragma Inline (ObtenerSectorP);

   function DistanciaSectorialOrientada (s1, s2 : SECTOR_ID) return SECTOR_ID
   is
   begin
      if s1 <= s2 then
         return s2 - s1;
      else
         return ((s2 + SECTORES) - s1) mod SECTORES;
      end if;
   end DistanciaSectorialOrientada;

   -- ----------------------------------------------------------------------------
   -- InicializarND y sus funciones auxiliares.
   -- ----------------------------------------------------------------------------

--     procedure InicializarE
--     with
--       Pre => robot.Dimensiones.Rear > 0.0 and
--       robot.Dimensiones.Left > 0.0;

   procedure InicializarE is
      -- Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
      -- hasta el per�etro (que contiene el origen) en la direcci�n de la bisectriz
      -- de cada sector.
      limite : TCoordenadasPolares;
      li, ld : SECTOR_ID;
   begin
                                                                             -- arctan(y > 0, x < 0)
      limite.a := ARCOTANGENTE (robot.Dimensiones.Rear, robot.Dimensiones.Left); -- limite.a in ( pi/2 ; pi )
      li := angulo2sector (limite.a);                                           -- li in 0..45
      if sector2angulo (li) > limite.a and then li < SECTOR_ID'Last then
         li := SECTOR_ID'Succ (li);
      end if;

      ConstruirCoordenadasPxy (limite, robot.Dimensiones.Front, robot.Dimensiones.Left); -- limite.a in ( 0 ; pi/2 )
      ld := angulo2sector (limite.a);                                                    -- ld in 46 ..90
      if sector2angulo (ld) > limite.a and then ld < SECTOR_ID'Last then
         ld := SECTOR_ID'Succ (ld);
      end if;

      robot.E (SECTOR_ID'First) := -robot.Dimensiones.Rear;

      for i in 1 .. li - 1 loop
         robot.E (i) := robot.Dimensiones.Rear / Cos (sector2angulo (i));
      end loop;

      for i in li .. ld - 1 loop
         robot.E (i) := robot.Dimensiones.Left / Sin (sector2angulo (i));
      end loop;

      for i in ld .. SECTORES / 2 loop
         robot.E (i) := limite.r;
      end loop;

      for i in SECTORES / 2 + 1 .. SECTOR_ID'Last loop
         robot.E (i) := robot.E (SECTORES - i); -- Por simetria respecto del eje X.
      end loop;

   end InicializarE;

   procedure InicializarERedondo is
   begin
      -- Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
      -- hasta el per�etro (que contiene el origen) en la direcci�n de la bisectriz
      -- de cada sector.
      robot.E := (others => robot.R);
   end InicializarERedondo;

   procedure InicializarDSRedondo (dmax : Positive_Float) is
   begin
      -- Calcula la distancia desde el origen (punto de coordenadas 0.0F,0.0F)
      -- hasta el per�etro (que contiene el origen) en la direcci�n de la bisectriz
      -- de cada sector.
      robot.ds := (others => dmax);
   end InicializarDSRedondo;

   procedure InicializarDS (dsmax, dsmin : Positive_Float)
   with
      Pre => (robot.Dimensiones.Front - robot.Dimensiones.Rear) - (dsmax - dsmin) >= 0.0 and then dsmax > dsmin;

   procedure InicializarDS (dsmax, dsmin : Positive_Float) is

      p1, p2 : TCoordenadas;
      q1, q2, q3 : TCoordenadas;
      q4 : TCoordenadasPolares;
      limite1, limite2, limite3, limite4 : float;
      coseno, seno : NonNegative_Float;
      a, b, c, m, n : float;
      angulo, distancia : float;
   begin

      ConstruirCoordenadasCxy (p1, robot.Dimensiones.Rear, robot.Dimensiones.Left);
      ConstruirCoordenadasCxy (p2, robot.Dimensiones.Front, robot.Dimensiones.Left);

      b := dsmax - dsmin;  -- b > 0.0
      c := p2.x - p1.x; -- c > 0.0
      a := Sqrt (c * c - b * b); -- a:=sqrt((c-b)*(c+b)); -- FIXME: only Metitarski can prove the numerically simpler form. -- a >= 0.0
--      pragma Assert_and_cut(dsmin > 0.0 and dsmax > 0.0 and a >= 0.0 and b > 0.0 and c > 0.0);
      coseno := a / c; -- >= 0.0
      seno := b / c; -- > 0.0
--      pragma Assert(seno > 0.0);
      SumarCoordenadasCxyC (p1, -dsmin, 0.0, q1);
      SumarCoordenadasCxyC (p1, -dsmin * seno, dsmin * coseno, q2);
      SumarCoordenadasCxyC (p2, -dsmax * seno, dsmax * coseno, q3);
      ConstruirCoordenadasPC (q4, p2);
      q4.r := q4.r + dsmax;

      limite1 := ARCOTANGENTE (q1.x, q1.y);
      limite2 := ARCOTANGENTE (q2.x, q2.y);
      limite3 := ARCOTANGENTE (q3.x, q3.y);
      limite4 := q4.a;

      robot.ds (0) := -q1.x - robot.E (0); -- Pre => robot.Dimensiones.Rear - dsmin - robot.E(0) > 0.0

      m := CUADRADO (p1.x) + CUADRADO (p1.y) - CUADRADO (dsmin);
      n := CUADRADO (p2.x) + CUADRADO (p2.y) - CUADRADO (dsmax);

      b := q3.x - q2.x;
      c := q3.y - q2.y;
      a := b * q2.y - c * q2.x;

      for i in 1 .. SECTORES / 2 - 1 loop
         angulo := sector2angulo (i);

         -- C�lculo de la distancia de seguridad correspondiente a la bisectriz del sector i.

         if angulo >= limite1 then

            -- r1
            distancia := q1.x / Cos (angulo);

         elsif angulo >= limite2 then

            -- r2
            distancia := p1.x * Cos (angulo) + p1.y * Sin (angulo);
            distancia := distancia + Sqrt (CUADRADO (distancia) - m);

         elsif angulo >= limite3 then

            -- r3
            distancia := a / (b * Sin (angulo) - c * Cos (angulo));

         elsif angulo >= limite4 then

            -- r4
            distancia := p2.x * Cos (angulo) + p2.y * Sin (angulo);
            distancia := distancia + Sqrt (CUADRADO (distancia) - n);

         else

            -- r5
            distancia := q4.r;

         end if;

         -- Fin del c�lculo de la distancia de seguridad correspondiente a la bisectriz del sector i.

         robot.ds (i) := distancia - robot.E (i);
         robot.ds (SECTORES - i) := robot.ds (i); -- El robot es sim�trico respecto del eje X.

      end loop;

      robot.ds (SECTORES / 2) := q4.r - robot.E (SECTORES / 2); -- = q4.x/(float)cos(0.0F) - ...;
   end InicializarDS;

   procedure InicializarND (parametros : TParametersND) is
   begin
      robot.geometriaRect := parametros.geometryRect;
      robot.holonomo := parametros.holonomic;

      if parametros.geometryRect then
         -- Cuadrado

         robot.Dimensiones.Rear := -parametros.back;
         robot.Dimensiones.Left := parametros.left;
         robot.Dimensiones.Front := parametros.front;
         robot.Dimensiones.Right := -robot.Dimensiones.Left;

         robot.enlarge := parametros.enlarge;

         InicializarE;
         InicializarDS (parametros.dsmax, parametros.dsmin);

      else
         -- Redondo
         robot.R := parametros.R;
         InicializarERedondo;
         InicializarDSRedondo (parametros.dsmax);
      end if;

      robot.velocidad_lineal_maxima := parametros.vlmax;
      robot.velocidad_angular_maxima := parametros.vamax;

      robot.aceleracion_lineal_maxima := parametros.almax;
      robot.aceleracion_angular_maxima := parametros.aamax;

      robot.discontinuidad := parametros.discontinuity;

      robot.T := parametros.T;

      if not robot.holonomo then
         robot.H (0, 0) := Exp (-parametros.almax * parametros.T / parametros.vlmax); -- in (0.0 .. 1.0)
         robot.H (0, 1) := 0.0; -- Se tiene en cuenta m�s adelante y no se incluye en las ecuaciones.
         robot.H (1, 0) := 0.0; -- Se tiene en cuenta m�s adelante y no se incluye en las ecuaciones.
         robot.H (1, 1) := Exp (-parametros.aamax * parametros.T / parametros.vamax); -- in (0.0 .. 1.0)

         robot.G (0, 0) := (1.0 - Exp (-parametros.almax * parametros.T / parametros.vlmax)) * (parametros.vlmax / parametros.almax);
         robot.G (0, 1) := 0.0; -- Se tiene en cuenta m�s adelante y no se incluye en las ecuaciones.
         robot.G (1, 0) := 0.0; -- Se tiene en cuenta m�s adelante y no se incluye en las ecuaciones.
         robot.G (1, 1) := (1.0 - Exp (-parametros.aamax * parametros.T / parametros.vamax)) * (parametros.vamax / parametros.almax); -- Y no "aamax".
      end if;
   end InicializarND;

   ----------------------------------------------------------------------------
   --  -- IterarND y sus funciones auxiliares.
   --  -- ----------------------------------------------------------------------------
   --
   --  -- IterarND / SectorizarMapa
   --
   procedure SectorizarMapa (mapa : TInfoEntorno; nd : in out TInfoND) is
      p : TCoordenadas;
      pp : TCoordenadasPolares; -- Modulos al cuadrado para evitar ra�es innecesarias.
      j : Integer;
   begin

      nd.d := (others => (r => -1.0, a => 0.0));

      for i in 0 .. mapa.longitud - 1 loop
         p := mapa.punto (i);
         TRANSFORMACION01 (nd.SR1, p);
         ConstruirCoordenadasPcC (pp, p);

         j := ObtenerSectorP (pp);
         if nd.d (j).r < 0.0 or else pp.r < nd.d (j).r then
            nd.d (j) := pp;
         end if;
      end loop;

      for i in SECTOR_ID'Range loop
         if nd.d (i).r >= 0.0 then
            nd.d (i).r := Sqrt (nd.d (i).r);
            if i /= SECTORES / 2 and then nd.d (i).r < robot.E (i) + 0.01 then
               nd.d (i).r := robot.E (i) + 0.01;
            end if;
         end if;
      end loop;
   end SectorizarMapa;

   -- ----------------------------------------------------------------------------
   --
   --  -- IterarND / ParadaEmergencia
   --
   function ParadaEmergencia (nd : TInfoND) return Boolean is
      p : TCoordenadas;
      pp : TCoordenadasPolares;
   begin

      -- Devuelve 1 si hay peligro de colisi�n y hay que hacer una parada de emergencia;
      -- devuelve 0 en caso contrario.
      -- En la detecci�n de colisi�n se tiene en cuenta que el robot es sim�trico respecto del eje X.

      -- Detecta si obstaculo en la parte delantera
      ConstruirCoordenadasCxy (p, robot.Dimensiones.Front, robot.Dimensiones.Left);
      ConstruirCoordenadasPC (pp, p);

      for i in angulo2sector (pp.a) .. angulo2sector (-pp.a) loop
         if nd.d (i).r >= 0.0 and then nd.d (i).r <= pp.r and then abs (nd.d (i).a) <= pp.a then
            return True;
         end if;
      end loop;

      return False;
   end ParadaEmergencia;
   ----------------------------------------------------------------------------

   -- IterarND / SeleccionarRegiones / SiguienteDiscontinuidad

   procedure SiguienteDiscontinuidad (nd : TInfoND;
                                     principio : SECTOR_ID;
                                     izquierda : Boolean;
                                     discontinuidad : in out SECTOR_ID_Optional;
                                     ascendente : out Boolean) is
      --# hide SiguienteDiscontinuidad;
      i, j : SECTOR_ID;
      distancia_i, distancia_j : Float;
      no_obstaculo_i, no_obstaculo_j : Boolean;

      function Inv_Dec (I : SECTOR_ID) return Positive is
        ((if I <= principio then  I + SECTORES else I) - principio);
      -- with Convention => Ghost;

      function Inv_Inc (I : SECTOR_ID) return Positive is
        (SECTORES + principio - (if I >= principio then I else  I + SECTORES));
      -- with Convention => Ghost;

   begin

      -- Se busca desde "principio" en la direcci�n indicada por "izquierda".
      j := principio;
      distancia_j := nd.d (j).r;
      no_obstaculo_j := (distancia_j < 0.0);

      loop
         pragma Loop_Variant (Decreases => (if izquierda then Inv_Dec (j) else Inv_Inc (j)));
         pragma Loop_Invariant (True);
         i := j;
         distancia_i := distancia_j;
         no_obstaculo_i := no_obstaculo_j;

         j := (if izquierda then DECREMENTAR_SECTOR (i) else INCREMENTAR_SECTOR (i));

         distancia_j := nd.d (j).r;
         no_obstaculo_j := (distancia_j <= 0.0);

         if no_obstaculo_i and then no_obstaculo_j then
            null;
         else
            if no_obstaculo_i or else no_obstaculo_j then
               discontinuidad := i;
               ascendente := no_obstaculo_i;
               return;
            end if;

            -- FIXME: 'abs' makes loop_invariant unprovable.
            declare
               function dij return Float is
               begin
                  return abs (distancia_i - distancia_j);
               end dij;
            begin
               if dij >= robot.discontinuidad then
                  discontinuidad := i;
                  ascendente := (distancia_i > distancia_j);
                  return;
               end if;
            end;
         end if;

         exit when j = principio;
      end loop;

      discontinuidad := -1;
   end SiguienteDiscontinuidad;

   procedure ObjetivoAlcanzableSPARK (nd : TInfoND;
                                     region : in out TRegion;
                                     direccion_tipo : DIRECTION_TYPE;
                                     ret : out Boolean
                                    )
   with
     Pre => (direccion_tipo = DIRECCION_DISCONTINUIDAD_INICIAL and then region.principio /= -1)
     or else
       (direccion_tipo = DIRECCION_DISCONTINUIDAD_FINAL and then region.final /= -1)
     or else
       (direccion_tipo = DIRECCION_OBJETIVO);

   procedure ObjetivoAlcanzableSPARK (nd : TInfoND;
                                     region : in out TRegion;
                                     direccion_tipo : DIRECTION_TYPE;
                                     ret : out Boolean
                                    )
   is
      FL, FR : array (SECTOR_ID) of TCoordenadas;

      subtype SECTOR_ID_PLUS is Natural range SECTOR_ID'First .. Natural'Succ (SECTOR_ID'Last);

      nl, nr : SECTOR_ID_PLUS;

      objetivo_intermedio_polares : TCoordenadasPolares; -- Respecto de SR1.
      objetivo_intermedio : TCoordenadas;                -- Respecto de un SR con origen en el origen de SR1 y girado hasta que el semieje positivo de abscisas coincide con la direcci�n al objetivo intermedio.

      sector_auxiliar : SECTOR_ID;
      limite : Float;

      p1, p2, p : TCoordenadas;
   begin

      -- "direccion_tipo" puede tomar los siguientes valores declarados en 'nd2.h':
      -- - DIRECCION_OBJETIVO
      -- - DIRECCION_DISCONTINUIDAD_INICIAL
      -- - DIRECCION_DISCONTINUIDAD_FINAL

      region.direccion_tipo := direccion_tipo;

      if region.direccion_tipo = DIRECCION_OBJETIVO then

         region.direccion_sector := nd.objetivo.s;
         objetivo_intermedio_polares := nd.objetivo.p1;

      else

         if region.direccion_tipo = DIRECCION_DISCONTINUIDAD_INICIAL then
            region.direccion_sector := region.principio;
            sector_auxiliar := DECREMENTAR_SECTOR (region.direccion_sector);
         else
            region.direccion_sector := region.final;
            sector_auxiliar := INCREMENTAR_SECTOR (region.direccion_sector);
         end if;

         if nd.d (region.direccion_sector).r < 0.0 then
            ConstruirCoordenadasPra (objetivo_intermedio_polares, nd.d (sector_auxiliar).r + DISTANCIA_INFINITO,
                                    BisectrizAnguloNoOrientado (sector2angulo (region.direccion_sector), nd.d (sector_auxiliar).a));
         else
            ConstruirCoordenadasCP (p1, nd.d (region.direccion_sector));
            ConstruirCoordenadasCP (p2, nd.d (sector_auxiliar));
            ConstruirCoordenadasPxy (objetivo_intermedio_polares, (p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
         end if;

      end if;

      region.direccion_angulo := objetivo_intermedio_polares.a;
      ConstruirCoordenadasCxy (objetivo_intermedio, objetivo_intermedio_polares.r, 0.0);

      -- Determinaci�n de si el objetivo est�Edentro de un C-Obst�culo y
      -- construcci�n de las listas de puntos FL y FR.

      limite := CUADRADO (robot.discontinuidad / 2.0); -- Para no hacer ra�es cuadradas dentro del bucle.
      nl := 0;
      nr := 0;
      for i in SECTOR_ID'Range loop
         pragma Loop_Invariant (nl <= i and then nr <= i);

         if nd.d (i).r < 0.0 then -- Si no existe un obst�culo en el sector actual, pasamos al siguiente sector.
            null;
         else

            ConstruirCoordenadasCra (p, nd.d (i).r, nd.d (i).a - region.direccion_angulo);
            if p.x < 0.0 or else p.x >= objetivo_intermedio.x or else abs (p.y) > robot.discontinuidad then -- Si el obst�culo no est�Een el rect�ngulo que consideramos, pasamos al siguiente sector.
               null;
            else

               if DISTANCIA_CUADRADO2 (p, objetivo_intermedio) < limite then -- Si el objetivo intermedio est�Een colisi�n con el obst�culo, es inalcanzable.
                  ret := False; -- Objetivo intermedio inalcanzable.
                  return;
               end if;

               if p.y > 0.0 then
                  FL (nl) := p;
                  nl := nl + 1;
               else
                  FR (nr) := p;
                  nr := nr + 1;
               end if;

            end if;
         end if;
      end loop;

      -- Determinaci�n de si los obst�culos nos impiden alcanzar el objetivo intermedio.

      limite := CUADRADO (robot.discontinuidad); -- Para no hacer ra�es cuadradas dentro de los bucles.
      for i in 0 .. nl - 1 loop
         for j in 0 .. nr - 1 loop
            if DISTANCIA_CUADRADO2 (FL (i), FR (j)) < limite then
               ret := False;
               return;
            end if;
         end loop;
      end loop;

      ret := True; -- Objetivo intermedio alcanzable.
   end ObjetivoAlcanzableSPARK;

   procedure SeleccionarRegionSPARK (nd : in out TInfoND) is

      IZQUIERDA : constant Boolean := VERDADERO;
      DERECHA : constant Boolean := FALSO;

      objetivo_a_la_vista : constant Boolean := nd.d (nd.objetivo.s).r < 0.0 or else nd.objetivo.p1.r <= nd.d (nd.objetivo.s).r;

      indice, indice_izquierda, indice_derecha, indice_auxiliar : SECTOR_ID;
      distancia_izquierda, distancia_derecha : SECTOR_ID;
   begin

      -- Inicializamos el vector de regiones.

      nd.regiones.longitud := 0;

      indice := nd.regiones.longitud;
      nd.regiones.longitud := nd.regiones.longitud + 1;
      -- region:=nd.regiones.vector(indice)'Access;
      nd.regiones.vector (indice).descartada := FALSO;

      nd.region := -1;

      -- Buscamos la primera discontinuidad.

      SiguienteDiscontinuidad (nd, nd.objetivo.s, IZQUIERDA, nd.regiones.vector (indice).principio, nd.regiones.vector (indice).principio_ascendente);
      if nd.regiones.vector (indice).principio = -1 then

         -- No hay discontinuidades.

         nd.regiones.vector (indice).principio := SECTOR_ID'First;
         nd.regiones.vector (indice).final := SECTOR_ID'Last;

         if objetivo_a_la_vista then
            nd.regiones.vector (indice).direccion_tipo := DIRECCION_OBJETIVO;
            nd.regiones.vector (indice).direccion_sector := nd.objetivo.s;
            nd.regiones.vector (indice).direccion_angulo := nd.objetivo.p1.a;
            nd.region := indice;
            return;
         end if;

         -- Objetivo inalcanzable.
         nd.regiones.vector (indice).descartada := VERDADERO;
         return;
      end if;

      pragma Assert_And_Cut (nd.regiones.vector (indice).principio /= -1 and then nd.regiones.longitud = 1);

      -- Existe al menos una discontinuidad.

      SiguienteDiscontinuidad (nd, nd.objetivo.s, DERECHA, nd.regiones.vector (indice).final, nd.regiones.vector (indice).final_ascendente);
      if nd.regiones.vector (indice).final = DECREMENTAR_SECTOR (nd.regiones.vector (indice).principio) then

         -- Hay una sola discontinuidad.

         if objetivo_a_la_vista then
            nd.regiones.vector (indice).direccion_tipo := DIRECCION_OBJETIVO;
            nd.regiones.vector (indice).direccion_sector := nd.objetivo.s;
            nd.regiones.vector (indice).direccion_angulo := nd.objetivo.p1.a;
            nd.region := indice;
            return;
         end if;

         if nd.regiones.vector (indice).principio_ascendente then
            declare
               ret : Boolean;
            begin
               ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice), DIRECCION_DISCONTINUIDAD_INICIAL, ret);
               if ret then
                  nd.region := indice;
                  return;
               end if;
            end;
         else
            declare
               ret : Boolean;
            begin
               ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice), DIRECCION_DISCONTINUIDAD_FINAL, ret);
               if ret then
                  nd.region := indice;
                  return;
               end if;
            end;
         end if;

         -- Objetivo inalcanzable.
         nd.regiones.vector (indice).descartada := VERDADERO;
         return;
      end if;

      pragma Assert_And_Cut (nd.regiones.vector (indice).principio /= -1 and then nd.regiones.longitud = 1);

      -- Hay dos o m�s discontinuidades.

      if objetivo_a_la_vista then

         -- Regi�n del objetivo.

         if (not nd.regiones.vector (indice).principio_ascendente) and then (not nd.regiones.vector (indice).final_ascendente) then

            -- Regi�n artificial.

            indice_auxiliar := nd.regiones.longitud; -- No incrementamos la longitud del vector: Nuestra regi�n auxiliar es ilegal.
            -- region_auxiliar:=nd.regiones.vector(indice_auxiliar)'Access;

            nd.regiones.vector (indice_auxiliar) := nd.regiones.vector (indice); -- Utilizamos la regi�n auxiliar para almacenar el contenido de la regi�n que vamos a modificar.

            nd.regiones.vector (indice).principio := nd.objetivo.s;
            nd.regiones.vector (indice).final := nd.objetivo.s;

            declare
               ret : Boolean;
            begin
               ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice), DIRECCION_OBJETIVO, ret);

               if ret then
                  nd.region := indice;
                  return;
               end if;
            end;

            nd.regiones.longitud := nd.regiones.longitud + 1; -- Regularizamos la situaci�n de nuestra regi�n auxiliar y
            indice := indice_auxiliar;  -- la escogemos como regi�n a examinar.
            -- region:=region_auxiliar;

         else
            declare
               ret : Boolean;
            begin

               ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice), DIRECCION_OBJETIVO, ret);
               if ret then

                  -- Regi�n "natural".

                  nd.region := indice;
                  return;
               end if;
            end;
         end if;

      end if;

      indice_izquierda := indice;

      indice_derecha := indice;

      loop

         distancia_izquierda := DistanciaSectorialOrientada (nd.regiones.vector (indice_izquierda).principio, nd.objetivo.s);
         distancia_derecha := DistanciaSectorialOrientada (nd.objetivo.s, nd.regiones.vector (indice_derecha).final);

         if distancia_izquierda <= distancia_derecha then

            -- Probamos por la regi�n izquierda.

            if nd.regiones.vector (indice_izquierda).principio_ascendente then

               declare
                  ret : Boolean;
               begin
                  ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice_izquierda), DIRECCION_DISCONTINUIDAD_INICIAL, ret);

                  if ret then
                     if nd.regiones.vector (indice_derecha).principio_ascendente or else nd.regiones.vector (indice_derecha).final_ascendente then
                        nd.region := indice_izquierda;
                        return;
                     end if;

                     nd.regiones.longitud := nd.regiones.longitud - 1;

                     if indice_derecha > indice_izquierda then
                        nd.region := indice_izquierda;
                        return;
                     end if;

                     nd.regiones.vector (indice_derecha) := nd.regiones.vector (indice_izquierda);
                     nd.region := indice_derecha;
                     return;
                  end if;
               end;

               if indice_izquierda /= indice_derecha then
                  nd.regiones.vector (indice_izquierda).descartada := VERDADERO;
               end if;

               indice_auxiliar := indice_izquierda;

               indice_izquierda := nd.regiones.longitud;
               nd.regiones.longitud := nd.regiones.longitud + 1;
               -- region_izquierda:=nd.regiones.vector(indice_izquierda)'Access;
               nd.regiones.vector (indice_izquierda).descartada := FALSO;

               nd.regiones.vector (indice_izquierda).final := DECREMENTAR_SECTOR (nd.regiones.vector (indice_auxiliar).principio);
               nd.regiones.vector (indice_izquierda).final_ascendente := not nd.regiones.vector (indice_auxiliar).principio_ascendente;

               SiguienteDiscontinuidad (nd, nd.regiones.vector (indice_izquierda).final, IZQUIERDA, nd.regiones.vector (indice_izquierda).principio, nd.regiones.vector (indice_izquierda).principio_ascendente);

            else -- Principio descendente: Ser�Eun final ascendente en la siguiente regi�n izquierda.

               if indice_izquierda /= indice_derecha then

                  nd.regiones.vector (indice_izquierda).final := DECREMENTAR_SECTOR (nd.regiones.vector (indice_izquierda).principio);
                  nd.regiones.vector (indice_izquierda).final_ascendente := not nd.regiones.vector (indice_izquierda).principio_ascendente;

               else

                  indice_auxiliar := indice_izquierda;

                  indice_izquierda := nd.regiones.longitud;
                  nd.regiones.longitud := nd.regiones.longitud + 1;
                  -- region_izquierda:=nd.regiones.vector(indice_izquierda)'Access;
                  nd.regiones.vector (indice_izquierda).descartada := FALSO;

                  nd.regiones.vector (indice_izquierda).final := DECREMENTAR_SECTOR (nd.regiones.vector (indice_auxiliar).principio);
                  nd.regiones.vector (indice_izquierda).final_ascendente := not nd.regiones.vector (indice_auxiliar).principio_ascendente;

               end if;

               SiguienteDiscontinuidad (nd, nd.regiones.vector (indice_izquierda).final, IZQUIERDA, nd.regiones.vector (indice_izquierda).principio, nd.regiones.vector (indice_izquierda).principio_ascendente);

               declare
                  ret : Boolean;
               begin
                  ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice_izquierda), DIRECCION_DISCONTINUIDAD_FINAL, ret);

                  if ret then
                     if nd.regiones.vector (indice_derecha).principio_ascendente or else nd.regiones.vector (indice_derecha).final_ascendente then
                        nd.region := indice_izquierda;
                        return;
                     end if;

                     nd.regiones.longitud := nd.regiones.longitud - 1;

                     if indice_derecha > indice_izquierda then
                        nd.region := indice_izquierda;
                        return;
                     end if;

                     nd.regiones.vector (indice_derecha) := nd.regiones.vector (indice_izquierda);
                     nd.region := indice_derecha;
                     return;
                  end if;
               end;

            end if;

         else

            -- Probamos por la regi�n derecha.

            if nd.regiones.vector (indice_derecha).final_ascendente then

               declare
                  ret : Boolean;
               begin
                  ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice_derecha), DIRECCION_DISCONTINUIDAD_FINAL, ret);

                  if ret then
                     if nd.regiones.vector (indice_izquierda).principio_ascendente or else nd.regiones.vector (indice_izquierda).final_ascendente then
                        nd.region := indice_derecha;
                        return;
                     end if;

                     if indice_izquierda > indice_derecha then
                        nd.regiones.longitud := nd.regiones.longitud - 1;
                        nd.region := indice_derecha;
                        return;
                     end if;

                     nd.regiones.vector (indice_izquierda) := nd.regiones.vector (indice_derecha);
                     nd.regiones.longitud := nd.regiones.longitud - 1;
                     nd.region := indice_izquierda;
                     return;
                  end if;
               end;

               if indice_derecha /= indice_izquierda then
                  nd.regiones.vector (indice_derecha).descartada := VERDADERO;
               end if;

               indice_auxiliar := indice_derecha;

               indice_derecha := nd.regiones.longitud;
               nd.regiones.longitud := nd.regiones.longitud + 1;
               -- region_derecha:=nd.regiones.vector(indice_derecha)'Access;
               nd.regiones.vector (indice_derecha).descartada := FALSO;

               nd.regiones.vector (indice_derecha).principio := INCREMENTAR_SECTOR (nd.regiones.vector (indice_auxiliar).final);
               nd.regiones.vector (indice_derecha).principio_ascendente := not nd.regiones.vector (indice_auxiliar).final_ascendente;

               SiguienteDiscontinuidad (nd, nd.regiones.vector (indice_derecha).principio, DERECHA, nd.regiones.vector (indice_derecha).final, nd.regiones.vector (indice_derecha).final_ascendente);

            else -- Final descendente: Ser�Eun principio ascendente en la siguiente regi�n derecha.

               if indice_derecha /= indice_izquierda then

                  nd.regiones.vector (indice_derecha).principio := INCREMENTAR_SECTOR (nd.regiones.vector (indice_derecha).final);
                  nd.regiones.vector (indice_derecha).principio_ascendente := not nd.regiones.vector (indice_derecha).final_ascendente;

               else

                  indice_auxiliar := indice_derecha;

                  indice_derecha := nd.regiones.longitud;
                  nd.regiones.longitud := nd.regiones.longitud + 1;
                  -- region_derecha:=nd.regiones.vector(indice_derecha)'Access;
                  nd.regiones.vector (indice_derecha).descartada := FALSO;

                  nd.regiones.vector (indice_derecha).principio := INCREMENTAR_SECTOR (nd.regiones.vector (indice_auxiliar).final);
                  nd.regiones.vector (indice_derecha).principio_ascendente := not nd.regiones.vector (indice_auxiliar).final_ascendente;

               end if;

               SiguienteDiscontinuidad (nd, nd.regiones.vector (indice_derecha).principio, DERECHA, nd.regiones.vector (indice_derecha).final, nd.regiones.vector (indice_derecha).final_ascendente);

               declare
                  ret : Boolean;
               begin
                  ObjetivoAlcanzableSPARK (nd, nd.regiones.vector (indice_derecha), DIRECCION_DISCONTINUIDAD_INICIAL, ret);
                  if ret then
                     if nd.regiones.vector (indice_izquierda).principio_ascendente or else nd.regiones.vector (indice_izquierda).final_ascendente then
                        nd.region := indice_derecha;
                        return;
                     end if;

                     if indice_izquierda > indice_derecha then
                        nd.regiones.longitud := nd.regiones.longitud - 1;
                        nd.region := indice_derecha;
                        return;
                     end if;

                     nd.regiones.vector (indice_izquierda) := nd.regiones.vector (indice_derecha);
                     nd.regiones.longitud := nd.regiones.longitud - 1;
                     nd.region := indice_izquierda;
                     return;
                  end if;
               end;

            end if;

         end if;

         exit when not (distancia_izquierda < SECTORES / 2 or else distancia_derecha < SECTORES / 2);
      end loop;

      -- *region_izquierda == *region_derecha (al menos los campos que determinan la region) y son las dos �ltimas del vector.
      nd.regiones.longitud := nd.regiones.longitud - 1;
      nd.regiones.vector (nd.regiones.longitud - 1).descartada := VERDADERO;
   end SeleccionarRegionSPARK;

   -- IterarND / ConstruirDR

   procedure ConstruirDR (nd : in out TInfoND) is
   begin
      for i in SECTOR_ID'Range loop
         if nd.d (i).r < 0.0 then
            nd.dr (i) := -1.0;
         else
            nd.dr (i) := nd.d (i).r - robot.E (i);
         end if;
      end loop;
   end ConstruirDR;

   ----------------------------------------------------------------------------

   -- IterarND / control_angulo / ObtenerObstaculos / ActualizarMinimo

   procedure ActualizarMinimo (sector_minimo : in out SECTOR_ID_Optional;
                              valor_minimo : in out Float;
                              sector : SECTOR_ID;
                              valor : Float) is
   begin
      if sector_minimo = -1 or else valor < valor_minimo then
         sector_minimo := sector;
         valor_minimo := valor;
      end if;
   end ActualizarMinimo;

   -- IterarND / control_angulo / ObtenerObstaculos

   procedure ObtenerObstaculos (nd : in out TInfoND;
                               beta : Float)
   with
     Post => nd.region = nd.region'Old;

   procedure ObtenerObstaculos (nd : in out TInfoND;
                               beta : Float) is
      p : TCoordenadas;
      pp : TCoordenadasPolares;
      angulo : Float;

      min_izq : Float := Float'Last;
      min_der : Float := Float'Last;

      -- Buscamos todos los obst�culos que est�n dentro de la distancia de seguridad y nos quedamos
      -- con el m�s cercano por la izquierda y el m�s cercano por la derecha.
      -- El obst�culo m�s cercano es el de menor dr/ds.
      -- Un obst�culo es por la izquierda si nos limita el giro a la izquierda (nos obliga a
      -- rectificar el �ngulo de partida hacia la derecha para esquivarlo).
   begin

      ConstruirCoordenadasCxy (p, robot.Dimensiones.Rear, robot.Dimensiones.Left);
      ConstruirCoordenadasPcC (pp, p);

      nd.obstaculo_izquierda := -1;
      nd.obstaculo_derecha := -1;
      for i in SECTOR_ID'Range loop
         pragma Loop_Invariant (nd.region = nd.region'Loop_Entry);

         if nd.dr (i) >= 0.0 and then nd.dr (i) <= robot.ds (i) then
            angulo := nd.d (i).a;
            --       if (AnguloNormalizado(angulo-beta)>=0) then --
            if angulo >= beta then
               -- ACTUALIZAR_OBSTACULO_IZQUIERDA
               ActualizarMinimo (nd.obstaculo_izquierda, min_izq, i, nd.dr (i) / robot.ds (i));
            else
               -- ACTUALIZAR_OBSTACULO_DERECHA
               ActualizarMinimo (nd.obstaculo_derecha, min_der, i, nd.dr (i) / robot.ds (i));
            end if;

         end if;
      end loop;

   end ObtenerObstaculos;

   -- IterarND / control_angulo / solHSGR

   function solHSGR (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1;

   function solHSGR (nd : TInfoND) return Float is
   begin
      return nd.regiones.vector (nd.region).direccion_angulo;
   end solHSGR;

   -- IterarND / control_angulo / solHSNR

   function solHSNR (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1;

   function solHSNR (nd : TInfoND) return Float is
      region : constant TRegion := nd.regiones.vector (nd.region);
      final : Integer := region.final;
   begin
      if region.principio > region.final then
         final := final + SECTORES;
      end if;

      return sector2angulo (((region.principio + final) / 2) mod SECTORES);
   end solHSNR;

   -- IterarND / control_angulo / solHSWR

   function solHSWR (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1 and then
     (nd.regiones.vector (nd.region).principio /= -1 and then
      nd.regiones.vector (nd.region).final /= -1);

   function solHSWR (nd : TInfoND) return Float is
      region : constant TRegion := nd.regiones.vector (nd.region);
   begin

      if region.direccion_tipo = DIRECCION_DISCONTINUIDAD_INICIAL then
         return (nd.d (DECREMENTAR_SECTOR (region.principio)).a
                - Arctan ((robot.discontinuidad / 2.0 + robot.ds (SECTORES / 2)), nd.d (DECREMENTAR_SECTOR (region.principio)).r));
      else
         return (nd.d (INCREMENTAR_SECTOR (region.final)).a
                + Arctan ((robot.discontinuidad / 2.0 + robot.ds (SECTORES / 2)), nd.d (INCREMENTAR_SECTOR (region.final)).r));
      end if;
   end solHSWR;

   -- IterarND / control_angulo / solLS1

   function solLS1 (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1 and then
     (nd.obstaculo_derecha /= -1 or else nd.obstaculo_izquierda /= -1);

   function solLS1 (nd : TInfoND) return Float is
      region : constant TRegion := nd.regiones.vector (nd.region);
      anguloPrueba : Float;
      angulo_parcial, dist_obs_dsegur, angulo_cota : Float;
      final : Integer := region.final;
   begin
      -- float angulo_objetivo=nd.regiones.vector(nd.region).direccion_angulo;

      if region.principio > region.final then
         final := final + SECTORES;
      end if;

      if final - nd.regiones.vector (nd.region).principio > SECTORES / 4 then
         if region.direccion_tipo = DIRECCION_DISCONTINUIDAD_INICIAL then
            angulo_parcial := nd.d (DECREMENTAR_SECTOR (region.principio)).a
              - Arctan ((robot.discontinuidad / 2.0 + robot.ds (SECTORES / 2)), nd.d (DECREMENTAR_SECTOR (region.principio)).r);
         else
            angulo_parcial := nd.d (INCREMENTAR_SECTOR (region.final)).a
              + Arctan ((robot.discontinuidad / 2.0 + robot.ds (SECTORES / 2)), nd.d (INCREMENTAR_SECTOR (region.final)).r);
         end if;
      else
         angulo_parcial := sector2angulo (((region.principio + final) / 2) mod SECTORES);
      end if;

      if nd.obstaculo_izquierda /= -1 then
         angulo_cota := AnguloNormalizado (nd.d (nd.obstaculo_izquierda).a + Pi - angulo_parcial) + angulo_parcial;
         dist_obs_dsegur := nd.dr (nd.obstaculo_izquierda) / robot.ds (nd.obstaculo_izquierda);
      else
         angulo_cota := AnguloNormalizado (nd.d (nd.obstaculo_derecha).a + Pi - angulo_parcial) + angulo_parcial;
         dist_obs_dsegur := nd.dr (nd.obstaculo_derecha) / robot.ds (nd.obstaculo_derecha);
      end if;

      -- Codigo Osuna
      -- return AnguloNormalizado(angulo_parcial * dist_obs_dsegur  + angulo_cota * (1-dist_obs_dsegur));

      -- Codigo Minguez
      anguloPrueba := angulo_parcial * dist_obs_dsegur  + angulo_cota * (1.0 - dist_obs_dsegur);

      if anguloPrueba > Pi then
         anguloPrueba := (Pi - 0.01);
      elsif anguloPrueba < -Pi then
         anguloPrueba := -(Pi + 0.01);
      end if;

      return AnguloNormalizado (anguloPrueba);

   end solLS1;

   -- IterarND / control_angulo / solLSG

   function solLSG (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1 and then
     (nd.obstaculo_derecha /= -1 or else nd.obstaculo_izquierda /= -1);

   function solLSG (nd : TInfoND) return Float is
      angulo_parcial, dist_obs_dsegur, angulo_cota, anguloPrueba : Float;
   begin

      angulo_parcial := nd.regiones.vector (nd.region).direccion_angulo;

      if nd.obstaculo_izquierda /= -1 then
         angulo_cota := AnguloNormalizado (nd.d (nd.obstaculo_izquierda).a + Pi - angulo_parcial) + angulo_parcial;
         dist_obs_dsegur := nd.dr (nd.obstaculo_izquierda) / robot.ds (nd.obstaculo_izquierda);
      else
         angulo_cota := AnguloNormalizado (nd.d (nd.obstaculo_derecha).a + Pi - angulo_parcial) + angulo_parcial;
         dist_obs_dsegur := nd.dr (nd.obstaculo_derecha) / robot.ds (nd.obstaculo_derecha);
      end if;

      -- Codigo Osuna
      -- return AnguloNormalizado(angulo_parcial * dist_obs_dsegur  + angulo_cota * (1-dist_obs_dsegur));

      anguloPrueba := angulo_parcial * dist_obs_dsegur  + angulo_cota * (1.0 - dist_obs_dsegur);

      -- Codigo Minguez
      if anguloPrueba > Pi then
         anguloPrueba := (Pi - 0.01);
      elsif anguloPrueba < -Pi then
         anguloPrueba := -(Pi + 0.01);
      end if;

      return AnguloNormalizado (anguloPrueba);
   end solLSG;

   -- IterarND / control_angulo / solLS2

   function solLS2 (nd : TInfoND) return Float
   with
     Pre => nd.region /= -1 and then
     nd.obstaculo_izquierda /= -1 and then
     nd.obstaculo_derecha /= -1;

   function solLS2 (nd : TInfoND) return Float is
      ci : constant Float := nd.dr (nd.obstaculo_izquierda) / robot.ds (nd.obstaculo_izquierda);
      cd : constant Float := nd.dr (nd.obstaculo_derecha) / robot.ds (nd.obstaculo_derecha);
      -- �ngulos cota izquierdo y derecho.
      ad : constant := Pi / 2.0;
      ai : constant := -Pi / 2.0;
      ang_par : constant Float := nd.regiones.vector (nd.region).direccion_angulo;
   begin

      --  /*
      --    ad = AnguloNormalizado(nd.d(nd.obstaculo_derecha).a+PI-ang_par)+ang_par;
      --
      --    ai = AnguloNormalizado(nd.d(nd.obstaculo_izquierda).a-PI-ang_par)+ang_par;
      --
      --    return AnguloNormalizado((ad+ai)/2.0F+(ci-cd)/(ci+cd)*(ad-ai)/2.0F);
      --  */

      if ci <= cd then
         return AnguloNormalizado (ang_par + (ci - cd) / (ci + cd) * (ang_par - ai));
      else
         return AnguloNormalizado (ang_par + (ci - cd) / (ci + cd) * (ad - ang_par));
      end if;
   end solLS2;

   -- IterarND / control_angulo

   procedure control_angulo (nd : in out TInfoND)
   with
     Pre => nd.region /= -1;

   procedure control_angulo (nd : in out TInfoND) is
      -- C�lculo del �ngulo de movimiento en funci�n de la regi�n escogida para el movimiento del robot, la situaci�n del objetivo y,
      -- en su caso, la distancia a los obst�culos m�s pr�ximos.
      region : constant TRegion := nd.regiones.vector (nd.region);
      final : Integer := region.final;
   begin

      if region.principio > region.final then
         final := final + SECTORES;
      end if;

      ObtenerObstaculos (nd, nd.regiones.vector (nd.region).direccion_angulo);

      if nd.obstaculo_izquierda = -1 and then nd.obstaculo_derecha = -1 then
         if region.direccion_tipo = DIRECCION_OBJETIVO then
            nd.situacion := HSGR;
            nd.angulosin := solHSGR (nd);
            nd.angulo := nd.angulosin;
         elsif final - nd.regiones.vector (nd.region).principio > SECTORES / 4 then
            nd.situacion := HSWR;

            pragma Assume (nd.region /= -1 and then
                             (nd.regiones.vector (nd.region).principio /= -1 and then
                                nd.regiones.vector (nd.region).final /= -1));

            nd.angulosin := solHSWR (nd);

            nd.angulo := nd.angulosin;
         else
            nd.situacion := HSNR;
            nd.angulosin := solHSNR (nd);
            nd.angulo := nd.angulosin;
         end if;
      else
         if nd.obstaculo_izquierda /= -1 and then nd.obstaculo_derecha /= -1 then
            nd.situacion := LS2;
            nd.angulo := solLS2 (nd);
            nd.angulosin := nd.angulo;
         elsif region.direccion_tipo = DIRECCION_OBJETIVO then
            nd.situacion := LSG;
            nd.angulo := solLSG (nd);
            nd.angulosin := nd.angulo;
         else
            nd.situacion := LS1;
            nd.angulo := solLS1 (nd);
            nd.angulosin := nd.angulo;
         end if;
      end if;

      AplicarCotas (nd.angulo, -Pi / 2.0, Pi / 2.0);
   end control_angulo;

   -----------------------------------------------------------
   --
   -- IterarND / control_velocidad

   function control_velocidad (nd : TInfoND) return Float is
      ci, cd : Float;
   begin
      -- Velocidad lineal del robot.

      if nd.obstaculo_izquierda /= -1 then
         ci := nd.dr (nd.obstaculo_izquierda) / robot.ds (nd.obstaculo_izquierda);
      else
         ci := 1.0;
      end if;

      if nd.obstaculo_derecha /= -1 then
         cd := nd.dr (nd.obstaculo_derecha) / robot.ds (nd.obstaculo_derecha);
      else
         cd := 1.0;
      end if;

      return robot.velocidad_lineal_maxima * MINIMO (ci, cd);
   end control_velocidad;

   ----------------------------------------------------------------------------

   -- Cutting / GenerarMovimientoFicticio

   function GenerarMovimientoFicticio (nd : TInfoND) return TVelocities
   is
--      pragma Unreferenced(angulo);

      -- Coeficiente de distancia por la izquierda.
      ci : constant Float :=
        (if nd.obstaculo_izquierda /= -1 then
         nd.dr (nd.obstaculo_izquierda) / robot.ds (nd.obstaculo_izquierda) else
         1.0);

      -- Coeficiente de distancia por la derecha.
      cd : constant Float :=
        (if nd.obstaculo_derecha /= -1 then
         nd.dr (nd.obstaculo_derecha) / robot.ds (nd.obstaculo_derecha) else
         1.0);

      cvmax : constant Float := MAXIMO (0.2, MINIMO (ci, cd));

      velocidades : TVelocities;
   begin
      velocidades.v := robot.velocidad_lineal_maxima * cvmax * Cos (nd.angulo); -- Calculada en SR2C.
      velocidades.w := robot.velocidad_angular_maxima * cvmax * Sin (nd.angulo); -- Calculada en SR2C.

      --  fprintf(depuracion,"%d: <a,ci,cd,cvmax,v,w>=<%f,%f,%f,%f,%f,%f>\n",++iteracion,nd.angulo,ci,cd,cvmax,velocidades.v,velocidades.w);
      AplicarCotas (velocidades.v, 0.0, robot.velocidad_lineal_maxima);
      AplicarCotas (velocidades.w, -robot.velocidad_angular_maxima, robot.velocidad_angular_maxima);
      --     /*
      --
      --       #define FMAX robot.aceleracion_lineal_maxima
      --
      --       TCoordenadas F;
      --
      --       ConstruirCoordenadasCra(&F,FMAX*nd.velocidad/robot.velocidad_lineal_maxima,angulo);
      --
      --       velocidades.v=robot.H(0)(0)*nd.velocidades.v+robot.G(0)(0)*F.x;
      --       velocidades.w=robot.H(1)(1)*nd.velocidades.w+robot.G(1)(1)*F.y;
      --
      --       #undef FMAX
      --     */
      return velocidades;
   end GenerarMovimientoFicticio;

   --
   -- GiroBrusco

   procedure GiroBrusco (nd : TInfoND;
                        velocidades : in out TVelocities) is
      --# hide GiroBrusco;
      esquina : TCoordenadasPolares;
      derecha, izquierda : Boolean;
   begin
      ConstruirCoordenadasPxy (esquina, robot.Dimensiones.Front, robot.Dimensiones.Left);

      if nd.obstaculo_derecha /= -1 then
         derecha := abs (nd.d (nd.obstaculo_derecha).a) <= esquina.a and then nd.d (nd.obstaculo_derecha).r <= esquina.r + robot.enlarge;
      else
         derecha := false;
      end if;
      if nd.obstaculo_izquierda /= -1 then
         izquierda := abs (nd.d (nd.obstaculo_izquierda).a) <= esquina.a and then nd.d (nd.obstaculo_izquierda).r <= esquina.r + robot.enlarge;
      else
         izquierda := false;
      end if;

      if derecha and then izquierda then
         velocidades.w := 0.0;
         --    printf("giro brusco central\n");
         return;
      end if;

      if derecha or else izquierda then
         velocidades.v := 0.0;
         --    printf("giro brusco lateral\n");
      end if;
   end GiroBrusco;

   -- Cutting / ObtenerSituacionCutting

   function ObtenerSituacionCutting (nd : TInfoND)
--                                    w : in Float)
                                    return CUTTING_TYPE
   is
--      pragma Unreferenced(w);
      p : TCoordenadas;
      resultado : CUTTING_TYPE := CUTTING_NINGUNO;
      obstaculo_izquierda : Boolean := False;
      obstaculo_derecha : Boolean := False;
      i : SECTOR_ID := SECTOR_ID'First;
   begin

      loop
         pragma Loop_Invariant (i < SECTOR_ID'Last);
         pragma Loop_Variant (Increases => i);
         if (nd.d (i).a < -Pi / 2.0 or else nd.d (i).a > Pi / 2.0) and then nd.d (i).r >= 0.0 and then nd.dr (i) <= robot.enlarge / 2.0 then

            ConstruirCoordenadasCP (p, nd.d (i));

            if p.y >= robot.Dimensiones.Left then -- Obst�culo a la izquierda.

               if obstaculo_derecha then
                  return CUTTING_AMBOS;
               end if;

               obstaculo_izquierda := True;
               resultado := CUTTING_IZQUIERDA;

            elsif p.y <= robot.Dimensiones.Right then -- Obst�culo a la derecha.

               if obstaculo_izquierda then
                  return CUTTING_AMBOS;
               end if;

               obstaculo_derecha := True;
               resultado := CUTTING_DERECHA;

            elsif p.x <= robot.Dimensiones.Rear then -- Obst�culo detr�s.
               return CUTTING_AMBOS;
            end if;
         end if;

         i := i + 1;
         if i = SECTORES / 4 + 1 then
            i := 3 * SECTORES / 4;
         end if;

         exit when i = SECTOR_ID'Last;
      end loop;

      return resultado;
   end ObtenerSituacionCutting;

   -- Cutting / AnguloSinRotacion

   procedure AnguloSinRotacion (nd : TInfoND;
                              velocidades : in out TVelocities;
                              angulo : out Float)
   is
      F : TCoordenadas;
   begin
      if robot.aceleracion_angular_maxima * robot.T < abs (nd.velocidades.w) then
         if nd.velocidades.w > 0.0 then
            velocidades.w := nd.velocidades.w - robot.aceleracion_angular_maxima * robot.T;
         else
            velocidades.w := nd.velocidades.w + robot.aceleracion_angular_maxima * robot.T;
         end if;
         if robot.aceleracion_lineal_maxima * robot.T < nd.velocidades.v then
            velocidades.v := nd.velocidades.v - robot.aceleracion_lineal_maxima * robot.T;
         else
            velocidades.v := 0.0;
         end if;
      else
         velocidades.v := nd.velocidad;
         velocidades.w := 0.0;
      end if;

      F.x := (velocidades.v - robot.H (0, 0) * nd.velocidades.v) / robot.G (0, 0);
      F.y := (velocidades.w - robot.H (1, 1) * nd.velocidades.w) / robot.G (1, 1);
      angulo := ARCOTANGENTE (F.x, F.y);
   end AnguloSinRotacion;

   -- Cutting

   procedure Cutting (nd : in out TInfoND;
                     velocidades : in out TVelocities) is
   begin
      nd.cutting := ObtenerSituacionCutting (nd); --,velocidades.w);

      if (nd.cutting = CUTTING_IZQUIERDA and then velocidades.w >= 0.0)
        or else
          (nd.cutting = CUTTING_DERECHA and then velocidades.w <= 0.0)
        or else
          (nd.cutting = CUTTING_NINGUNO)
      then
         return;
      end if;

      AnguloSinRotacion (nd, velocidades, nd.angulo);
   end Cutting;

   --
   --  -- ----------------------------------------------------------------------------
   --
   --  -- IterarND / GenerarMovimiento
   --
   --  /* Unused
   --  static void GenerarMovimiento(TInfoND *nd,TVelocities *velocidades) {
   --    #define FMAX robot.aceleracion_lineal_maxima
   --
   --    TCoordenadas F;
   --
   --    ConstruirCoordenadasCra(&F,FMAX*nd.velocidad/robot.velocidad_lineal_maxima,nd.angulo);
   --
   --    velocidades.v=robot.H(0)(0)*nd.velocidades.v+robot.G(0)(0)*F.x;
   --    velocidades.w=robot.H(1)(1)*nd.velocidades.w+robot.G(1)(1)*F.y;
   --
   --  --   printf("v= %f \n w= %f \n",velocidades.v,velocidades.w);
   --
   --
   --    AplicarCotas(&(velocidades.v),0.0F,robot.velocidad_lineal_maxima);
   --    AplicarCotas(&(velocidades.w),-robot.velocidad_angular_maxima,robot.velocidad_angular_maxima);
   --
   --    #undef FMAX
   --  }
   --  */
   --
   -- ----------------------------------------------------------------------------

   -- IterarND

   function IterarND (objetivo : TCoordenadas;
                     goal_tol : Float;
                     movimiento : TInfoMovimiento;
                     mapa : TInfoEntorno
                     --,void *informacion
                    ) return TVelocities_Option is
      nd : TInfoND;
      velocidades : TVelocities;
   begin
      -- Devuelve NULL si se requiere una parada de emergencia o si no encuentra una regi�n por la que hacer avanzar el robot.
      -- Devuelve un puntero a (0.0F,0.0F) si se ha alcanzado el objetivo.

      -- Valgrind says that some of the values in this nd structure are
      -- uninitialized when it's accessed in ObtenerSituacionCutting(), so I'm
      -- zeroing it here.  - BPG
      -- memset(&nd, 0, sizeof(TInfoND));

      -- depuracion=fopen("depuracion.txt","at");
      -- Tratamiento de los par�metros "objetivo" y "movimiento".

      nd.objetivo.c0 := objetivo;
      nd.SR1 := movimiento.SR1;
      nd.velocidades := movimiento.velocidades;

      nd.objetivo.c1 := nd.objetivo.c0;
      TRANSFORMACION01 (nd.SR1, nd.objetivo.c1);

      ConstruirCoordenadasPC (nd.objetivo.p1, nd.objetivo.c1);

      nd.objetivo.s := ObtenerSectorP (nd.objetivo.p1);

      -- Sectorizaci�n del mapa.

      SectorizarMapa (mapa, nd);

      -- Evaluaci�n de la necesidad de una parada de emergencia.
      -- Solo en el caso de robot rectangular
      if robot.geometriaRect then
         if ParadaEmergencia (nd) then
            -- printf("ND . Parada Emergencia\n");
            -- return 0;
            return (Opt => O_NONE);
         end if;
      end if;

      -- Selecci�n de la regi�n por la cual avanzar�Eel robot.

      SeleccionarRegionSPARK (nd);
      if nd.region < 0 then
         -- printf("ND . No encuentra region\n");
         return (Opt => O_NONE);
      end if;

      -- Construcci�n de la distancia desde el per�etro del robot al obst�culo m�s cercano en cada sector.

      ConstruirDR (nd);

      -- Deteccion de fin de trayecto. -- Despu�s de considerar la necesidad de una parada de emergencia.
      -- Caso geometria rectangular
      if robot.geometriaRect then
         -- Replaced this check with the user-specified goal tolerance - BPG
         --  /*
         --  -- Cuadrado
         --  if ((nd.objetivo.c1.x>=robot.Dimensiones(0)) and (nd.objetivo.c1.x<=robot.Dimensiones(2)) and
         --  (nd.objetivo.c1.y>=robot.Dimensiones(3)) and (nd.objetivo.c1.y<=robot.Dimensiones(1))) {
         --  */
         if  Hypot (objetivo.x - movimiento.SR1.posicion.x,
                    objetivo.y - movimiento.SR1.posicion.y) < goal_tol
         then
            -- Ya hemos llegado.
            velocidades.v := 0.0;
            velocidades.w := 0.0;
            return (Opt => O_SOME, value => velocidades);
         end if;
      elsif  Hypot (nd.objetivo.c1.x, nd.objetivo.c1.y) < robot.R then
         -- Redondo
         velocidades.v := 0.0;
         velocidades.w := 0.0;
         return (Opt => O_SOME, value => velocidades);
      end if;

      -- C�lculo del movimiento del robot.
      control_angulo (nd); -- Obtenci�n de la direcci�n de movimiento.
      nd.velocidad := control_velocidad (nd); -- Obtenci�n de la velocidad de movimiento.
      --  if (nd.velocidad<0.05F)
      --    nd.velocidad=0.05F;
      nd.velocidad := robot.velocidad_lineal_maxima;
      -- Hasta aqui es el ND standart

      -- En funcion del tipo de robot.
      if robot.holonomo then -- ya se han aplicado cotas al angulo
         --    printf("Movimiento Holonomo\n");
         --    velocidades.v= nd.velocidad*fabs(DistanciaAngular(fabs(nd.angulo),Pi/2))/(Pi/2);
         velocidades.v := nd.velocidad * abs (AmplitudAnguloNoOrientado (abs (nd.angulo), Pi / 2.0)) / (Pi / 2.0);
         -- /*     velocidades.v= nd.velocidad; */
         -- /*     velocidades.w=0.4; */
         --    velocidades.w= (Pi/2-DistanciaAngular(fabs(nd.angulo),Pi/2))/(Pi/2)*robot.velocidad_angular_maxima;
         velocidades.w := nd.angulo / (Pi / 2.0) * robot.velocidad_angular_maxima;
         velocidades.v_theta := nd.angulo;
         -- /*    printf("w= %f dist=%f\n",velocidades.w,DistanciaAngular(fabs(nd.angulo),Pi/2)); */
         -- /*    if (nd.angulo<0) then
         --   velocidades.w:=-velocidades.w;
         -- printf("vdsd�f\n");
         -- end if;
         -- */

      else
         velocidades.v_theta := 0.0;
         --   printf("Movimiento No Holonomo\n");
         -- Calculo del movimiento Generador de movimientos
         --**/printf("<Vnd,And>=<%f,%f>\n",nd.velocidad,nd.angulo);
         velocidades := GenerarMovimientoFicticio (nd);
         velocidades.v_theta := 0.0;
         --**/printf("<Vr,Wr>=<%f,%f>\n",velocidades.v,velocidades.w);

         -- Aplicar correcciones al movimiento calculado.
         if robot.geometriaRect then
            -- Cuadrado
            GiroBrusco (nd, velocidades);
            -- /* printf("Entra en cutting %d\n",robot.geometriaRect); */
            Cutting (nd, velocidades); -- Evitar colisi�n en zona posterior.
         end if;
      end if;

      AplicarCotas (velocidades.v,
                   0.0,
                   robot.velocidad_lineal_maxima);
      AplicarCotas (velocidades.w,
                   -robot.velocidad_angular_maxima,
                   robot.velocidad_angular_maxima);

      -- /*     printf("w = %f\n",velocidades.w);   */

      -- Copia (si se requiere) de la informaci�n interna de ND para que quede accesible desde el exterior.

      -- if (informacion)
      --  *(TInfoND*)informacion=nd;

      -- Devoluci�n de resultados.

      -- fclose(depuracion);
      return (Opt => O_SOME, value => velocidades);
   end IterarND;

end nd;
