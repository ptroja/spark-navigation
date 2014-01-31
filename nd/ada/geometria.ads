with Formal.Numerics;
with Formal.Numerics.Elementary_Functions;

use Formal.Numerics;
use Formal.Numerics.Elementary_Functions;

package geometria is

   function CUADRADO(x : Unbounded_Float) return Unbounded_Float is (X*X);

   -- Note: We must swap the arguments.
   function ARCOTANGENTE(x,y : Unbounded_Float) return Unbounded_Float
     is (Arctan (y,x))
   with
     Pre => X /= 0.0 or else Y /= 0.0;

   function ARCOCOSENO(x,r : Unbounded_Float) return Unbounded_Float
     is (Arccos (x / r))
   with
     Pre => r /= 0.0 and then abs(x/r) <= 1.0;

   function ARCOSENO(y,r : Unbounded_Float) return Unbounded_Float
     is (Arcsin (y / r))
   with
     Pre => r /= 0.0 and then abs(y/r) <= 1.0;

   procedure AplicarCotas(n : in out Unbounded_Float;
                          i : Unbounded_Float;
                          s : Unbounded_Float)
   with
     Pre => i <= s,
     Post => i <= n and then n <= s;

   function Hypot(x,y : Unbounded_Float) return Unbounded_Float
   with
     Post => (if x /= 0.0 or else y /= 0.0 then Hypot'Result > 0.0 else Hypot'Result >= 0.0);
   pragma Import(C, Hypot, "hypotf");

   --
   -- GENERIC TYPES
   --

   -- Cartesian coordinates.
   type TCoordenadas is
      record
         x,y : Unbounded_Float;
      end record;

   -- Coordenadas polares. Espacio real.
   type TCoordenadasPolares is
      record
         r : Unbounded_Float; -- Radio.
         a : Unbounded_Float; -- Angle.
      end record;

   -- System of reference
   type TSR is
      record
         posicion : TCoordenadas;
         orientacion : Unbounded_Float;
      end record;

   --
   -- Declaraci�n de tipos y macros relacionadas.
   --

   -- Coordenadas cartesianas. Espacio real (y pantalla).
   function DISTANCIA_CUADRADO2(p, q : TCoordenadas) return Unbounded_Float
   with
     Post => DISTANCIA_CUADRADO2'Result >= 0.0;

   --
   -- Construcci�n de coordenadas.
   --
   procedure ConstruirCoordenadasCP(p : out TCoordenadas; q : TCoordenadasPolares);

   procedure ConstruirCoordenadasCxy(p : out TCoordenadas; x,y : Unbounded_Float)
   with
     Post => p.x = x and then p.y = y;

   procedure ConstruirCoordenadasCra(p : out TCoordenadas; r : Unbounded_Float; a : Unbounded_Float);

   procedure ConstruirCoordenadasPC(p : out TCoordenadasPolares; q : TCoordenadas)
   with
     Pre => q.x /= 0.0 or else q.y /= 0.0,
     Post => p.r > 0.0 and then p.a in -Pi .. Pi;

   procedure ConstruirCoordenadasPxy(p : out TCoordenadasPolares; x,y : Unbounded_Float)
   with
     Pre => x /= 0.0 or else y /= 0.0,
     Post => p.r > 0.0 and then p.a in -Pi .. Pi;

   procedure ConstruirCoordenadasPra(p : out TCoordenadasPolares; r : Unbounded_Float ; a : Unbounded_Float);

   -- Paso de cartesianas a polares, pero con el m�dulo al cuadrado.
   procedure ConstruirCoordenadasPcC(p : out TCoordenadasPolares; q : TCoordenadas)
   with
     Pre => q.x /= 0.0 or else q.y /= 0.0,
     Post => p.r > 0.0;

   --
   -- Suma y resta de coordenadas.                                              */
   --
   procedure SumarCoordenadasCxy(p : in out TCoordenadas; x, y : Unbounded_Float)
   with
     Post => p.x = p.x'Old + x and then p.y = p.y'Old + y;

   procedure SumarCoordenadasCxyC(p : TCoordenadas; x,y : Unbounded_Float; q : out TCoordenadas)
   with
     Post => q.x = p.x + x and then q.y = p.y + y;

   procedure SumarCoordenadasCra(p : in out TCoordenadas; r : Unbounded_Float; a : Unbounded_Float);
   procedure SumarCoordenadasCraC(p : TCoordenadas; r : Unbounded_Float; a : Unbounded_Float; q : out TCoordenadas);

   --
   -- Transformaciones entre sistemas de coordenadas.                           */
   --

   -- Transformaciones directas.
   procedure TransformacionDirecta(SR : TSR; p : in out TCoordenadas);

   procedure TRANSFORMACION01(SR1 : TSR; p : in out TCoordenadas);
   procedure TRANSFORMACION12(SR2 : TSR; p : in out TCoordenadas);
   procedure TRANSFORMACION23(SR3 : TSR; p : in out TCoordenadas);

   procedure TRANSFORMACION02(SR1,SR2 : TSR; p : in out TCoordenadas);

   -- Transformaciones inversas.

   procedure TransformacionInversa(SR : TSR; p : in out TCoordenadas);

   --  #define TRANSFORMACION32(SR3,p) TransformacionInversa(SR3,p);
   --  #define TRANSFORMACION21(SR2,p) TransformacionInversa(SR2,p);
   --  #define TRANSFORMACION10(SR1,p) TransformacionInversa(SR1,p);
   --
   --  #define TRANSFORMACION20(SR2,SR1,p) \
   --      { \
   --        TRANSFORMACION21(SR2,p) \
   --        TRANSFORMACION10(SR1,p) \
   --      }

   --  // Transformaciones mixtas.
   --
   --  #define TRANSFORMACION101(SR1a,SR1b,p) \
   --      { \
   --        TRANSFORMACION10(SR1a,p) \
   --        TRANSFORMACION01(SR1b,p) \
   --      }

   --
   -- �ngulos e intervalos de �ngulos.                                          */
   --

   function AnguloNormalizado(angulo : Unbounded_Float) return Unbounded_Float;

   -- Esta funci�n devuelve 1 si el �ngulo est� entre los l�mites; 0 en caso contrario.
   -- Todos los par�metros deben pertenecer al intervalo (-PI,PI].
   function AnguloPerteneceIntervaloOrientadoCerrado(angulo, limite1, limite2 : Unbounded_Float) return Boolean;

   -- Devuelve la bisectriz del �ngulo de "limite1" a "limite2" en sentido contrario a las agujas del reloj.
   function BisectrizAnguloOrientado(limite1, limite2 : Unbounded_Float) return Unbounded_Float;

   -- Devuelve la bisectriz del menor �ngulo formado por "limite1" y "limite2", ya sea en el sentido de las agujas del reloj o en el opuesto.
   function BisectrizAnguloNoOrientado(limite1, limite2 : Unbounded_Float) return Unbounded_Float;

   -- Devuelve la amplitud del �ngulo de "limite1" a "limite2" en sentido contrario a las agujas del reloj.
   function AmplitudAnguloOrientado(limite1, limite2 : Unbounded_Float) return Unbounded_Float;

   -- Devuelve la amplitud del menor �ngulo formado por "limite1" y "limite2", ya sea en el sentido de las agujas del reloj o en el opuesto.
   function AmplitudAnguloNoOrientado(limite1, limite2 : Unbounded_Float) return Unbounded_Float;

   --
   -- Cortes entre dos segmentos, uno de los cuales tiene como uno de sus       */
   -- extremos el origen.                                                       */
   --

   -- Mediante su aplicaci�n reiterada obtenemos el m�s pr�ximo de entre los puntos de corte de un
   -- grupo de segmentos con una direcci�n determinada.
   -- "p1" y "p2" son los extremos de un segmento.
   -- "angulo" es la direcci�n de corte (desde el origen).
   -- "distancia" es la menor distancia obtenida hasta el momento.
   procedure MinimaDistanciaCuadradoCorte(pp1, pp2 : TCoordenadasPolares; angulo : Unbounded_Float; distancia : in out Unbounded_Float);

end geometria;
