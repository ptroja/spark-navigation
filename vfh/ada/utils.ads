package Utils is

   function DTOR (D : Float) return Float;

   function Hypot (X, Y : Float) return Float;
   pragma Import(C, Hypot, "hypotf");

end Utils;
