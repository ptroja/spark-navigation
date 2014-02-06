package Utils is

   function DTOR (D : Float) return Float;

   function rint (X : Float) return Integer is
     (Integer (Float'Unbiased_Rounding (X)));

   function Hypot (X, Y : Float) return Float
   with
     Post => Hypot'Result >= 0.0;
   pragma Import (C, Hypot, "hypotf");

end Utils;
