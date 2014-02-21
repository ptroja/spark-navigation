with Ada.Numerics.Generic_Elementary_Functions;

package body Utils is
   package Elem_Fun is new ADA.Numerics.Generic_Elementary_Functions(Float);

   function DTOR (D : Float) return Float is
   begin
      return (D * Pi / 180.0);
   end DTOR;

   function rint (X : Float) return Integer is
   begin
     return Integer (Float'Unbiased_Rounding (X));
   end rint;

   function Hypot (X, Y : Float) return Float is
   begin
     return Elem_Fun.sqrt(X*X + Y*Y);
   end Hypot;
end Utils;
