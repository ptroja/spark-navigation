with Ada.Numerics;

package body Utils is

   function DTOR (D : Float) return Float is
   begin
      return (D * Ada.Numerics.Pi / 180.0);
   end DTOR;

end Utils;
