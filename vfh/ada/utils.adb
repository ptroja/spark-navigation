with Ada.Numerics;

package body utils is

   function DTOR (D : Float) return Float is
   begin
      return (D * Ada.Numerics.Pi / 180.0);
   end;

end;
