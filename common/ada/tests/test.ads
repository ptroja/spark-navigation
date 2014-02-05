with Ada.Containers.Formal_Vectors;

package Test is

   use Ada.Containers;

   -- Formal vector type.
   function Integer_Eq(X,Y : Integer) return Boolean is (X = Y);

   package Integer_Vector is new Ada.Containers.Formal_Vectors(Index_Type   => Natural,
                                                               Element_Type => Integer,
                                                               "=" => Integer_Eq);

   -- Record type with formal vector.
   type Rec is
      record
         Vec : Integer_Vector.Vector(-10);
         A : Integer;
      end record;

   function Foo(This: Rec; X : Integer) return Integer;
end;
