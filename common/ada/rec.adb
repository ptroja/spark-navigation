procedure Rec is

   type Foo is
      record
         X : Integer;
         Y : Integer;
      end record;

   R : Foo;
   
begin
   for J in Integer range 0 .. 3 loop
      pragma Loop_Invariant(R.X = R.X'Loop_Entry);
      for I in Integer range 0 .. 3 loop
         pragma Loop_Invariant(R.X = R.X'Loop_Entry);
         R.Y := I;
      end loop;
   end loop;
end;
