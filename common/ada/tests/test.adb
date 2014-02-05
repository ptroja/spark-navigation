package body Test is

   function Foo(This: Rec; X : Integer) return Integer is
      -- XXX: If a dummy variable is initialized with record field
      -- division check is erroneously proved to be safe.
      --dummy : Integer := This.A;
   begin
      return 1/X;
   end;

end;
