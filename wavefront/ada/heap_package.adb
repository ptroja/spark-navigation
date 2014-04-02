-- heap_package.adb: implementation of a heap using an array

package body Heap_Package is

   procedure Swap (Left, Right : in out Element_Type) is
      Temp: constant Element_Type := Left;
   begin
      Left := Right;  Right := Temp;
   end Swap;

   -- The calculation assumes that the root of the heap is index 1
   function Left_Subtree (I: Positive) return Positive is
   begin
      return I*2;
   end Left_Subtree;

   -- The calculation assumes that the root of the heap is index 1
   function Right_Subtree (I: Positive) return Positive is
   begin
      return I*2 + 1;
   end Right_Subtree;

   -- The calculation assumes that the root of the heap is index 1
   function Parent_Index (I: Positive) return Positive is
   begin
      return I/2;
   end Parent_Index;

   -- Return the index of the root node's child with the larger key.
   function Larger_Child (Heap: Heap_Array;
                          Root: Positive) return Positive is
      Left_Child  : constant Positive := Left_Subtree (Root);
      Right_Child : constant Positive := Right_Subtree (Root);
   begin
      if Right_Child <= Heap'Last and then
        not (Heap(Left_Child) > Heap(Right_Child)) then
         return Right_Child;
      else
         -- There must be a left subtree, since the root is not a leaf
         -- and this is a heap.
         return Left_Child;
      end if;
   end Larger_Child;

   -- Assuming that the root of the heap is the only element out of
   -- place, restore the heap property:  not (Heap(Child) > Heap(Parent))
   procedure Reheap_Down (Heap : in out Heap_Array) is
      Root      : Positive := Heap'First;
      Max_Child : Positive;  -- Index of child with larger key
   begin
      while Root <= Heap'Last/2 loop
         Max_Child := Larger_Child (Heap, Root);
         exit when not (Heap(Max_Child)>Heap(Root));
         Swap (Heap(Max_Child), Heap(Root));
         Root := Max_Child;
      end loop;
   end Reheap_Down;

   -- Assuming that the last leaf of the heap is the only element out of
   -- place, restore the heap property:  not (Heap(Child) > Heap(Parent))
   procedure Reheap_Up (Heap : in out Heap_Array) is
      Bottom: Positive := Heap'Last;
      Parent: Positive;  -- Index of the parent of Bottom
   begin
      while Bottom > Heap'First loop
         Parent := Parent_Index (Bottom);
         exit when not (Heap(Bottom) > Heap(Parent));
         Swap (Heap(Parent), Heap(Bottom));
         Bottom := Parent;
      end loop;
   end Reheap_Up;

   procedure Heapify (Heap: in out Heap_Array) is
      Last_Interior_Node: constant Positive := Heap'Last / 2;
   begin
      pragma Assert (Heap'First=1);
      for I in reverse 1..Last_Interior_Node loop
         Reheap_Down (Heap(I..Heap'Last));
      end loop;
   end Heapify;

end Heap_Package;
