-- heap_package.ads:  implementation of a heap using an array

generic

   type Element_Type is private;       -- The type of element in the heap
   type Heap_Array is array (Positive range <>) of Element_Type;
   with function ">" (Left, Right: Element_Type) return Boolean is <>;

package Heap_Package is

   pragma Pure;

   -- Assuming that the root of the heap is the only element out of
   -- place, restore the heap property:  not (Heap(Child) > Heap(Parent))
   -- Time complexity: O(log N)
   procedure Reheap_Down (Heap: in out Heap_Array);


   -- Assuming that the last leaf of the heap is the only element out of
   -- place, restore the heap property:  not (Heap(Child) > Heap(Parent))
   -- Time complexity: O(log N)
   procedure Reheap_Up (Heap: in out Heap_Array);


   -- Form a heap out of an array of objects.
   -- Time complexity: O(N log N)
   procedure Heapify (Heap: in out Heap_Array);

end Heap_Package;
