-- priority_queue.adb:  priority queue implemented using a heap
with Heap_Package;

package body Priority_Queue is

   function Empty (Queue: Queue_Type) return Boolean is
   begin
      return Queue.Number = 0;
   end Empty;

   function Full (Queue: Queue_Type) return Boolean is
   begin
      return (Queue.Number = Queue.Capacity);
   end Full;

   function Size (Queue: Queue_Type) return Natural is
   begin
      return Queue.Number;
   end Size;

   procedure Clear (Queue: in out Queue_Type) is
   begin
      Queue.Number := 0;
   end Clear;

   function ">" (Left, Right: Element_Type) return Boolean is
   begin
      return (Priority_Of (Left) > Priority_Of (Right));
   end ">";

   package Heap is new Heap_Package (Element_Type, Heap_Array);

   procedure Enqueue (Queue: in out Queue_Type;
                      Item : in     Element_Type) is
   begin
      if not Full (Queue) then
         Queue.Number := Queue.Number + 1;
         Queue.Elements(Queue.Number) := Item;
         Heap.Reheap_Up (Queue.Elements (1..Queue.Number));
      else
         raise Constraint_Error with "queue overflow";
      end if;
   end Enqueue;

   procedure Dequeue (Queue: in out Queue_Type;
                      Item : out     Element_Type) is
   begin
      if not Empty (Queue) then
         Item := Queue.Elements(1);
         Queue.Elements(1) := Queue.Elements(Queue.Number);
         Queue.Number := Queue.Number - 1;
         Heap.Reheap_Down (Queue.Elements (1..Queue.Number));
      else
         raise Constraint_Error with "queue underflow";
      end if;
   end Dequeue;

end Priority_Queue;
