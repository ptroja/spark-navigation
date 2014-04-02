-- priority_queue.ads:  priority queue implemented using a heap

generic

   type Element_Type is private;   -- The type of element in the queue
   type Priority_Type is limited private;
   with function Priority_Of(Element: Element_Type) return Priority_Type;
   with function ">" (Left, Right: Priority_Type) return Boolean is <>;

package Priority_Queue is
   pragma Pure;

   pragma Annotate (GNATprove, External_Axiomatization);

   type Queue_Type (Capacity: Positive) is limited private;

   function Empty (Queue : Queue_Type) return Boolean;
   function Full (Queue : Queue_Type) return Boolean;
   function Size (Queue : Queue_Type) return Natural;

   procedure Clear (Queue : in out Queue_Type);

   procedure Enqueue (Queue : in out Queue_Type;
                      Item : in     Element_Type)
   with
     Pre => not Full(Queue);

   procedure Dequeue (Queue : in out Queue_Type;
                      Item : out     Element_Type)
   with
     Pre => not Empty(Queue);

private

   type Heap_Array is array (Positive range <>) of Element_Type;

   -- Record type with discrimiant for array index constraint
   type Queue_Type (Capacity : Positive) is
      record
         Number   : Natural := 0;
         Elements : Heap_Array (1..Capacity);
      end record;

end Priority_Queue;
