#include <stddef.h>

#include "heap.h"

int
main(int argc, char *argv[])
{
	heap_t * h;
	double v = 12.34;

	h = heap_alloc(8, (heap_free_elt_fn_t) NULL);
	heap_dump(h);
	heap_insert(h, 10.0, &v);
	heap_dump(h);
	heap_free(h);

	return 0;
}
