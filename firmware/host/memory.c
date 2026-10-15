/* Free an entire firmware instance when a desktop episode ends. */
#include <stdlib.h>
#include <assert.h>
void *__real_malloc(size_t);
void __real_free(void *);
static void *allocations[512];
static unsigned count;
void *__wrap_malloc(size_t size) {
    void *p = __real_malloc(size);
    if (p) { assert(count < 512); allocations[count++] = p; }
    return p;
}
void __wrap_free(void *p) {
    if (!p) return;
    for (unsigned i=0; i<count; ++i) if (allocations[i] == p) {
        allocations[i] = allocations[--count]; break;
    }
    __real_free(p);
}
void lab_close(void) {
    while (count) __real_free(allocations[--count]);
}
