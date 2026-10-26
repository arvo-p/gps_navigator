/**
 * @file astar_heap.c
 * @brief Binary min-heap priority queue implementation.
 */

#include "astar_heap.h"
#include <string.h>

void astar_heap_init(AstarHeap *heap) {
    if (heap) {
        heap->size = 0;
    }
}

void astar_heap_clear(AstarHeap *heap) {
    if (heap) {
        heap->size = 0;
    }
}

bool astar_heap_is_empty(const AstarHeap *heap) {
    return (!heap || heap->size == 0);
}

bool astar_heap_push(AstarHeap *heap, uint32_t node_key, uint32_t g_cost, uint32_t f_score) {
    if (!heap || heap->size >= ASTAR_HEAP_MAX_ENTRIES) {
        return false;
    }

    int32_t idx = heap->size++;
    heap->entries[idx].node_key = node_key;
    heap->entries[idx].g_cost = g_cost;
    heap->entries[idx].f_score = f_score;

    /* Sift up */
    while (idx > 0) {
        int32_t parent = (idx - 1) / 2;
        if (heap->entries[idx].f_score >= heap->entries[parent].f_score) {
            break;
        }
        HeapEntry temp = heap->entries[idx];
        heap->entries[idx] = heap->entries[parent];
        heap->entries[parent] = temp;
        idx = parent;
    }

    return true;
}

bool astar_heap_pop(AstarHeap *heap, HeapEntry *out_entry) {
    if (!heap || heap->size == 0) return false;

    if (out_entry) {
        *out_entry = heap->entries[0];
    }

    heap->entries[0] = heap->entries[--heap->size];

    /* Sift down */
    int32_t idx = 0;
    while (1) {
        int32_t left = 2 * idx + 1;
        int32_t right = 2 * idx + 2;
        int32_t smallest = idx;

        if (left < heap->size && heap->entries[left].f_score < heap->entries[smallest].f_score) {
            smallest = left;
        }
        if (right < heap->size && heap->entries[right].f_score < heap->entries[smallest].f_score) {
            smallest = right;
        }

        if (smallest == idx) {
            break;
        }

        HeapEntry temp = heap->entries[idx];
        heap->entries[idx] = heap->entries[smallest];
        heap->entries[smallest] = temp;
        idx = smallest;
    }

    return true;
}

void astar_heap_destroy(AstarHeap *heap) {
    if (heap) {
        heap->size = 0;
    }
}
