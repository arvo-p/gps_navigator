/**
 * @file astar_heap.h
 * @brief Binary min-heap priority queue for A* pathfinding.
 *
 * NOTE: Pure C integer arithmetic, embedded-compatible.
 */

#ifndef ASTAR_HEAP_H
#define ASTAR_HEAP_H

#include <stdint.h>
#include <stdbool.h>

#define ASTAR_HEAP_MAX_ENTRIES 32768

typedef struct {
    uint32_t node_key;  /**< Node identifier */
    uint32_t g_cost;    /**< Exact cost from start (seconds or distance) */
    uint32_t f_score;   /**< Estimated total cost (g_cost + heuristic) */
} HeapEntry;

typedef struct {
    HeapEntry entries[ASTAR_HEAP_MAX_ENTRIES];
    int32_t size;
} AstarHeap;

void astar_heap_init(AstarHeap *heap);
void astar_heap_clear(AstarHeap *heap);
bool astar_heap_push(AstarHeap *heap, uint32_t node_key, uint32_t g_cost, uint32_t f_score);
bool astar_heap_pop(AstarHeap *heap, HeapEntry *out_entry);
bool astar_heap_is_empty(const AstarHeap *heap);
void astar_heap_destroy(AstarHeap *heap);

#endif /* ASTAR_HEAP_H */
