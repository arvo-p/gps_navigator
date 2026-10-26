/**
 * @file tile_cache.h
 * @brief Raylib-based tile loader and LRU texture cache.
 */

#ifndef TILE_CACHE_H
#define TILE_CACHE_H

#include "raylib.h"
#include "geo_math.h"
#include <stdbool.h>

#define TILE_CACHE_DEFAULT_CAPACITY 256
#define TILE_PATH_MAX 512

typedef struct {
    TileCoord tile;
    Texture2D texture;
    bool loaded;
    bool file_missing;  /**< Negative cache: tile file does not exist on disk */
    unsigned int lru_counter;
} TileCacheEntry;

typedef struct {
    char base_dir[256];
    TileCacheEntry entries[TILE_CACHE_DEFAULT_CAPACITY];
    int capacity;
    int count;
    unsigned int access_clock;

    /* Performance counters */
    unsigned long long cache_hits;
    unsigned long long cache_misses;
    unsigned long long disk_loads;
} TileCache;

/**
 * @brief Initialize the tile cache.
 * @param cache Pointer to TileCache
 * @param base_dir Directory containing tile tree (e.g. "../tiles")
 * @param capacity Maximum number of textures to keep in memory (e.g. 256)
 */
bool tile_cache_init(TileCache *cache, const char *base_dir, int capacity);

/**
 * @brief Get texture for a tile. Loads from disk if needed.
 * @return Pointer to Texture2D if loaded, or NULL if file is not found.
 */
Texture2D* tile_cache_get(TileCache *cache, TileCoord tile);

/**
 * @brief Check if tile file exists on disk without loading.
 */
bool tile_cache_file_exists(const TileCache *cache, TileCoord tile);

/**
 * @brief Format relative or absolute path for a tile.
 */
void tile_cache_get_tile_path(const TileCache *cache, TileCoord tile, char *out_path, int max_len);

/**
 * @brief Free all textures and allocated memory.
 */
void tile_cache_destroy(TileCache *cache);

#endif /* TILE_CACHE_H */
