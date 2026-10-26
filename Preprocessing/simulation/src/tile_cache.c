/**
 * @file tile_cache.c
 * @brief Raylib-based tile loader and LRU texture cache implementation.
 */

#include "tile_cache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void tile_cache_get_tile_path(const TileCache *cache, TileCoord tile, char *out_path, int max_len) {
    if (!cache || !out_path || max_len <= 0) return;
    snprintf(out_path, max_len, "%s/%d/%d/%d.png", cache->base_dir, tile.zoom, tile.x, tile.y);
}

bool tile_cache_file_exists(const TileCache *cache, TileCoord tile) {
    char path[TILE_PATH_MAX];
    tile_cache_get_tile_path(cache, tile, path, sizeof(path));
    return FileExists(path);
}

bool tile_cache_init(TileCache *cache, const char *base_dir, int capacity) {
    if (!cache) return false;

    memset(cache, 0, sizeof(TileCache));
    if (base_dir && base_dir[0] != '\0') {
        strncpy(cache->base_dir, base_dir, sizeof(cache->base_dir) - 1);
    } else {
        strcpy(cache->base_dir, "../tiles");
    }

    cache->capacity = (capacity > 0 && capacity <= TILE_CACHE_DEFAULT_CAPACITY)
                      ? capacity : TILE_CACHE_DEFAULT_CAPACITY;
    cache->count = 0;
    cache->access_clock = 1;
    return true;
}

Texture2D* tile_cache_get(TileCache *cache, TileCoord tile) {
    if (!cache) return NULL;

    cache->access_clock++;

    /* 1. Check if already in cache */
    for (int i = 0; i < cache->count; ++i) {
        TileCacheEntry *entry = &cache->entries[i];
        if (entry->tile.zoom == tile.zoom &&
            entry->tile.x == tile.x &&
            entry->tile.y == tile.y) {

            entry->lru_counter = cache->access_clock;
            if (entry->loaded) {
                cache->cache_hits++;
                return &entry->texture;
            }
            if (entry->file_missing) {
                cache->cache_hits++;
                return NULL;
            }
        }
    }

    cache->cache_misses++;

    /* 2. Check if file exists on disk */
    char path[TILE_PATH_MAX];
    tile_cache_get_tile_path(cache, tile, path, sizeof(path));

    if (!FileExists(path)) {
        /* Record in negative cache so we don't query filesystem repeatedly */
        int slot = -1;
        if (cache->count < cache->capacity) {
            slot = cache->count++;
        } else {
            /* Evict LRU */
            unsigned int min_lru = 0xFFFFFFFF;
            int lru_idx = 0;
            for (int i = 0; i < cache->capacity; ++i) {
                if (cache->entries[i].lru_counter < min_lru) {
                    min_lru = cache->entries[i].lru_counter;
                    lru_idx = i;
                }
            }
            slot = lru_idx;
            if (cache->entries[slot].loaded) {
                UnloadTexture(cache->entries[slot].texture);
            }
        }

        TileCacheEntry *entry = &cache->entries[slot];
        entry->tile = tile;
        entry->loaded = false;
        entry->file_missing = true;
        entry->lru_counter = cache->access_clock;
        return NULL;
    }

    /* 3. Load texture from disk */
    Image img = LoadImage(path);
    if (img.data == NULL) {
        return NULL;
    }

    Texture2D tex = LoadTextureFromImage(img);
    UnloadImage(img);

    if (tex.id == 0) {
        return NULL;
    }

    SetTextureFilter(tex, TEXTURE_FILTER_POINT);
    cache->disk_loads++;

    /* 4. Store in cache */
    int slot = -1;
    if (cache->count < cache->capacity) {
        slot = cache->count++;
    } else {
        /* Evict LRU */
        unsigned int min_lru = 0xFFFFFFFF;
        int lru_idx = 0;
        for (int i = 0; i < cache->capacity; ++i) {
            if (cache->entries[i].lru_counter < min_lru) {
                min_lru = cache->entries[i].lru_counter;
                lru_idx = i;
            }
        }
        slot = lru_idx;
        if (cache->entries[slot].loaded) {
            UnloadTexture(cache->entries[slot].texture);
        }
    }

    TileCacheEntry *entry = &cache->entries[slot];
    entry->tile = tile;
    entry->texture = tex;
    entry->loaded = true;
    entry->file_missing = false;
    entry->lru_counter = cache->access_clock;

    return &entry->texture;
}

void tile_cache_destroy(TileCache *cache) {
    if (!cache) return;

    for (int i = 0; i < cache->count; ++i) {
        if (cache->entries[i].loaded) {
            UnloadTexture(cache->entries[i].texture);
            cache->entries[i].loaded = false;
        }
    }

    cache->count = 0;
    cache->capacity = 0;
}
