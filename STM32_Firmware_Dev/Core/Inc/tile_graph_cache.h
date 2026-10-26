/**
 * @file tile_graph_cache.h
 * @brief Spatial tile graph manager and LRU cache for local & backbone binary graphs
 *        (Embedded STM32 + FatFs + External SDRAM/PSRAM adaptation).
 */

#ifndef TILE_GRAPH_CACHE_H
#define TILE_GRAPH_CACHE_H

#include <stdint.h>
#include <stdbool.h>
#include "map_binary_formats.h"
#include "geo_math.h"

#define TILE_GRAPH_CACHE_MAX_LOCAL 2
#define TILE_GRAPH_PATH_MAX        128

#define TILE_MAX_LOCAL_NODES       40000
#define TILE_MAX_LOCAL_EDGES       106000

#define TILE_MAX_BACKBONE_NODES    95000
#define TILE_MAX_BACKBONE_EDGES    135000

/**
 * @brief Routing vehicle/pedestrian profile.
 */
typedef enum {
    PROFILE_CAR = 0,
    PROFILE_STROLLER = 1
} RoutingProfile;

/**
 * @brief In-memory graph structure for a single local tile or backbone network.
 */
typedef struct {
    int32_t grid_lat;          /**< Latitude * 10 (e.g. 506 for 50.6°) or -1 for backbone */
    int32_t grid_lon;          /**< Longitude * 10 (e.g. 55 for 5.5°) or -1 for backbone */
    bool is_backbone;

    NodeRecord *nodes;
    uint32_t node_count;
    uint32_t max_nodes;

    EdgeRecord *edges;
    uint32_t edge_count;
    uint32_t max_edges;

    char geom_path[TILE_GRAPH_PATH_MAX];

    uint32_t lru_counter;
    int32_t pin_count;         /**< Pin counter: cannot be evicted if > 0 */
    bool loaded;
} TileGraph;

/**
 * @brief Tile graph cache holding active local tiles and the backbone graph.
 */
typedef struct {
    char base_dir[64];
    TileGraph local_slots[TILE_GRAPH_CACHE_MAX_LOCAL];
    int32_t local_capacity;

    TileGraph backbone;
    bool backbone_loaded;
    int32_t backbone_profile;

    uint32_t access_clock;
} TileGraphCache;

/**
 * @brief Convert geographic coordinates to 0.1° grid coordinates.
 */
static inline void geo_coord_to_grid(int32_t lat_e7, int32_t lon_e7, int32_t *out_glat, int32_t *out_glon) {
    if (out_glat) *out_glat = lat_e7 / 1000000;
    if (out_glon) *out_glon = lon_e7 / 1000000;
}

/**
 * @brief Check if two grid cells are adjacent or identical.
 */
static inline bool is_adjacent_grid(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    int32_t dlat = lat1 - lat2;
    int32_t dlon = lon1 - lon2;
    if (dlat < 0) dlat = -dlat;
    if (dlon < 0) dlon = -dlon;
    return (dlat <= 1 && dlon <= 1);
}

/**
 * @brief Initialize the tile graph cache.
 */
bool tile_graph_cache_init(TileGraphCache *cache, const char *base_dir, int32_t local_capacity);

/**
 * @brief Get or load a local grid tile by grid ID (e.g. glat=506, glon=55).
 */
TileGraph* tile_graph_cache_get_tile(TileGraphCache *cache, int32_t grid_lat, int32_t grid_lon);

/**
 * @brief Get or load local tile by coordinate (lat_e7, lon_e7).
 */
TileGraph* tile_graph_cache_get_by_coord(TileGraphCache *cache, int32_t lat_e7, int32_t lon_e7);

/**
 * @brief Get or load the global backbone network (backbone_*.bin for Car, stroller_backbone_*.bin for Stroller).
 */
TileGraph* tile_graph_cache_get_backbone(TileGraphCache *cache);
TileGraph* tile_graph_cache_get_backbone_for_profile(TileGraphCache *cache, RoutingProfile profile);

/**
 * @brief Find the closest road node in a graph to a target coordinate.
 * @param min_edges Minimum outgoing edge count (e.g. 1 or 2 to avoid dead ends)
 */
bool tile_graph_snap_node(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t min_edges,
                          int32_t *out_node_idx, uint32_t *out_dist_m);
bool tile_graph_snap_node_for_profile(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t min_edges,
                                      RoutingProfile profile, int32_t *out_node_idx, uint32_t *out_dist_m);

/**
 * @brief Find the closest gateway node (FLAG_BACKBONE_LINK or FLAG_STROLLER_GATEWAY) in a local graph.
 */
bool tile_graph_find_gateway(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7,
                             int32_t *out_node_idx, uint32_t *out_dist_m);
bool tile_graph_find_gateway_for_profile(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7,
                                         RoutingProfile profile, int32_t *out_node_idx, uint32_t *out_dist_m);

/**
 * @brief Find matching node in target graph with identical lat_scaled and lon_scaled.
 */
bool tile_graph_find_matching_node(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t *out_node_idx);

/**
 * @brief Pin and unpin graphs to prevent eviction during A* sessions.
 */
void tile_graph_pin(TileGraph *graph);
void tile_graph_unpin(TileGraph *graph);

/**
 * @brief Free all cached graphs and open file handles.
 */
void tile_graph_cache_destroy(TileGraphCache *cache);

#endif /* TILE_GRAPH_CACHE_H */
