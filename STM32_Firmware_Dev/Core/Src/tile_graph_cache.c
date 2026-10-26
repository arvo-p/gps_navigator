/**
 * @file tile_graph_cache.c
 * @brief Spatial tile graph manager and LRU cache implementation
 *        (Embedded STM32 adaptation using FatFs & shared external SDRAM/PSRAM pool).
 */

#include "tile_graph_cache.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* Pre-allocated static graph buffers in external SDRAM (.sdram).
 * Because Local A* (Phase 1 / Phase 3) and Backbone A* (Phase 2) execute
 * sequentially in separate phases, local_slots and backbone share the same
 * 5.52 MB external RAM union so both fit inside an 8 MB SDRAM/PSRAM chip.
 */
#define EXT_RAM_BSS __attribute__((section(".sdram")))

static union {
    struct {
        NodeRecord nodes[TILE_GRAPH_CACHE_MAX_LOCAL][TILE_MAX_LOCAL_NODES];
        EdgeRecord edges[TILE_GRAPH_CACHE_MAX_LOCAL][TILE_MAX_LOCAL_EDGES];
    } local;
    struct {
        NodeRecord nodes[TILE_MAX_BACKBONE_NODES];
        EdgeRecord edges[TILE_MAX_BACKBONE_EDGES];
    } bb;
} s_graph_pool EXT_RAM_BSS;

static void free_graph(TileGraph *g) {
    if (!g) return;
    g->node_count = 0;
    g->edge_count = 0;
    g->loaded = false;
    g->pin_count = 0;
}

static bool load_graph_files(TileGraph *g, const char *nodes_path, const char *edges_path, const char *geom_path) {
    static FIL fil;
    UINT br = 0;

    free_graph(g);

    /* 1. Read Nodes via FatFs */
    if (f_open(&fil, nodes_path, FA_READ) != FR_OK) {
        return false;
    }

    FSIZE_t n_size = f_size(&fil);
    if (n_size == 0 || (n_size % sizeof(NodeRecord)) != 0) {
        f_close(&fil);
        return false;
    }

    uint32_t count = (uint32_t)(n_size / sizeof(NodeRecord));
    if (count > g->max_nodes) {
        f_close(&fil);
        return false;
    }

    uint32_t bytes_to_read = count * (uint32_t)sizeof(NodeRecord);
    if (f_read(&fil, g->nodes, bytes_to_read, &br) != FR_OK || br != bytes_to_read) {
        f_close(&fil);
        return false;
    }
    f_close(&fil);
    g->node_count = count;

    /* 2. Read Edges via FatFs */
    if (f_open(&fil, edges_path, FA_READ) == FR_OK) {
        FSIZE_t e_size = f_size(&fil);
        if (e_size > 0 && (e_size % sizeof(EdgeRecord)) == 0) {
            uint32_t e_count = (uint32_t)(e_size / sizeof(EdgeRecord));
            if (e_count <= g->max_edges) {
                uint32_t e_bytes = e_count * (uint32_t)sizeof(EdgeRecord);
                if (f_read(&fil, g->edges, e_bytes, &br) == FR_OK && br == e_bytes) {
                    g->edge_count = e_count;
                }
            }
        }
        f_close(&fil);
    }

    /* 3. Store Geometry file path for on-demand curve loading */
    strncpy(g->geom_path, geom_path, sizeof(g->geom_path) - 1);
    g->geom_path[sizeof(g->geom_path) - 1] = '\0';

    g->loaded = true;
    return true;
}

bool tile_graph_cache_init(TileGraphCache *cache, const char *base_dir, int32_t local_capacity) {
    if (!cache) return false;

    memset(cache, 0, sizeof(TileGraphCache));
    if (base_dir && base_dir[0] != '\0') {
        strncpy(cache->base_dir, base_dir, sizeof(cache->base_dir) - 1);
    } else {
        strcpy(cache->base_dir, "binary_map_data");
    }

    cache->local_capacity = (local_capacity > 0 && local_capacity <= TILE_GRAPH_CACHE_MAX_LOCAL)
                            ? local_capacity : TILE_GRAPH_CACHE_MAX_LOCAL;
    cache->access_clock = 1;

    for (int32_t i = 0; i < TILE_GRAPH_CACHE_MAX_LOCAL; ++i) {
        cache->local_slots[i].nodes = s_graph_pool.local.nodes[i];
        cache->local_slots[i].max_nodes = TILE_MAX_LOCAL_NODES;
        cache->local_slots[i].edges = s_graph_pool.local.edges[i];
        cache->local_slots[i].max_edges = TILE_MAX_LOCAL_EDGES;
        cache->local_slots[i].loaded = false;
        cache->local_slots[i].pin_count = 0;
    }

    cache->backbone.nodes = s_graph_pool.bb.nodes;
    cache->backbone.max_nodes = TILE_MAX_BACKBONE_NODES;
    cache->backbone.edges = s_graph_pool.bb.edges;
    cache->backbone.max_edges = TILE_MAX_BACKBONE_EDGES;
    cache->backbone.loaded = false;
    cache->backbone.pin_count = 0;
    cache->backbone_loaded = false;
    cache->backbone_profile = -1;

    return true;
}

TileGraph* tile_graph_cache_get_tile(TileGraphCache *cache, int32_t grid_lat, int32_t grid_lon) {
    if (!cache) return NULL;

    cache->access_clock++;

    /* If backbone currently occupies the shared SDRAM union, invalidate it before loading local tiles */
    if (cache->backbone_loaded) {
        free_graph(&cache->backbone);
        cache->backbone_loaded = false;
        cache->backbone_profile = -1;
    }

    /* 1. Check if already cached */
    for (int32_t i = 0; i < cache->local_capacity; ++i) {
        TileGraph *slot = &cache->local_slots[i];
        if (slot->loaded && slot->grid_lat == grid_lat && slot->grid_lon == grid_lon) {
            slot->lru_counter = cache->access_clock;
            return slot;
        }
    }

    /* 2. Format file paths */
    char nodes_path[TILE_GRAPH_PATH_MAX];
    char edges_path[TILE_GRAPH_PATH_MAX];
    char geom_path[TILE_GRAPH_PATH_MAX];

    snprintf(nodes_path, sizeof(nodes_path), "%s/local_%03ld_%03ld_nodes.bin", cache->base_dir, (long)grid_lat, (long)grid_lon);
    snprintf(edges_path, sizeof(edges_path), "%s/local_%03ld_%03ld_edges.bin", cache->base_dir, (long)grid_lat, (long)grid_lon);
    snprintf(geom_path,  sizeof(geom_path),  "%s/local_%03ld_%03ld_geom.bin",  cache->base_dir, (long)grid_lat, (long)grid_lon);

    /* 3. Find LRU slot that is NOT pinned */
    int32_t lru_idx = -1;
    uint32_t min_lru = 0xFFFFFFFF;
    for (int32_t i = 0; i < cache->local_capacity; ++i) {
        if (!cache->local_slots[i].loaded) {
            lru_idx = i;
            break;
        }
        if (cache->local_slots[i].pin_count == 0 && cache->local_slots[i].lru_counter < min_lru) {
            min_lru = cache->local_slots[i].lru_counter;
            lru_idx = i;
        }
    }

    if (lru_idx < 0) {
        return NULL;
    }

    TileGraph *slot = &cache->local_slots[lru_idx];
    if (!load_graph_files(slot, nodes_path, edges_path, geom_path)) {
        return NULL;
    }

    slot->grid_lat = grid_lat;
    slot->grid_lon = grid_lon;
    slot->is_backbone = false;
    slot->lru_counter = cache->access_clock;
    slot->pin_count = 0;

    return slot;
}

TileGraph* tile_graph_cache_get_by_coord(TileGraphCache *cache, int32_t lat_e7, int32_t lon_e7) {
    int32_t glat, glon;
    geo_coord_to_grid(lat_e7, lon_e7, &glat, &glon);
    return tile_graph_cache_get_tile(cache, glat, glon);
}

TileGraph* tile_graph_cache_get_backbone_for_profile(TileGraphCache *cache, RoutingProfile profile) {
    if (!cache) return NULL;

    if (cache->backbone_loaded && cache->backbone_profile == (int32_t)profile) {
        return &cache->backbone;
    }

    /* Invalidate local slots occupying the shared SDRAM union before loading backbone */
    for (int32_t i = 0; i < cache->local_capacity; ++i) {
        free_graph(&cache->local_slots[i]);
    }

    char nodes_path[TILE_GRAPH_PATH_MAX];
    char edges_path[TILE_GRAPH_PATH_MAX];
    char geom_path[TILE_GRAPH_PATH_MAX];

    if (profile == PROFILE_STROLLER) {
        snprintf(nodes_path, sizeof(nodes_path), "%s/stroller_backbone_nodes.bin", cache->base_dir);
        snprintf(edges_path, sizeof(edges_path), "%s/stroller_backbone_edges.bin", cache->base_dir);
        snprintf(geom_path,  sizeof(geom_path),  "%s/stroller_backbone_geom.bin",  cache->base_dir);
        if (!load_graph_files(&cache->backbone, nodes_path, edges_path, geom_path)) {
            snprintf(nodes_path, sizeof(nodes_path), "%s/backbone_nodes.bin", cache->base_dir);
            snprintf(edges_path, sizeof(edges_path), "%s/backbone_edges.bin", cache->base_dir);
            snprintf(geom_path,  sizeof(geom_path),  "%s/backbone_geom.bin",  cache->base_dir);
            if (!load_graph_files(&cache->backbone, nodes_path, edges_path, geom_path)) {
                cache->backbone_loaded = false;
                cache->backbone_profile = -1;
                return NULL;
            }
        }
    } else {
        snprintf(nodes_path, sizeof(nodes_path), "%s/backbone_nodes.bin", cache->base_dir);
        snprintf(edges_path, sizeof(edges_path), "%s/backbone_edges.bin", cache->base_dir);
        snprintf(geom_path,  sizeof(geom_path),  "%s/backbone_geom.bin",  cache->base_dir);
        if (!load_graph_files(&cache->backbone, nodes_path, edges_path, geom_path)) {
            cache->backbone_loaded = false;
            cache->backbone_profile = -1;
            return NULL;
        }
    }

    cache->backbone.grid_lat = -1;
    cache->backbone.grid_lon = -1;
    cache->backbone.is_backbone = true;
    cache->backbone.lru_counter = 0;
    cache->backbone_loaded = true;
    cache->backbone_profile = (int32_t)profile;

    return &cache->backbone;
}

TileGraph* tile_graph_cache_get_backbone(TileGraphCache *cache) {
    return tile_graph_cache_get_backbone_for_profile(cache, PROFILE_CAR);
}

static bool node_is_well_connected(const TileGraph *graph, uint32_t start_idx, RoutingProfile profile, uint32_t min_reach) {
    if (min_reach <= 1) return true;
    uint32_t q[64];
    uint32_t count = 0;
    uint32_t head = 0;
    q[count++] = start_idx;

    uint16_t req_edge = (profile == PROFILE_STROLLER) ? EDGE_STROLLER_ALLOWED : EDGE_CAR_ALLOWED;

    while (head < count && count < min_reach && count < 64) {
        uint32_t u = q[head++];
        const NodeRecord *un = &graph->nodes[u];
        for (uint16_t e = 0; e < un->edge_count; ++e) {
            uint32_t e_idx = un->first_edge + e;
            if (e_idx >= graph->edge_count) continue;
            const EdgeRecord *edge = &graph->edges[e_idx];
            if (edge->edge_flags != 0 && !(edge->edge_flags & req_edge)) continue;

            uint32_t v = edge->target_idx;
            if (v >= graph->node_count) continue;

            bool seen = false;
            for (uint32_t k = 0; k < count; ++k) {
                if (q[k] == v) {
                    seen = true;
                    break;
                }
            }
            if (!seen && count < 64) {
                q[count++] = v;
                if (count >= min_reach) return true;
            }
        }
    }
    return (count >= min_reach);
}

bool tile_graph_snap_node_for_profile(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t min_edges,
                                      RoutingProfile profile, int32_t *out_node_idx, uint32_t *out_dist_m) {
    if (!graph || !graph->loaded || graph->node_count == 0) return false;

    GeoCoord target = geo_coord_make(lat_e7, lon_e7);
    uint32_t best_dist = 0xFFFFFFFF;
    int32_t best_idx = -1;

    uint16_t req_flag = (profile == PROFILE_STROLLER) ? FLAG_STROLLER_NODE : FLAG_CAR_NODE;
    uint16_t avoid_barrier = (profile == PROFILE_STROLLER)
                             ? (FLAG_BARRIER_GATE | FLAG_STROLLER_BARRIER)
                             : FLAG_BARRIER_GATE;
    uint32_t min_reach = (graph->node_count > 64) ? 24 : 1;

    /* Pass 1: Match well-connected nodes explicitly flagged for this profile */
    for (uint32_t i = 0; i < graph->node_count; ++i) {
        const NodeRecord *n = &graph->nodes[i];
        if (n->edge_count >= min_edges && (n->flags & req_flag) && !(n->flags & avoid_barrier)) {
            GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
            uint32_t d = geo_distance_m(nc, target);
            if (d < best_dist && node_is_well_connected(graph, i, profile, min_reach)) {
                best_dist = d;
                best_idx = (int32_t)i;
            }
        }
    }

    /* Pass 2: Legacy binary compatibility (when neither FLAG_CAR_NODE nor FLAG_STROLLER_NODE is set) */
    if (best_idx < 0) {
        for (uint32_t i = 0; i < graph->node_count; ++i) {
            const NodeRecord *n = &graph->nodes[i];
            bool has_mode_bits = (n->flags & (FLAG_CAR_NODE | FLAG_STROLLER_NODE)) != 0;
            if (has_mode_bits && !(n->flags & req_flag)) continue;

            if (n->edge_count >= min_edges && !(n->flags & avoid_barrier)) {
                GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
                uint32_t d = geo_distance_m(nc, target);
                if (d < best_dist) {
                    best_dist = d;
                    best_idx = (int32_t)i;
                }
            }
        }
    }

    /* Pass 3: Relaxed fallback (any node with matching mode bit) */
    if (best_idx < 0) {
        for (uint32_t i = 0; i < graph->node_count; ++i) {
            const NodeRecord *n = &graph->nodes[i];
            bool has_mode_bits = (n->flags & (FLAG_CAR_NODE | FLAG_STROLLER_NODE)) != 0;
            if (has_mode_bits && !(n->flags & req_flag)) continue;

            if (!(n->flags & FLAG_BARRIER_GATE)) {
                GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
                uint32_t d = geo_distance_m(nc, target);
                if (d < best_dist) {
                    best_dist = d;
                    best_idx = (int32_t)i;
                }
            }
        }
    }

    if (best_idx >= 0) {
        if (out_node_idx) *out_node_idx = best_idx;
        if (out_dist_m) *out_dist_m = best_dist;
        return true;
    }
    return false;
}

bool tile_graph_snap_node(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t min_edges,
                          int32_t *out_node_idx, uint32_t *out_dist_m) {
    return tile_graph_snap_node_for_profile(graph, lat_e7, lon_e7, min_edges, PROFILE_CAR, out_node_idx, out_dist_m);
}

bool tile_graph_find_gateway_for_profile(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7,
                                         RoutingProfile profile, int32_t *out_node_idx, uint32_t *out_dist_m) {
    if (!graph || !graph->loaded || graph->node_count == 0) return false;

    GeoCoord target = geo_coord_make(lat_e7, lon_e7);
    uint32_t best_dist = 0xFFFFFFFF;
    int32_t best_idx = -1;

    uint16_t gw_flag = (profile == PROFILE_STROLLER) ? FLAG_STROLLER_GATEWAY : FLAG_BACKBONE_LINK;

    for (uint32_t i = 0; i < graph->node_count; ++i) {
        const NodeRecord *n = &graph->nodes[i];
        if ((n->flags & gw_flag) && (n->edge_count > 0)) {
            GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
            uint32_t d = geo_distance_m(nc, target);
            if (d < best_dist) {
                best_dist = d;
                best_idx = (int32_t)i;
            }
        }
    }

    /* Fallback to FLAG_BACKBONE_LINK if stroller gateway flag is absent in legacy binary data */
    if (best_idx < 0 && profile == PROFILE_STROLLER) {
        for (uint32_t i = 0; i < graph->node_count; ++i) {
            const NodeRecord *n = &graph->nodes[i];
            if ((n->flags & FLAG_BACKBONE_LINK) && (n->edge_count > 0)) {
                GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
                uint32_t d = geo_distance_m(nc, target);
                if (d < best_dist) {
                    best_dist = d;
                    best_idx = (int32_t)i;
                }
            }
        }
    }

    if (best_idx >= 0) {
        if (out_node_idx) *out_node_idx = best_idx;
        if (out_dist_m) *out_dist_m = best_dist;
        return true;
    }
    return false;
}

bool tile_graph_find_gateway(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7,
                             int32_t *out_node_idx, uint32_t *out_dist_m) {
    return tile_graph_find_gateway_for_profile(graph, lat_e7, lon_e7, PROFILE_CAR, out_node_idx, out_dist_m);
}

bool tile_graph_find_matching_node(const TileGraph *graph, int32_t lat_e7, int32_t lon_e7, int32_t *out_node_idx) {
    if (!graph || !graph->loaded || graph->node_count == 0) return false;

    /* First pass: exact coordinate match */
    for (uint32_t i = 0; i < graph->node_count; ++i) {
        const NodeRecord *n = &graph->nodes[i];
        if (n->lat_scaled == lat_e7 && n->lon_scaled == lon_e7 && n->edge_count > 0) {
            if (out_node_idx) *out_node_idx = (int32_t)i;
            return true;
        }
    }

    /* Second pass: relaxed exact match (even if edge_count == 0) */
    for (uint32_t i = 0; i < graph->node_count; ++i) {
        const NodeRecord *n = &graph->nodes[i];
        if (n->lat_scaled == lat_e7 && n->lon_scaled == lon_e7) {
            if (out_node_idx) *out_node_idx = (int32_t)i;
            return true;
        }
    }

    /* Third pass: nearest neighbor within 25 meters */
    GeoCoord target = geo_coord_make(lat_e7, lon_e7);
    uint32_t best_dist = 0xFFFFFFFF;
    int32_t best_idx = -1;

    for (uint32_t i = 0; i < graph->node_count; ++i) {
        const NodeRecord *n = &graph->nodes[i];
        if (n->edge_count > 0) {
            GeoCoord nc = geo_coord_make(n->lat_scaled, n->lon_scaled);
            uint32_t d = geo_distance_m(nc, target);
            if (d < best_dist) {
                best_dist = d;
                best_idx = (int32_t)i;
            }
        }
    }

    if (best_idx >= 0 && best_dist <= 25) {
        if (out_node_idx) *out_node_idx = best_idx;
        return true;
    }

    return false;
}

void tile_graph_pin(TileGraph *graph) {
    if (graph) {
        graph->pin_count++;
    }
}

void tile_graph_unpin(TileGraph *graph) {
    if (graph && graph->pin_count > 0) {
        graph->pin_count--;
    }
}

void tile_graph_cache_destroy(TileGraphCache *cache) {
    if (!cache) return;

    for (int32_t i = 0; i < cache->local_capacity; ++i) {
        free_graph(&cache->local_slots[i]);
    }

    if (cache->backbone_loaded) {
        free_graph(&cache->backbone);
        cache->backbone_loaded = false;
    }
}
