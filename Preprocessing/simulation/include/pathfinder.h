/**
 * @file pathfinder.h
 * @brief Hybrid A* pathfinding engine (Mode A: Local Multi-Tile, Mode B: Hierarchical Backbone).
 */

#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <stdint.h>
#include <stdbool.h>
#include "geo_math.h"
#include "tile_graph_cache.h"
#include "map_binary_formats.h"

#define ROUTE_MAX_SEGMENTS 2048
#define ROUTE_MAX_GEOM_POINTS 16384

typedef struct {
    int32_t start_lat_e7;
    int32_t start_lon_e7;
    int32_t end_lat_e7;
    int32_t end_lon_e7;
    uint16_t length_m;
    uint16_t cost_sec;
    uint32_t street_id;
    uint32_t geom_offset;
    uint16_t geom_count;
    const char *geom_path;
} RouteSegment;

typedef struct {
    RouteSegment segments[ROUTE_MAX_SEGMENTS];
    int32_t segment_count;

    uint32_t total_distance_m;
    uint32_t total_time_sec;

    /* Flattened curve geometry points ready for instant Raylib rendering */
    GeomPoint geom_points[ROUTE_MAX_GEOM_POINTS];
    int32_t geom_point_count;

    bool found;
    int routing_mode;    /* 1 = Mode A (Local Multi-Tile), 2 = Mode B (Hierarchical Backbone) */
    int routing_profile; /* 0 = PROFILE_CAR, 1 = PROFILE_STROLLER */
} RouteResult;

typedef struct {
    TileGraphCache *cache;
    RoutingProfile profile;
} Pathfinder;

/**
 * @brief Initialize the pathfinder engine with a tile graph cache.
 */
bool pathfinder_init(Pathfinder *pf, TileGraphCache *cache);

/**
 * @brief Switch active routing profile (PROFILE_CAR or PROFILE_STROLLER).
 */
void pathfinder_set_profile(Pathfinder *pf, RoutingProfile profile);

/**
 * @brief Find optimal route between two GPS coordinates using the hybrid strategy.
 */
bool pathfinder_find_route(Pathfinder *pf, GeoCoord start, GeoCoord goal, RouteResult *out_route);

/**
 * @brief Load intermediate curve geometry points from *_geom.bin for the route.
 */
bool pathfinder_load_geometry(Pathfinder *pf, RouteResult *out_route);

/**
 * @brief Free allocated memory in RouteResult.
 */
void pathfinder_free_route(RouteResult *route);

/**
 * @brief Destroy pathfinder resources.
 */
void pathfinder_destroy(Pathfinder *pf);

#endif /* PATHFINDER_H */
