/**
 * @file test_pathfinder.c
 * @brief Unit tests for tile graph loader and hybrid pathfinding engine.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "geo_math.h"
#include "tile_graph_cache.h"
#include "pathfinder.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            fprintf(stderr, "FAIL: %s (line %d): %s\n", __func__, __LINE__, msg); \
            exit(1); \
        } \
    } while (0)

void test_tile_graph_loading(void) {
    printf("[TEST] Tile graph cache loading...\n");

    TileGraphCache cache;
    TEST_ASSERT(tile_graph_cache_init(&cache, "../binary_map_data", 4), "Failed to init cache");

    /* Load Liège local tile (50.6° N, 5.5° E -> glat=506, glon=55) */
    TileGraph *g = tile_graph_cache_get_tile(&cache, 506, 55);
    TEST_ASSERT(g != NULL, "Failed to load local_506_055");
    TEST_ASSERT(g->loaded, "Graph not marked loaded");
    TEST_ASSERT(g->node_count > 10000, "Node count unexpectedly low for local_506_055");
    TEST_ASSERT(g->edge_count > 0, "Edge count is zero for local_506_055");
    printf("  -> local_506_055 loaded: %u nodes, %u edges\n", g->node_count, g->edge_count);

    /* Snap to node near Guillemins (50.6242, 5.5666) */
    int32_t snapped_idx = -1;
    uint32_t snap_dist = 0;
    TEST_ASSERT(tile_graph_snap_node(g, 506242000, 55666000, 1, &snapped_idx, &snap_dist), "Snap failed");
    TEST_ASSERT(snapped_idx >= 0, "Snapped index invalid");
    TEST_ASSERT(snap_dist < 200, "Snap distance too large (>200m)");
    printf("  -> Snapped Guillemins to node %d (distance %u m)\n", snapped_idx, snap_dist);

    tile_graph_cache_destroy(&cache);
    printf("  -> PASSED\n");
}

void test_short_trip_pathfinding(void) {
    printf("[TEST] Short trip pathfinding (Liège local)...\n");

    TileGraphCache cache;
    tile_graph_cache_init(&cache, "../binary_map_data", 4);

    Pathfinder pf;
    pathfinder_init(&pf, &cache);

    GeoCoord start = geo_coord_make(506242000, 55666000); /* Guillemins */
    GeoCoord goal  = geo_coord_make(506454000, 55735000); /* Saint-Lambert */

    static RouteResult route;
    memset(&route, 0, sizeof(RouteResult));

    bool ok = pathfinder_find_route(&pf, start, goal, &route);
    TEST_ASSERT(ok, "Failed to find local route");
    TEST_ASSERT(route.found, "Route marked not found");
    TEST_ASSERT(route.segment_count > 0, "Route has 0 segments");
    TEST_ASSERT(route.total_distance_m > 1000, "Route distance unreasonably short");
    TEST_ASSERT(route.geom_point_count > 0, "No geometry curve points loaded");

    printf("  -> Route found: %d segments, %u m (%.2f km), %u sec, %d curve points (Mode: %d)\n",
           route.segment_count, route.total_distance_m, (double)route.total_distance_m / 1000.0,
           route.total_time_sec, route.geom_point_count, route.routing_mode);

    pathfinder_free_route(&route);
    pathfinder_destroy(&pf);
    tile_graph_cache_destroy(&cache);
    printf("  -> PASSED\n");
}

void test_long_trip_pathfinding(void) {
    printf("[TEST] Long trip pathfinding (Liège -> Namur highway)...\n");

    TileGraphCache cache;
    tile_graph_cache_init(&cache, "../binary_map_data", 4);

    Pathfinder pf;
    pathfinder_init(&pf, &cache);

    GeoCoord start = geo_coord_make(506400000, 55000000); /* Liège E42 highway junction */
    GeoCoord goal  = geo_coord_make(504800000, 48500000); /* Namur E42 highway junction */

    static RouteResult route;
    memset(&route, 0, sizeof(RouteResult));

    bool ok = pathfinder_find_route(&pf, start, goal, &route);
    TEST_ASSERT(ok, "Failed to find long distance route");
    TEST_ASSERT(route.found, "Route marked not found");
    TEST_ASSERT(route.routing_mode == 2, "Expected Mode B (Hierarchical)");
    TEST_ASSERT(route.total_distance_m > 40000, "Distance too short for intercity trip");
    TEST_ASSERT(route.geom_point_count > 0, "No geometry curve points loaded");

    printf("  -> Highway Route found: %d segments, %u m (%.2f km), %u sec (%.1f min), %d curve points (Mode: %d)\n",
           route.segment_count, route.total_distance_m, (double)route.total_distance_m / 1000.0,
           route.total_time_sec, (double)route.total_time_sec / 60.0, route.geom_point_count, route.routing_mode);

    pathfinder_free_route(&route);
    pathfinder_destroy(&pf);
    tile_graph_cache_destroy(&cache);
    printf("  -> PASSED\n");
}

void test_pedestrian_profile_switching(void) {
    printf("[TEST] Pedestrian / Stroller profile switching (Liège local)...\n");

    TileGraphCache cache;
    tile_graph_cache_init(&cache, "../binary_map_data", 4);

    Pathfinder pf;
    pathfinder_init(&pf, &cache);

    GeoCoord start = geo_coord_make(506242000, 55666000); /* Guillemins */
    GeoCoord goal  = geo_coord_make(506454000, 55735000); /* Saint-Lambert */

    static RouteResult car_route;
    static RouteResult ped_route;
    memset(&car_route, 0, sizeof(RouteResult));
    memset(&ped_route, 0, sizeof(RouteResult));

    pathfinder_set_profile(&pf, PROFILE_CAR);
    TEST_ASSERT(pathfinder_find_route(&pf, start, goal, &car_route), "Car route failed");
    TEST_ASSERT(car_route.routing_profile == PROFILE_CAR, "Car profile mismatch");

    pathfinder_set_profile(&pf, PROFILE_STROLLER);
    TEST_ASSERT(pathfinder_find_route(&pf, start, goal, &ped_route), "Pedestrian route failed");
    TEST_ASSERT(ped_route.routing_profile == PROFILE_STROLLER, "Pedestrian profile mismatch");
    TEST_ASSERT(ped_route.total_time_sec > car_route.total_time_sec, "Walking time should exceed car driving time");

    printf("  -> Car: %u m, %u sec | Pedestrian/Stroller: %u m, %u sec (%.1f min)\n",
           car_route.total_distance_m, car_route.total_time_sec,
           ped_route.total_distance_m, ped_route.total_time_sec,
           (double)ped_route.total_time_sec / 60.0);

    /* Test Parc de Cointe pedestrian route (from no_route.jpg screenshot) */
    GeoCoord cointe_start = geo_coord_make(506189910, 55705580); /* Parc de Cointe / Rue des Cailloux */
    GeoCoord cointe_goal  = geo_coord_make(506234510, 55705400); /* Residential street north of park */
    memset(&ped_route, 0, sizeof(RouteResult));
    TEST_ASSERT(pathfinder_find_route(&pf, cointe_start, cointe_goal, &ped_route), "Parc de Cointe pedestrian route failed");
    printf("  -> Parc de Cointe pedestrian route found: %d segments, %u m, %u sec (Mode %d)\n",
           ped_route.segment_count, ped_route.total_distance_m, ped_route.total_time_sec, ped_route.routing_mode);

    /* Test Bois de la Vecquée / Sart-Tilman forest route (from wrong_route.jpg screenshot) */
    GeoCoord vecquee_start = geo_coord_make(505690000, 55583000);
    GeoCoord vecquee_goal  = geo_coord_make(505622000, 55480000);
    memset(&ped_route, 0, sizeof(RouteResult));
    TEST_ASSERT(pathfinder_find_route(&pf, vecquee_start, vecquee_goal, &ped_route), "Bois de la Vecquee route failed");
    TEST_ASSERT(ped_route.routing_mode == 1, "Expected Mode 1 (local forest route) instead of Mode 2 highway jump");
    printf("  -> Bois de la Vecquée pedestrian route found: %d segments, %u m, %u sec (Mode %d)\n",
           ped_route.segment_count, ped_route.total_distance_m, ped_route.total_time_sec, ped_route.routing_mode);

    /* Test Sart-Tilman North-to-South forest route (from route_found_but_not_drawn.jpg screenshot) */
    GeoCoord st_start = geo_coord_make(505832000, 55666000);
    GeoCoord st_goal  = geo_coord_make(505560000, 55680000);
    memset(&ped_route, 0, sizeof(RouteResult));
    TEST_ASSERT(pathfinder_find_route(&pf, st_start, st_goal, &ped_route), "Sart-Tilman N-S route failed");
    TEST_ASSERT(ped_route.geom_point_count > 10, "Route must have drawable geometry points");
    printf("  -> Sart-Tilman N->S pedestrian route found: %d segments, %u m, %d geom pts (Mode %d)\n",
           ped_route.segment_count, ped_route.total_distance_m, ped_route.geom_point_count, ped_route.routing_mode);

    /* Test Long-Distance Mode B Stroller Backbone (Eupen -> High Fens / Signal de Botrange -> Malmedy, >20 km) */
    GeoCoord eupen_start = geo_coord_make(506300000, 60350000);   /* Eupen */
    GeoCoord malmedy_goal = geo_coord_make(504260000, 60280000);  /* Malmedy (across High Fens) */
    memset(&ped_route, 0, sizeof(RouteResult));
    TEST_ASSERT(pathfinder_find_route(&pf, eupen_start, malmedy_goal, &ped_route), "Eupen -> Malmedy stroller backbone route failed");
    TEST_ASSERT(ped_route.routing_mode == 2, "Expected Mode 2 (Stroller Backbone)");
    printf("  -> Eupen -> Malmedy (High Fens Stroller Backbone): %d segments, %.2f km, %.1f hours, %d geom pts (Mode %d)\n",
           ped_route.segment_count, (double)ped_route.total_distance_m / 1000.0,
           (double)ped_route.total_time_sec / 3600.0, ped_route.geom_point_count, ped_route.routing_mode);

    pathfinder_free_route(&car_route);
    pathfinder_free_route(&ped_route);
    pathfinder_destroy(&pf);
    tile_graph_cache_destroy(&cache);
    printf("  -> PASSED\n");
}

int main(void) {
    printf("===================================================\n");
    printf(" Running Pathfinder Unit Tests\n");
    printf("===================================================\n");

    test_tile_graph_loading();
    test_short_trip_pathfinding();
    test_long_trip_pathfinding();
    test_pedestrian_profile_switching();

    printf("\nALL PATHFINDER TESTS PASSED SUCCESSFULLY!\n");
    return 0;
}
