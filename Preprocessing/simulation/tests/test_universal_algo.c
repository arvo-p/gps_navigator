/**
 * @file test_universal_algo.c
 * @brief Unit tests for pure fixed-point / integer geographic and camera algorithms.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "geo_math.h"
#include "map_camera.h"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            fprintf(stderr, "FAIL: %s (line %d): %s\n", __func__, __LINE__, msg); \
            exit(1); \
        } \
    } while (0)

static int32_t int_abs(int32_t v) {
    return (v < 0) ? -v : v;
}

void test_coordinate_roundtrip(void) {
    printf("[TEST] Integer coordinate projection roundtrip...\n");
    GeoCoord coords[] = {
        { 506333000, 55667000 },   /* Liège (50.6333000°, 5.5667000°) */
        { 0, 0 },                   /* Equator / Prime Meridian */
        { 450000000, -930000000 },  /* North America */
        { -338688000, 1512093000 }  /* Sydney (-33.8688000°, 151.2093000°) */
    };

    for (size_t i = 0; i < sizeof(coords)/sizeof(coords[0]); ++i) {
        WorldPoint32 wp;
        GeoCoord out;
        geo_latlon_to_world_point32(coords[i], &wp);
        geo_world_point32_to_latlon(wp, &out);

        int32_t diff_lat = int_abs(coords[i].lat_e7 - out.lat_e7);
        int32_t diff_lon = int_abs(coords[i].lon_e7 - out.lon_e7);

        /* Roundtrip precision within a few e7 units (1 e7 unit ≈ 1.1 cm) */
        TEST_ASSERT(diff_lat < 250, "Latitude mismatch in integer roundtrip");
        TEST_ASSERT(diff_lon < 250, "Longitude mismatch in integer roundtrip");
    }
    printf("  -> PASSED\n");
}

void test_belgium_tile_coordinates(void) {
    printf("[TEST] Belgian integer tile coordinate mapping...\n");
    GeoCoord center = { 506150000, 55750000 }; /* 50.6150°, 5.5750° */

    /* Verify Zoom 12 */
    TileCoord t12;
    geo_latlon_to_tile(center, 12, &t12, NULL);
    TEST_ASSERT(t12.x == 2111 && t12.y == 1378, "Z12 center tile mismatch");

    /* Verify Zoom 14 */
    TileCoord t14;
    geo_latlon_to_tile(center, 14, &t14, NULL);
    TEST_ASSERT(t14.x == 8445 && t14.y == 5512, "Z14 center tile mismatch");

    /* Verify Zoom 16 */
    TileCoord t16;
    geo_latlon_to_tile(center, 16, &t16, NULL);
    TEST_ASSERT(t16.x == 33782 && t16.y == 22050, "Z16 center tile mismatch");

    printf("  -> PASSED\n");
}

void test_camera_panning_and_zoom(void) {
    printf("[TEST] Integer camera pan, zoom, and invariant center...\n");
    MapCamera cam;
    GeoCoord start = { 506242000, 55666000 }; /* 50.6242000°, 5.5666000° */
    map_camera_init(&cam, start, 14, 1280, 720);

    TEST_ASSERT(cam.center.lat_e7 == start.lat_e7, "Camera center lat not initialized correctly");
    TEST_ASSERT(cam.center.lon_e7 == start.lon_e7, "Camera center lon not initialized correctly");

    /* Pan right (East): Lon should increase */
    map_camera_pan_pixels(&cam, 100, 0);
    TEST_ASSERT(cam.center.lon_e7 > start.lon_e7, "Panning right did not increase longitude");
    TEST_ASSERT(int_abs(cam.center.lat_e7 - start.lat_e7) < 5, "Panning horizontally altered latitude");

    /* Pan down (South): Lat should decrease */
    map_camera_pan_pixels(&cam, 0, 100);
    TEST_ASSERT(cam.center.lat_e7 < start.lat_e7, "Panning down did not decrease latitude");

    /* Zoom change should preserve center */
    GeoCoord before_zoom = cam.center;
    map_camera_set_zoom(&cam, 16);
    TEST_ASSERT(int_abs(cam.center.lat_e7 - before_zoom.lat_e7) < 5, "Zooming altered center latitude");
    TEST_ASSERT(int_abs(cam.center.lon_e7 - before_zoom.lon_e7) < 5, "Zooming altered center longitude");

    printf("  -> PASSED\n");
}

void test_visible_tiles_generation(void) {
    printf("[TEST] Visible tiles integer calculation...\n");
    MapCamera cam;
    GeoCoord start = { 506242000, 55666000 };
    map_camera_init(&cam, start, 14, 1280, 720);

    VisibleTileList list;
    map_camera_get_visible_tiles(&cam, &list);

    TEST_ASSERT(list.count > 0, "No visible tiles generated");
    TEST_ASSERT(list.count < MAP_MAX_VISIBLE_TILES, "Visible tiles exceeded maximum capacity");
    TEST_ASSERT(list.count >= 20 && list.count <= 45, "Tile count unexpected for 1280x720");

    /* Center tile (8445, 5512) must be inside visible list */
    bool found_center_tile = false;
    for (int32_t i = 0; i < list.count; ++i) {
        if (list.tiles[i].tile.x == 8445 && list.tiles[i].tile.y == 5512) {
            found_center_tile = true;
            break;
        }
    }
    TEST_ASSERT(found_center_tile, "Center tile not found in visible tiles list");

    printf("  -> PASSED (visible tiles count: %d)\n", list.count);
}

void test_integer_distance_and_bearing(void) {
    printf("[TEST] Integer distance and bearing...\n");
    GeoCoord liege_center = { 506454000, 55735000 };     /* 50.6454°, 5.5735° */
    GeoCoord liege_guillemins = { 506242000, 55666000 }; /* 50.6242°, 5.5666° */

    uint32_t dist = geo_distance_m(liege_center, liege_guillemins);
    /* Distance between Place Saint-Lambert and Liège-Guillemins is approx 2.4 km */
    TEST_ASSERT(dist > 2200 && dist < 2600, "Distance calculation out of expected range");

    int32_t bearing = geo_bearing_deg(liege_center, liege_guillemins);
    /* Guillemins is South-South-West of Center (approx 190-210 degrees) */
    TEST_ASSERT(bearing > 180 && bearing < 220, "Bearing calculation out of expected range");

    printf("  -> PASSED (dist: %u m, bearing: %d deg)\n", dist, bearing);
}

int main(void) {
    printf("===================================================\n");
    printf(" Running Universal Fixed-Point Algorithm Unit Tests\n");
    printf("===================================================\n");

    test_coordinate_roundtrip();
    test_belgium_tile_coordinates();
    test_camera_panning_and_zoom();
    test_visible_tiles_generation();
    test_integer_distance_and_bearing();

    printf("\nALL FIXED-POINT TESTS PASSED SUCCESSFULLY!\n");
    return 0;
}
