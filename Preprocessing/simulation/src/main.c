/**
 * @file main.c
 * @brief GPS Embedded Simulation Entry Point.
 *
 * Demonstrates map rendering with Raylib using pre-rendered tiles from ../tiles
 * and route planning overlay loaded from ../binary_map_data (*_geom.bin).
 * Keeps universal geographic, camera, and pathfinding algorithms isolated.
 */

#include "raylib.h"
#include "geo_math.h"
#include "map_camera.h"
#include "tile_cache.h"
#include "tile_graph_cache.h"
#include "pathfinder.h"
#include "ui.h"
#include <stdio.h>
#include <string.h>

/* Pre-allocated static route buffer in .bss - Zero malloc */
static RouteResult s_current_route;

int main(void) {
    const int initial_width = 1280;
    const int initial_height = 720;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(initial_width, initial_height, "GPS Embedded System Simulation - Raylib Map & Hybrid Pathfinder");
    SetTargetFPS(60);

    /* 1. Resolve map tiles directory path */
    const char *tiles_dir = "../tiles";
    if (!DirectoryExists(tiles_dir)) {
        if (DirectoryExists("tiles")) {
            tiles_dir = "tiles";
        } else if (DirectoryExists("../../tiles")) {
            tiles_dir = "../../tiles";
        }
    }

    /* Initialize tile texture cache */
    TileCache cache;
    if (!tile_cache_init(&cache, tiles_dir, 256)) {
        TraceLog(LOG_ERROR, "Failed to initialize tile cache!");
    }

    /* 2. Resolve binary road network data directory */
    const char *bin_dir = "../binary_map_data";
    if (!DirectoryExists(bin_dir)) {
        if (DirectoryExists("binary_map_data")) {
            bin_dir = "binary_map_data";
        } else if (DirectoryExists("../../binary_map_data")) {
            bin_dir = "../../binary_map_data";
        }
    }

    /* Initialize spatial tile graph cache (pure static memory) */
    TileGraphCache graph_cache;
    if (!tile_graph_cache_init(&graph_cache, bin_dir, 4)) {
        TraceLog(LOG_ERROR, "Failed to initialize tile graph cache!");
    }

    /* Initialize hybrid pathfinding engine */
    Pathfinder pathfinder;
    pathfinder_init(&pathfinder, &graph_cache);

    /* Clear static route */
    memset(&s_current_route, 0, sizeof(RouteResult));

    /* Initial coordinates: Liège Center (50.6242000° N, 5.5666000° E), Zoom 14 */
    GeoCoord initial_center = geo_coord_make(506242000, 55666000);
    int initial_zoom = 14;

    /* Initialize universal camera */
    MapCamera camera;
    map_camera_init(&camera, initial_center, initial_zoom, initial_width, initial_height);

    /* Initialize user interface */
    UIState ui;
    ui_init(&ui, initial_center);

    VisibleTileList visible_tiles;

    /* Main application loop */
    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        /* Handle window resizing */
        int cur_w = GetScreenWidth();
        int cur_h = GetScreenHeight();
        if (cur_w != camera.viewport_width || cur_h != camera.viewport_height) {
            map_camera_set_viewport(&camera, cur_w, cur_h);
        }

        /* 1. Process user input, route clicks, and update camera position */
        ui_update(&ui, &camera, &cache, &pathfinder, &s_current_route, dt);

        /* 2. Compute visible tiles with universal algorithm */
        map_camera_get_visible_tiles(&camera, &visible_tiles);

        /* 3. Render frame */
        BeginDrawing();

        /* Background matching dark map style (#121417) */
        ClearBackground(CLITERAL(Color){ 18, 20, 23, 255 });

        /* Draw visible tiles */
        for (int i = 0; i < visible_tiles.count; ++i) {
            VisibleTile *vt = &visible_tiles.tiles[i];
            Texture2D *tex = tile_cache_get(&cache, vt->tile);

            if (tex) {
                DrawTexture(*tex, vt->screen_x, vt->screen_y, WHITE);
            } else {
                /* Render subtle placeholder when tile file is absent */
                DrawRectangleLines(vt->screen_x, vt->screen_y, vt->width, vt->height,
                                   CLITERAL(Color){ 32, 36, 42, 255 });
                DrawText(TextFormat("z%d\nx%d\ny%d", vt->tile.zoom, vt->tile.x, vt->tile.y),
                         vt->screen_x + 10, vt->screen_y + 10, 10,
                         CLITERAL(Color){ 60, 68, 78, 255 });
            }
        }

        /* 4. Draw Route Curve Geometry (from *_geom.bin) on top of tiles */
        ui_draw_route(&s_current_route, &camera);

        /* 5. Draw Start & Destination Location Pins */
        ui_draw_pins(&ui, &camera);

        /* 6. Draw center crosshair reticle */
        ui_draw_crosshair(&camera);

        /* 7. Draw user interface overlay (controls, route info, HUD, presets) */
        ui_draw(&ui, &camera, &cache, &s_current_route);

        EndDrawing();
    }

    /* Cleanup resources */
    pathfinder_free_route(&s_current_route);
    pathfinder_destroy(&pathfinder);
    tile_graph_cache_destroy(&graph_cache);
    tile_cache_destroy(&cache);
    CloseWindow();

    return 0;
}
