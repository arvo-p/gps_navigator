/**
 * @file ui.h
 * @brief User interface controls, coordinate input boxes, and HUD for Raylib simulation.
 */

#ifndef UI_H
#define UI_H

#include "raylib.h"
#include "map_camera.h"
#include "tile_cache.h"
#include <stdbool.h>

#include "pathfinder.h"

#define UI_TEXT_BUFFER_SIZE 32
#define UI_STATUS_BUFFER_SIZE 128

typedef struct {
    char text[UI_TEXT_BUFFER_SIZE];
    int cursor;
    bool active;
    Rectangle bounds;
    const char *label;
} TextInput;

typedef struct {
    TextInput lat_input;
    TextInput lon_input;
    char status_message[UI_STATUS_BUFFER_SIZE];
    Color status_color;
    float status_timer;

    bool mouse_dragging;
    Vector2 mouse_drag_start;

    /* Available zoom levels from ../tiles */
    int available_zooms[3];
    int available_zoom_count;

    /* Route planning pins */
    GeoCoord route_start;
    bool has_route_start;
    GeoCoord route_goal;
    bool has_route_goal;

    /* Active routing profile (PROFILE_CAR or PROFILE_STROLLER) */
    RoutingProfile routing_profile;
    bool recompute_route;
} UIState;

/**
 * @brief Initialize UI state with default coordinates and settings.
 */
void ui_init(UIState *ui, GeoCoord initial_coord);

/**
 * @brief Process user inputs (keyboard, mouse, text fields, route clicks) and update camera.
 */
void ui_update(UIState *ui, MapCamera *cam, TileCache *cache, Pathfinder *pf, RouteResult *route, float dt);

/**
 * @brief Render UI overlay (control panel, text fields, HUD, route info, preset buttons).
 */
void ui_draw(UIState *ui, const MapCamera *cam, const TileCache *cache, const RouteResult *route);

/**
 * @brief Draw calculated route geometry curves (*_geom.bin) on map.
 */
void ui_draw_route(const RouteResult *route, const MapCamera *cam);

/**
 * @brief Draw start and goal location pins.
 */
void ui_draw_pins(const UIState *ui, const MapCamera *cam);

/**
 * @brief Draw center crosshair and coordinate tag.
 */
void ui_draw_crosshair(const MapCamera *cam);

#endif /* UI_H */
