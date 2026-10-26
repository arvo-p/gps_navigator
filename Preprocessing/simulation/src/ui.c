/**
 * @file ui.c
 * @brief User interface controls, coordinate input boxes, and HUD implementation.
 */

#include "ui.h"
#include <stdio.h>
#include <string.h>

/* UI Theme Colors */
#define UI_COLOR_BG            CLITERAL(Color){ 20, 24, 30, 235 }
#define UI_COLOR_PANEL_HEADER CLITERAL(Color){ 30, 36, 46, 255 }
#define UI_COLOR_BORDER       CLITERAL(Color){ 55, 65, 81, 255 }
#define UI_COLOR_TEXT_MAIN    CLITERAL(Color){ 240, 243, 246, 255 }
#define UI_COLOR_TEXT_MUTED   CLITERAL(Color){ 145, 158, 171, 255 }
#define UI_COLOR_PRIMARY      CLITERAL(Color){ 37, 99, 235, 255 }
#define UI_COLOR_PRIMARY_HOVER CLITERAL(Color){ 59, 130, 246, 255 }
#define UI_COLOR_ACCENT       CLITERAL(Color){ 245, 158, 11, 255 }
#define UI_COLOR_SUCCESS      CLITERAL(Color){ 16, 185, 129, 255 }
#define UI_COLOR_ERROR        CLITERAL(Color){ 239, 68, 68, 255 }
#define UI_COLOR_INPUT_BG     CLITERAL(Color){ 13, 17, 23, 255 }
#define UI_COLOR_INPUT_ACTIVE CLITERAL(Color){ 30, 41, 59, 255 }

/* Preset Locations within Belgian tile area (in 10^-7 degrees) */
typedef struct {
    const char *name;
    int32_t lat_e7;
    int32_t lon_e7;
} PresetLocation;

static const PresetLocation PRESETS[] = {
    { "Liège Center",      506454000, 55735000 },
    { "Liège-Guillemins",  506242000, 55666000 },
    { "Sart-Tilman",       505840000, 55670000 },
    { "Chênée / Meuse",    506120000, 56150000 },
    { "Tile Center",       506150000, 55750000 }
};
static const int PRESET_COUNT = sizeof(PRESETS) / sizeof(PRESETS[0]);

/* Helper for clickable button */
static bool draw_button(Rectangle bounds, const char *text, bool active, Color normal_col, Color hover_col) {
    Vector2 mouse = GetMousePosition();
    bool hovered = CheckCollisionPointRec(mouse, bounds);

    Color bg = active ? UI_COLOR_ACCENT : (hovered ? hover_col : normal_col);
    Color border = hovered ? UI_COLOR_TEXT_MAIN : UI_COLOR_BORDER;
    Color text_col = active ? CLITERAL(Color){ 10, 10, 10, 255 } : UI_COLOR_TEXT_MAIN;

    DrawRectangleRounded(bounds, 0.2f, 4, bg);
    DrawRectangleRoundedLines(bounds, 0.2f, 4, border);

    int font_size = 14;
    int text_width = MeasureText(text, font_size);
    int text_x = (int)(bounds.x + (bounds.width - (float)text_width) * 0.5f);
    int text_y = (int)(bounds.y + (bounds.height - (float)font_size) * 0.5f);

    DrawText(text, text_x, text_y, font_size, text_col);

    return hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

/* Helper to update single text input */
static void update_text_input(TextInput *input) {
    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        input->active = CheckCollisionPointRec(mouse, input->bounds);
    }

    if (!input->active) return;

    /* Handle character typing */
    int key = GetCharPressed();
    while (key > 0) {
        if ((key >= '0' && key <= '9') || key == '.' || key == '-') {
            size_t len = strlen(input->text);
            if (len < UI_TEXT_BUFFER_SIZE - 1) {
                input->text[len] = (char)key;
                input->text[len + 1] = '\0';
                input->cursor = (int)(len + 1);
            }
        }
        key = GetCharPressed();
    }

    /* Backspace handling */
    if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressedRepeat(KEY_BACKSPACE)) {
        size_t len = strlen(input->text);
        if (len > 0) {
            input->text[len - 1] = '\0';
            input->cursor = (int)(len - 1);
        }
    }
}

/* Helper to draw single text input */
static void draw_text_input(const TextInput *input) {
    Color bg = input->active ? UI_COLOR_INPUT_ACTIVE : UI_COLOR_INPUT_BG;
    Color border = input->active ? UI_COLOR_PRIMARY : UI_COLOR_BORDER;

    DrawText(input->label, (int)input->bounds.x, (int)input->bounds.y - 14, 11, UI_COLOR_TEXT_MUTED);
    DrawRectangleRounded(input->bounds, 0.2f, 4, bg);
    DrawRectangleRoundedLines(input->bounds, 0.2f, 4, border);

    DrawText(input->text, (int)input->bounds.x + 8, (int)input->bounds.y + 8, 14, UI_COLOR_TEXT_MAIN);

    if (input->active) {
        int cursor_x = (int)input->bounds.x + 8 + MeasureText(input->text, 14);
        int cursor_y = (int)input->bounds.y + 6;
        DrawLine(cursor_x, cursor_y, cursor_x, cursor_y + 18, UI_COLOR_PRIMARY);
    }
}

static void ui_set_status(UIState *ui, const char *msg, Color color) {
    if (msg) {
        snprintf(ui->status_message, sizeof(ui->status_message), "%s", msg);
    } else {
        ui->status_message[0] = '\0';
    }
    ui->status_color = color;
    ui->status_timer = 4.0f;
}

static bool parse_coord_str(const char *str, int32_t *out_val) {
    if (!str || !out_val) return false;
    while (*str == ' ' || *str == '\t') str++;
    int sign = 1;
    if (*str == '-') { sign = -1; str++; }
    else if (*str == '+') { str++; }
    if (*str == '\0') return false;

    int64_t integer_part = 0;
    bool has_digits = false;
    while (*str >= '0' && *str <= '9') {
        integer_part = integer_part * 10 + (*str - '0');
        has_digits = true;
        str++;
    }
    int64_t frac_part = 0;
    int64_t frac_div = 1;
    if (*str == '.') {
        str++;
        int digits = 0;
        while (*str >= '0' && *str <= '9') {
            has_digits = true;
            if (digits < 7) {
                frac_part = frac_part * 10 + (*str - '0');
                frac_div *= 10;
                digits++;
            }
            str++;
        }
    }
    if (!has_digits) return false;
    while (frac_div < 10000000LL) {
        frac_part *= 10;
        frac_div *= 10;
    }
    int64_t total = (integer_part * 10000000LL + frac_part) * sign;
    *out_val = (int32_t)total;
    return true;
}

static void apply_coordinate_jump(UIState *ui, MapCamera *cam) {
    int32_t parsed_lat = 0;
    int32_t parsed_lon = 0;

    if (!parse_coord_str(ui->lat_input.text, &parsed_lat)) {
        ui_set_status(ui, "Invalid Latitude! Format: e.g. 50.6242", UI_COLOR_ERROR);
        return;
    }
    if (!parse_coord_str(ui->lon_input.text, &parsed_lon)) {
        ui_set_status(ui, "Invalid Longitude! Format: e.g. 5.5666", UI_COLOR_ERROR);
        return;
    }

    if (parsed_lat < -850511287 || parsed_lat > 850511287) {
        ui_set_status(ui, "Latitude out of bounds (-85.05 to +85.05)", UI_COLOR_ERROR);
        return;
    }
    if (parsed_lon < -1800000000 || parsed_lon > 1800000000) {
        ui_set_status(ui, "Longitude out of bounds (-180 to +180)", UI_COLOR_ERROR);
        return;
    }

    GeoCoord target = geo_coord_make(parsed_lat, parsed_lon);
    map_camera_set_center(cam, target);

    char buf[UI_STATUS_BUFFER_SIZE];
    snprintf(buf, sizeof(buf), "Centered on (%.6f, %.6f)",
             (double)parsed_lat / 10000000.0, (double)parsed_lon / 10000000.0);
    ui_set_status(ui, buf, UI_COLOR_SUCCESS);

    ui->lat_input.active = false;
    ui->lon_input.active = false;
}

void ui_init(UIState *ui, GeoCoord initial_coord) {
    memset(ui, 0, sizeof(UIState));

    snprintf(ui->lat_input.text, sizeof(ui->lat_input.text), "%.6f", (double)initial_coord.lat_e7 / 10000000.0);
    ui->lat_input.cursor = (int)strlen(ui->lat_input.text);
    ui->lat_input.label = "Latitude (deg)";

    snprintf(ui->lon_input.text, sizeof(ui->lon_input.text), "%.6f", (double)initial_coord.lon_e7 / 10000000.0);
    ui->lon_input.cursor = (int)strlen(ui->lon_input.text);
    ui->lon_input.label = "Longitude (deg)";

    ui->available_zooms[0] = 12;
    ui->available_zooms[1] = 14;
    ui->available_zooms[2] = 16;
    ui->available_zoom_count = 3;

    ui->routing_profile = PROFILE_CAR;
    ui->recompute_route = false;

    ui_set_status(ui, "Simulation ready (Fixed-point engine). Arrow keys to pan.", UI_COLOR_SUCCESS);
}

void ui_update(UIState *ui, MapCamera *cam, TileCache *cache, Pathfinder *pf, RouteResult *route, float dt) {
    (void)cache;

    if (ui->status_timer > 0.0f) {
        ui->status_timer -= dt;
    }

    /* Update input box bounds based on window layout */
    ui->lat_input.bounds = (Rectangle){ 20, 110, 140, 30 };
    ui->lon_input.bounds = (Rectangle){ 170, 110, 140, 30 };

    /* Process text inputs */
    update_text_input(&ui->lat_input);
    update_text_input(&ui->lon_input);

    /* Tab switching between Lat and Lon inputs */
    if (IsKeyPressed(KEY_TAB)) {
        if (ui->lat_input.active) {
            ui->lat_input.active = false;
            ui->lon_input.active = true;
        } else if (ui->lon_input.active) {
            ui->lon_input.active = false;
            ui->lat_input.active = true;
        } else {
            ui->lat_input.active = true;
        }
    }

    /* Enter key triggers coordinate jump when editing */
    if ((ui->lat_input.active || ui->lon_input.active) && IsKeyPressed(KEY_ENTER)) {
        apply_coordinate_jump(ui, cam);
    }

    /* Escape cancels input */
    if (IsKeyPressed(KEY_ESCAPE)) {
        ui->lat_input.active = false;
        ui->lon_input.active = false;
    }

    /* Process Arrow Keys for camera movement */
    int32_t dir_x = 0;
    int32_t dir_y = 0;

    if (IsKeyDown(KEY_UP))    dir_y -= 1;
    if (IsKeyDown(KEY_DOWN))  dir_y += 1;
    if (IsKeyDown(KEY_LEFT))  dir_x -= 1;
    if (IsKeyDown(KEY_RIGHT)) dir_x += 1;

    /* If text boxes are NOT active, also allow WASD and 'M' mode switch */
    if (!ui->lat_input.active && !ui->lon_input.active) {
        if (IsKeyDown(KEY_W)) dir_y -= 1;
        if (IsKeyDown(KEY_S)) dir_y += 1;
        if (IsKeyDown(KEY_A)) dir_x -= 1;
        if (IsKeyDown(KEY_D)) dir_x += 1;

        if (IsKeyPressed(KEY_M)) {
            ui->routing_profile = (ui->routing_profile == PROFILE_CAR) ? PROFILE_STROLLER : PROFILE_CAR;
            ui->recompute_route = true;
            ui_set_status(ui,
                          (ui->routing_profile == PROFILE_STROLLER)
                              ? "Switched to PEDESTRIAN / STROLLER mode"
                              : "Switched to CAR mode",
                          UI_COLOR_ACCENT);
        }
    }

    if (pf) {
        pathfinder_set_profile(pf, ui->routing_profile);
    }

    if (dir_x != 0 || dir_y != 0) {
        int32_t speed_mult = 1;
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
            speed_mult = 3;
        }
        int32_t dt_ms = (int32_t)(dt * 1000.0f);
        if (dt_ms <= 0) dt_ms = 1;
        map_camera_move_direction(cam, dir_x, dir_y, dt_ms, speed_mult);
    }

    /* Mouse drag panning & pin placement */
    Vector2 mouse = GetMousePosition();
    int screen_w = GetScreenWidth();
    bool mouse_in_ui = (mouse.x < 340 && mouse.y < 720) || (mouse.x > screen_w - 320 && mouse.y < 360);

    /* Click on map to place pins */
    if (!mouse_in_ui) {
        /* Shift + Left Click: Place START pin */
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            map_camera_screen_to_latlon(cam, (int32_t)mouse.x, (int32_t)mouse.y, &ui->route_start);
            ui->has_route_start = true;
            ui_set_status(ui, "Start pin placed (Green). Ctrl+Click or Right-Click to place Goal.", UI_COLOR_SUCCESS);

            if (ui->has_route_goal) {
                ui->recompute_route = true;
            }
        }
        /* Ctrl + Left Click or Right Click: Place GOAL pin */
        else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) ||
                 (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)))) {
            map_camera_screen_to_latlon(cam, (int32_t)mouse.x, (int32_t)mouse.y, &ui->route_goal);
            ui->has_route_goal = true;
            ui_set_status(ui, "Goal pin placed (Red). Calculating route...", UI_COLOR_ACCENT);

            if (ui->has_route_start) {
                ui->recompute_route = true;
            }
        }
        /* Normal Left Click: Map dragging */
        else if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            ui->mouse_dragging = true;
            ui->mouse_drag_start = mouse;
        }
    }

    if (ui->recompute_route) {
        ui->recompute_route = false;
        if (ui->has_route_start && ui->has_route_goal && pf && route) {
            pathfinder_set_profile(pf, ui->routing_profile);
            if (pathfinder_find_route(pf, ui->route_start, ui->route_goal, route)) {
                char msg[UI_STATUS_BUFFER_SIZE];
                const char *prof_label = (ui->routing_profile == PROFILE_STROLLER) ? "PEDESTRIAN" : "CAR";
                snprintf(msg, sizeof(msg), "[%s] Route: %.2f km, %d min (Mode %d)",
                         prof_label,
                         (double)route->total_distance_m / 1000.0,
                         route->total_time_sec / 60, route->routing_mode);
                ui_set_status(ui, msg, UI_COLOR_SUCCESS);
            } else {
                const char *prof_label = (ui->routing_profile == PROFILE_STROLLER) ? "PEDESTRIAN" : "CAR";
                char msg[UI_STATUS_BUFFER_SIZE];
                snprintf(msg, sizeof(msg), "[%s] No path found between selected pins.", prof_label);
                ui_set_status(ui, msg, UI_COLOR_ERROR);
            }
        }
    }

    if (ui->mouse_dragging) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 curr = GetMousePosition();
            float dx = ui->mouse_drag_start.x - curr.x;
            float dy = ui->mouse_drag_start.y - curr.y;
            map_camera_pan_pixels(cam, (int32_t)dx, (int32_t)dy);
            ui->mouse_drag_start = curr;
        } else {
            ui->mouse_dragging = false;
        }
    }

    /* Mouse Wheel Zoom */
    float wheel = GetMouseWheelMove();
    if (wheel > 0.1f) {
        for (int i = 0; i < ui->available_zoom_count; ++i) {
            if (ui->available_zooms[i] > cam->zoom) {
                map_camera_set_zoom(cam, ui->available_zooms[i]);
                break;
            }
        }
    } else if (wheel < -0.1f) {
        for (int i = ui->available_zoom_count - 1; i >= 0; --i) {
            if (ui->available_zooms[i] < cam->zoom) {
                map_camera_set_zoom(cam, ui->available_zooms[i]);
                break;
            }
        }
    }

    /* Keyboard zoom shortcuts: '+' and '-' */
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
        for (int i = 0; i < ui->available_zoom_count; ++i) {
            if (ui->available_zooms[i] > cam->zoom) {
                map_camera_set_zoom(cam, ui->available_zooms[i]);
                break;
            }
        }
    }
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
        for (int i = ui->available_zoom_count - 1; i >= 0; --i) {
            if (ui->available_zooms[i] < cam->zoom) {
                map_camera_set_zoom(cam, ui->available_zooms[i]);
                break;
            }
        }
    }

    /* Number shortcuts: 1->12, 2->14, 3->16 */
    if (!ui->lat_input.active && !ui->lon_input.active) {
        if (IsKeyPressed(KEY_ONE))   map_camera_set_zoom(cam, 12);
        if (IsKeyPressed(KEY_TWO))   map_camera_set_zoom(cam, 14);
        if (IsKeyPressed(KEY_THREE)) map_camera_set_zoom(cam, 16);
    }
}

void ui_draw(UIState *ui, const MapCamera *cam, const TileCache *cache, const RouteResult *route) {
    int screen_w = GetScreenWidth();
    int screen_h = GetScreenHeight();

    /* ============================================================== */
    /* 1. Main Navigation Control Panel (Left Sidebar)                */
    /* ============================================================== */
    Rectangle panel = { 10, 10, 320, 705 };
    DrawRectangleRounded(panel, 0.03f, 4, UI_COLOR_BG);
    DrawRectangleRoundedLines(panel, 0.03f, 4, UI_COLOR_BORDER);

    /* Panel Header */
    Rectangle header = { 10, 10, 320, 36 };
    DrawRectangleRounded(header, 0.03f, 4, UI_COLOR_PANEL_HEADER);
    DrawText("GPS EMBEDDED SIMULATION", 20, 20, 14, UI_COLOR_ACCENT);

    /* Section 1: Zoom Selection */
    DrawText("SELECT ZOOM LEVEL", 20, 54, 11, UI_COLOR_TEXT_MUTED);

    for (int i = 0; i < ui->available_zoom_count; ++i) {
        int z = ui->available_zooms[i];
        char label[16];
        snprintf(label, sizeof(label), "Z %d", z);
        Rectangle btn_rec = { 20 + i * 54, 68, 48, 26 };
        if (draw_button(btn_rec, label, cam->zoom == z, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
            ((MapCamera *)cam)->zoom = z;
            map_camera_set_zoom((MapCamera *)cam, z);
        }
    }

    /* Zoom step buttons [-] and [+] */
    Rectangle minus_rec = { 200, 68, 34, 26 };
    if (draw_button(minus_rec, "-", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        for (int i = ui->available_zoom_count - 1; i >= 0; --i) {
            if (ui->available_zooms[i] < cam->zoom) {
                map_camera_set_zoom((MapCamera *)cam, ui->available_zooms[i]);
                break;
            }
        }
    }

    Rectangle plus_rec = { 240, 68, 34, 26 };
    if (draw_button(plus_rec, "+", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        for (int i = 0; i < ui->available_zoom_count; ++i) {
            if (ui->available_zooms[i] > cam->zoom) {
                map_camera_set_zoom((MapCamera *)cam, ui->available_zooms[i]);
                break;
            }
        }
    }

    /* Section 2: Coordinate Inputs */
    draw_text_input(&ui->lat_input);
    draw_text_input(&ui->lon_input);

    /* Jump and Sync Buttons */
    Rectangle jump_rec = { 20, 148, 140, 26 };
    if (draw_button(jump_rec, "Center Map [Enter]", false, UI_COLOR_PRIMARY, UI_COLOR_PRIMARY_HOVER)) {
        apply_coordinate_jump(ui, (MapCamera *)cam);
    }

    Rectangle copy_rec = { 170, 148, 140, 26 };
    if (draw_button(copy_rec, "Get Current Center", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        snprintf(ui->lat_input.text, sizeof(ui->lat_input.text), "%.6f", (double)cam->center.lat_e7 / 10000000.0);
        ui->lat_input.cursor = (int)strlen(ui->lat_input.text);
        snprintf(ui->lon_input.text, sizeof(ui->lon_input.text), "%.6f", (double)cam->center.lon_e7 / 10000000.0);
        ui->lon_input.cursor = (int)strlen(ui->lon_input.text);
        ui_set_status(ui, "Copied current camera coordinate.", UI_COLOR_SUCCESS);
    }

    /* Section 3: Preset Locations */
    DrawText("LOCATION PRESETS (BELGIUM)", 20, 186, 11, UI_COLOR_TEXT_MUTED);
    int preset_y = 202;
    for (int i = 0; i < PRESET_COUNT; ++i) {
        Rectangle p_rec = { 20, (float)preset_y, 290, 22 };
        if (draw_button(p_rec, PRESETS[i].name, false, UI_COLOR_INPUT_BG, UI_COLOR_INPUT_ACTIVE)) {
            map_camera_set_center((MapCamera *)cam, geo_coord_make(PRESETS[i].lat_e7, PRESETS[i].lon_e7));
            snprintf(ui->lat_input.text, sizeof(ui->lat_input.text), "%.6f", (double)PRESETS[i].lat_e7 / 10000000.0);
            ui->lat_input.cursor = (int)strlen(ui->lat_input.text);
            snprintf(ui->lon_input.text, sizeof(ui->lon_input.text), "%.6f", (double)PRESETS[i].lon_e7 / 10000000.0);
            ui->lon_input.cursor = (int)strlen(ui->lon_input.text);

            char msg[UI_STATUS_BUFFER_SIZE];
            snprintf(msg, sizeof(msg), "Centered on %s", PRESETS[i].name);
            ui_set_status(ui, msg, UI_COLOR_SUCCESS);
        }
        preset_y += 26;
    }

    /* Section 4: ROUTE FINDING & GPS NAVIGATION */
    DrawText("ROUTING MODE [M KEY TO TOGGLE]", 20, (float)preset_y + 6, 11, UI_COLOR_TEXT_MUTED);
    preset_y += 22;

    Rectangle car_mode_btn = { 20, (float)preset_y, 140, 28 };
    if (draw_button(car_mode_btn, "CAR MODE", ui->routing_profile == PROFILE_CAR, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        if (ui->routing_profile != PROFILE_CAR) {
            ui->routing_profile = PROFILE_CAR;
            ui->recompute_route = true;
            ui_set_status(ui, "Switched to CAR routing mode.", UI_COLOR_PRIMARY_HOVER);
        }
    }

    Rectangle ped_mode_btn = { 170, (float)preset_y, 140, 28 };
    if (draw_button(ped_mode_btn, "PEDESTRIAN", ui->routing_profile == PROFILE_STROLLER, UI_COLOR_INPUT_ACTIVE, UI_COLOR_SUCCESS)) {
        if (ui->routing_profile != PROFILE_STROLLER) {
            ui->routing_profile = PROFILE_STROLLER;
            ui->recompute_route = true;
            ui_set_status(ui, "Switched to PEDESTRIAN / STROLLER routing mode.", UI_COLOR_SUCCESS);
        }
    }
    preset_y += 36;

    DrawText("ROUTE PLANNING (TRIP PRESETS)", 20, (float)preset_y, 11, UI_COLOR_TEXT_MUTED);
    preset_y += 16;

    /* Trip Preset 1: Liège Local (Mode A) */
    Rectangle trip1_rec = { 20, (float)preset_y, 290, 26 };
    if (draw_button(trip1_rec, "Trip 1: Liège Local (Guillemins->Lambert)", false, CLITERAL(Color){ 16, 80, 50, 255 }, UI_COLOR_SUCCESS)) {
        ui->route_start = geo_coord_make(506242000, 55666000);
        ui->has_route_start = true;
        ui->route_goal  = geo_coord_make(506454000, 55735000);
        ui->has_route_goal  = true;
        ui->recompute_route = true;

        map_camera_set_center((MapCamera *)cam, geo_coord_make(506348000, 55700000));
        map_camera_set_zoom((MapCamera *)cam, 14);
        ui_set_status(ui, "Liège Local pins set. Calculating...", UI_COLOR_SUCCESS);
    }
    preset_y += 30;

    /* Trip Preset 2: Liège -> Namur (Mode B Highway) */
    Rectangle trip2_rec = { 20, (float)preset_y, 290, 26 };
    if (draw_button(trip2_rec, "Trip 2: Liège -> Namur (E42 / Meuse)", false, CLITERAL(Color){ 20, 60, 110, 255 }, UI_COLOR_PRIMARY_HOVER)) {
        ui->route_start = geo_coord_make(506400000, 55000000);
        ui->has_route_start = true;
        ui->route_goal  = geo_coord_make(504800000, 48500000);
        ui->has_route_goal  = true;
        ui->recompute_route = true;

        map_camera_set_center((MapCamera *)cam, geo_coord_make(505600000, 51750000));
        map_camera_set_zoom((MapCamera *)cam, 12);
        ui_set_status(ui, "Liège -> Namur pins set. Calculating...", UI_COLOR_SUCCESS);
    }
    preset_y += 30;

    /* Trip Preset 3: Liège -> Brussels (Mode B Highway) */
    Rectangle trip3_rec = { 20, (float)preset_y, 290, 26 };
    if (draw_button(trip3_rec, "Trip 3: Liège -> Brussels", false, CLITERAL(Color){ 30, 45, 90, 255 }, UI_COLOR_PRIMARY_HOVER)) {
        ui->route_start = geo_coord_make(506400000, 55000000);
        ui->has_route_start = true;
        ui->route_goal  = geo_coord_make(508500000, 43500000); /* Brussels center */
        ui->has_route_goal  = true;
        ui->recompute_route = true;

        map_camera_set_center((MapCamera *)cam, geo_coord_make(507500000, 49200000));
        map_camera_set_zoom((MapCamera *)cam, 12);
        ui_set_status(ui, "Liège -> Brussels pins set. Calculating...", UI_COLOR_SUCCESS);
    }
    preset_y += 32;

    /* Pin Placement Instructions */
    DrawText("CUSTOM PINS (CLICK MAP):", 20, (float)preset_y, 11, UI_COLOR_TEXT_MUTED);
    preset_y += 16;
    DrawText("Shift + Click : Set Start Pin (Green)", 24, (float)preset_y, 12, UI_COLOR_SUCCESS);
    preset_y += 16;
    DrawText("Ctrl/R-Click  : Set Goal Pin (Red)", 24, (float)preset_y, 12, CLITERAL(Color){ 248, 113, 113, 255 });
    preset_y += 24;

    /* Clear Route Button */
    Rectangle clear_btn = { 20, (float)preset_y, 290, 26 };
    if (draw_button(clear_btn, "Clear Active Route", false, UI_COLOR_INPUT_BG, UI_COLOR_ERROR)) {
        ui->has_route_start = false;
        ui->has_route_goal = false;
        if (route) {
            pathfinder_free_route((RouteResult *)route);
        }
        ui_set_status(ui, "Route cleared.", UI_COLOR_TEXT_MUTED);
    }
    preset_y += 34;

    /* Camera D-Pad */
    int dpad_cx = 250;
    int dpad_cy = preset_y + 24;
    int btn_sz = 24;

    DrawText("CAMERA D-PAD", 20, (float)preset_y, 11, UI_COLOR_TEXT_MUTED);

    if (draw_button((Rectangle){ dpad_cx - btn_sz / 2.0f, dpad_cy - btn_sz * 1.5f, (float)btn_sz, (float)btn_sz }, "^", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        map_camera_pan_pixels((MapCamera *)cam, 0, -80);
    }
    if (draw_button((Rectangle){ dpad_cx - btn_sz / 2.0f, dpad_cy + btn_sz * 0.5f, (float)btn_sz, (float)btn_sz }, "v", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        map_camera_pan_pixels((MapCamera *)cam, 0, 80);
    }
    if (draw_button((Rectangle){ dpad_cx - btn_sz * 1.5f, dpad_cy - btn_sz / 2.0f, (float)btn_sz, (float)btn_sz }, "<", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        map_camera_pan_pixels((MapCamera *)cam, -80, 0);
    }
    if (draw_button((Rectangle){ dpad_cx + btn_sz * 0.5f, dpad_cy - btn_sz / 2.0f, (float)btn_sz, (float)btn_sz }, ">", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_PRIMARY)) {
        map_camera_pan_pixels((MapCamera *)cam, 80, 0);
    }

    if (draw_button((Rectangle){ 20, (float)dpad_cy - 12, 130, 26 }, "Reset Center", false, UI_COLOR_INPUT_ACTIVE, UI_COLOR_ACCENT)) {
        map_camera_set_center((MapCamera *)cam, geo_coord_make(506150000, 55750000));
        map_camera_set_zoom((MapCamera *)cam, 14);
        ui_set_status(ui, "Reset to Belgian map center (Z14)", UI_COLOR_SUCCESS);
    }

    /* ============================================================== */
    /* 2. Top-Right Telemetry & HUD Panel                             */
    /* ============================================================== */
    Rectangle hud_panel = { (float)screen_w - 290, 10, 280, 160 };
    DrawRectangleRounded(hud_panel, 0.04f, 4, UI_COLOR_BG);
    DrawRectangleRoundedLines(hud_panel, 0.04f, 4, UI_COLOR_BORDER);

    DrawText("GPS TELEMETRY (FIXED-POINT)", (int)hud_panel.x + 12, (int)hud_panel.y + 10, 12, UI_COLOR_ACCENT);

    char str_coord[64];
    snprintf(str_coord, sizeof(str_coord), "Lat: %.6f° N (e7: %d)", (double)cam->center.lat_e7 / 10000000.0, cam->center.lat_e7);
    DrawText(str_coord, (int)hud_panel.x + 12, (int)hud_panel.y + 28, 12, UI_COLOR_TEXT_MAIN);

    snprintf(str_coord, sizeof(str_coord), "Lon: %.6f° E (e7: %d)", (double)cam->center.lon_e7 / 10000000.0, cam->center.lon_e7);
    DrawText(str_coord, (int)hud_panel.x + 12, (int)hud_panel.y + 44, 12, UI_COLOR_TEXT_MAIN);

    TileCoord curr_tile;
    geo_latlon_to_tile(cam->center, cam->zoom, &curr_tile, NULL);
    char str_tile[64];
    snprintf(str_tile, sizeof(str_tile), "Tile: X=%d, Y=%d (Z%d)", curr_tile.x, curr_tile.y, cam->zoom);
    DrawText(str_tile, (int)hud_panel.x + 12, (int)hud_panel.y + 64, 12, UI_COLOR_TEXT_MUTED);

    char str_perf[80];
    snprintf(str_perf, sizeof(str_perf), "FPS: %d | Cache: %d/%d tiles", GetFPS(), cache->count, cache->capacity);
    DrawText(str_perf, (int)hud_panel.x + 12, (int)hud_panel.y + 82, 12, UI_COLOR_TEXT_MUTED);

    /* Bounding box indicator */
    GeoBBox tile_bbox = geo_bbox_make(505500000, 55000000, 506800000, 56500000);
    bool in_tiles = geo_coord_is_in_bbox(cam->center, tile_bbox);
    if (in_tiles) {
        DrawRectangle((int)hud_panel.x + 12, (int)hud_panel.y + 104, 8, 8, UI_COLOR_SUCCESS);
        DrawText("INSIDE TILE REGION (BELGIUM)", (int)hud_panel.x + 26, (int)hud_panel.y + 103, 11, UI_COLOR_SUCCESS);
    } else {
        DrawRectangle((int)hud_panel.x + 12, (int)hud_panel.y + 104, 8, 8, UI_COLOR_ERROR);
        DrawText("OUTSIDE TILE REGION", (int)hud_panel.x + 26, (int)hud_panel.y + 103, 11, UI_COLOR_ERROR);
    }

    DrawText("Memory: 100% Static (0 Malloc)", (int)hud_panel.x + 12, (int)hud_panel.y + 124, 11, UI_COLOR_ACCENT);
    DrawText("Arithmetic: Pure Fixed-Point", (int)hud_panel.x + 12, (int)hud_panel.y + 140, 11, UI_COLOR_SUCCESS);

    /* ============================================================== */
    /* 3. Route Telemetry HUD Card (Appears when route is active)     */
    /* ============================================================== */
    if (route && route->found && route->segment_count > 0) {
        Rectangle route_card = { (float)screen_w - 290, 180, 280, 168 };
        DrawRectangleRounded(route_card, 0.04f, 4, UI_COLOR_BG);
        bool is_ped = (route->routing_profile == PROFILE_STROLLER);
        Color border_col = is_ped
                           ? UI_COLOR_ACCENT
                           : ((route->routing_mode == 2) ? CLITERAL(Color){ 0, 180, 255, 255 } : UI_COLOR_SUCCESS);
        DrawRectangleRoundedLines(route_card, 0.04f, 4, border_col);

        /* Profile & Route Mode Header */
        const char *prof_str = is_ped ? "PROFILE: PEDESTRIAN / STROLLER" : "PROFILE: CAR NAVIGATION";
        const char *mode_str = (route->routing_mode == 2)
                               ? (is_ped ? "MODE B: STROLLER BACKBONE" : "MODE B: HIGHWAY BACKBONE")
                               : "MODE A: LOCAL MULTI-TILE";
        Color mode_col = is_ped
                         ? UI_COLOR_ACCENT
                         : ((route->routing_mode == 2) ? CLITERAL(Color){ 0, 220, 255, 255 } : UI_COLOR_SUCCESS);

        DrawText(prof_str, (int)route_card.x + 12, (int)route_card.y + 10, 11, mode_col);
        DrawText(mode_str, (int)route_card.x + 12, (int)route_card.y + 26, 13, UI_COLOR_TEXT_MAIN);

        /* Distance & Time */
        char dist_buf[64];
        if (route->total_distance_m >= 1000) {
            snprintf(dist_buf, sizeof(dist_buf), "Distance: %.2f km", (double)route->total_distance_m / 1000.0);
        } else {
            snprintf(dist_buf, sizeof(dist_buf), "Distance: %u m", route->total_distance_m);
        }
        DrawText(dist_buf, (int)route_card.x + 12, (int)route_card.y + 50, 14, UI_COLOR_TEXT_MAIN);

        char time_buf[64];
        uint32_t hours = route->total_time_sec / 3600;
        uint32_t mins = (route->total_time_sec % 3600) / 60;
        uint32_t secs = route->total_time_sec % 60;
        if (hours > 0) {
            snprintf(time_buf, sizeof(time_buf), "Est. Time: %u h %u min", hours, mins);
        } else {
            snprintf(time_buf, sizeof(time_buf), "Est. Time: %u min %u sec", mins, secs);
        }
        DrawText(time_buf, (int)route_card.x + 12, (int)route_card.y + 70, 14, UI_COLOR_TEXT_MAIN);

        /* Segments and Curve Points loaded from *_geom.bin */
        char seg_buf[64];
        snprintf(seg_buf, sizeof(seg_buf), "Road Segments: %d", route->segment_count);
        DrawText(seg_buf, (int)route_card.x + 12, (int)route_card.y + 96, 12, UI_COLOR_TEXT_MUTED);

        char geom_buf[64];
        snprintf(geom_buf, sizeof(geom_buf), "Curve Geometry: %d points (*_geom.bin)", route->geom_point_count);
        DrawText(geom_buf, (int)route_card.x + 12, (int)route_card.y + 114, 12, CLITERAL(Color){ 100, 220, 255, 255 });
    }

    /* ============================================================== */
    /* 4. Bottom Controls Cheat Sheet Bar                             */
    /* ============================================================== */
    Rectangle bar = { 10, (float)screen_h - 30, (float)screen_w - 20, 22 };
    DrawRectangleRounded(bar, 0.2f, 4, CLITERAL(Color){ 15, 20, 25, 210 });
    DrawText("[M] Car/Pedestrian Mode  |  [Shift+Click] Start  |  [Ctrl/Right-Click] Goal  |  [Arrows/WASD] Pan",
             20, screen_h - 25, 12, UI_COLOR_TEXT_MUTED);

    /* ============================================================== */
    /* 5. Notification Toast Status Message                           */
    /* ============================================================== */
    if (ui->status_timer > 0.0f) {
        float alpha = (ui->status_timer < 1.0f) ? ui->status_timer : 1.0f;
        Color bg = CLITERAL(Color){ 15, 23, 42, (unsigned char)(230.0f * alpha) };
        Color txt = CLITERAL(Color){ ui->status_color.r, ui->status_color.g, ui->status_color.b, (unsigned char)(255.0f * alpha) };

        int msg_w = MeasureText(ui->status_message, 14);
        int toast_x = (screen_w - msg_w) / 2 - 16;
        int toast_y = 20;

        DrawRectangleRounded((Rectangle){ (float)toast_x, (float)toast_y, (float)msg_w + 32, 32 }, 0.3f, 4, bg);
        DrawRectangleRoundedLines((Rectangle){ (float)toast_x, (float)toast_y, (float)msg_w + 32, 32 }, 0.3f, 4, txt);
        DrawText(ui->status_message, toast_x + 16, toast_y + 8, 14, txt);
    }
}

void ui_draw_route(const RouteResult *route, const MapCamera *cam) {
    if (!route || !route->found || route->geom_point_count < 2) return;

    bool is_ped = (route->routing_profile == PROFILE_STROLLER);
    Color glow_col = is_ped
                     ? CLITERAL(Color){ 245, 158, 11, 190 }    /* Pedestrian/Stroller: Warm Amber glow */
                     : ((route->routing_mode == 2)
                        ? CLITERAL(Color){ 0, 140, 255, 180 }   /* Car Highway: Blue/Cyan glow */
                        : CLITERAL(Color){ 16, 185, 129, 180 }); /* Car Local: Emerald Green glow */
    Color core_col = is_ped
                     ? CLITERAL(Color){ 255, 235, 160, 255 }
                     : ((route->routing_mode == 2)
                        ? CLITERAL(Color){ 120, 230, 255, 255 }
                        : CLITERAL(Color){ 180, 255, 220, 255 });

    int screen_w = cam->viewport_width;
    int screen_h = cam->viewport_height;

    /* Draw continuous curve points loaded from *_geom.bin */
    for (int32_t i = 0; i < route->geom_point_count - 1; ++i) {
        GeoCoord p1 = geo_coord_make(route->geom_points[i].lat_scaled, route->geom_points[i].lon_scaled);
        GeoCoord p2 = geo_coord_make(route->geom_points[i + 1].lat_scaled, route->geom_points[i + 1].lon_scaled);

        int32_t sx1, sy1, sx2, sy2;
        map_camera_latlon_to_screen(cam, p1, &sx1, &sy1);
        map_camera_latlon_to_screen(cam, p2, &sx2, &sy2);

        /* Viewport culling */
        if ((sx1 < -100 && sx2 < -100) || (sx1 > screen_w + 100 && sx2 > screen_w + 100) ||
            (sy1 < -100 && sy2 < -100) || (sy1 > screen_h + 100 && sy2 > screen_h + 100)) {
            continue;
        }

        Vector2 v1 = { (float)sx1, (float)sy1 };
        Vector2 v2 = { (float)sx2, (float)sy2 };

        DrawLineEx(v1, v2, 6.0f, glow_col);
        DrawLineEx(v1, v2, 3.0f, core_col);
    }
}

void ui_draw_pins(const UIState *ui, const MapCamera *cam) {
    if (ui->has_route_start) {
        int32_t sx, sy;
        map_camera_latlon_to_screen(cam, ui->route_start, &sx, &sy);
        DrawCircle(sx, sy, 10.0f, CLITERAL(Color){ 16, 185, 129, 255 });
        DrawCircleLines(sx, sy, 12.0f, WHITE);
        DrawCircle(sx, sy, 4.0f, WHITE);
        DrawText("START", sx - 18, sy - 24, 11, CLITERAL(Color){ 16, 185, 129, 255 });
    }

    if (ui->has_route_goal) {
        int32_t sx, sy;
        map_camera_latlon_to_screen(cam, ui->route_goal, &sx, &sy);
        DrawCircle(sx, sy, 10.0f, CLITERAL(Color){ 239, 68, 68, 255 });
        DrawCircleLines(sx, sy, 12.0f, WHITE);
        DrawCircle(sx, sy, 4.0f, WHITE);
        DrawText("GOAL", sx - 16, sy - 24, 11, CLITERAL(Color){ 248, 113, 113, 255 });
    }
}

void ui_draw_crosshair(const MapCamera *cam) {
    int cx = cam->viewport_width / 2;
    int cy = cam->viewport_height / 2;

    Color reticle_col = CLITERAL(Color){ 245, 158, 11, 220 }; /* Amber accent */
    Color shadow_col  = CLITERAL(Color){ 0, 0, 0, 160 };

    /* Drop shadow */
    DrawCircleLines(cx + 1, cy + 1, 14.0f, shadow_col);
    DrawLine(cx - 21, cy + 1, cx - 6, cy + 1, shadow_col);
    DrawLine(cx + 7, cy + 1, cx + 22, cy + 1, shadow_col);
    DrawLine(cx + 1, cy - 21, cx + 1, cy - 6, shadow_col);
    DrawLine(cx + 1, cy + 7, cx + 1, cy + 22, shadow_col);

    /* Reticle */
    DrawCircleLines(cx, cy, 14.0f, reticle_col);
    DrawCircle(cx, cy, 2.5f, reticle_col);
    DrawLine(cx - 22, cy, cx - 6, cy, reticle_col);
    DrawLine(cx + 6, cy, cx + 22, cy, reticle_col);
    DrawLine(cx, cy - 22, cx, cy - 6, reticle_col);
    DrawLine(cx, cy + 6, cx, cy + 22, reticle_col);
}
