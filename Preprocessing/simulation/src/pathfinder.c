/**
 * @file pathfinder.c
 * @brief Hybrid A* pathfinding engine implementation.
 *
 * Mode A: Local Multi-Tile A* for short trips (< 15 km or adjacent tiles).
 * Mode B: Hierarchical 3-Phase A* (Local -> Backbone Highway -> Local) for long trips.
 *
 * NOTE: 100% static allocation. Zero malloc, free, or realloc. Embedded safe.
 */

#include "pathfinder.h"
#include "astar_heap.h"
#include <string.h>
#include <stdio.h>

#define MAX_SESSION_TILES TILE_GRAPH_CACHE_MAX_LOCAL
#define INF_COST 0x3FFFFFFF

/* Pre-allocated static workspace for A* search in .bss */
static uint32_t s_local_g_scores[MAX_SESSION_TILES][TILE_MAX_LOCAL_NODES];
static uint32_t s_local_came_from_keys[MAX_SESSION_TILES][TILE_MAX_LOCAL_NODES];
static uint32_t s_local_came_from_edges[MAX_SESSION_TILES][TILE_MAX_LOCAL_NODES];

static uint32_t s_bb_g_scores[TILE_MAX_BACKBONE_NODES];
static uint32_t s_bb_came_from_nodes[TILE_MAX_BACKBONE_NODES];
static uint32_t s_bb_came_from_edges[TILE_MAX_BACKBONE_NODES];

static RouteSegment s_temp_segs[ROUTE_MAX_SEGMENTS];
static AstarHeap s_astar_heap;

typedef struct {
    TileGraph *graph;
    int32_t grid_lat;
    int32_t grid_lon;
    uint32_t *g_scores;
    uint32_t *came_from_keys;
    uint32_t *came_from_edges;
} SessionTile;

static inline uint32_t make_node_key(int32_t slot, uint32_t local_idx) {
    return (((uint32_t)slot & 0x0F) << 20) | (local_idx & 0x000FFFFF);
}

static inline int32_t key_to_slot(uint32_t key) {
    return (int32_t)((key >> 20) & 0x0F);
}

static inline uint32_t key_to_idx(uint32_t key) {
    return (key & 0x000FFFFF);
}

static inline bool edge_allowed_for_profile(const EdgeRecord *edge, RoutingProfile profile) {
    if (edge->edge_flags == 0) {
        /* Legacy binary format (before edge_flags were populated): allow in both modes */
        return true;
    }
    uint16_t req = (profile == PROFILE_STROLLER) ? EDGE_STROLLER_ALLOWED : EDGE_CAR_ALLOWED;
    return (edge->edge_flags & req) != 0;
}

static inline uint32_t edge_search_cost(const EdgeRecord *edge, RoutingProfile profile) {
    if (profile == PROFILE_STROLLER) {
        uint32_t penalty_q3 = (uint32_t)(edge->edge_flags >> 8);
        if (penalty_q3 < 8) penalty_q3 = 8;
        /* Base walking speed: 1.25 m/s (4/5 sec per meter) * (penalty_q3 / 8) = length_m * penalty_q3 / 10 */
        uint32_t c = ((uint32_t)edge->length_m * penalty_q3) / 10;
        return (c > 0) ? c : 1;
    }
    return (edge->cost_sec > 0) ? edge->cost_sec : 1;
}

static inline uint16_t edge_travel_time_sec(const EdgeRecord *edge, RoutingProfile profile) {
    if (profile == PROFILE_STROLLER) {
        /* Actual walking time at 1.25 m/s (4.5 km/h) */
        uint32_t sec = ((uint32_t)edge->length_m * 4) / 5;
        if (sec < 1) sec = 1;
        if (sec > 65535) sec = 65535;
        return (uint16_t)sec;
    }
    return edge->cost_sec;
}

static inline uint32_t heuristic_for_profile(GeoCoord a, GeoCoord b, RoutingProfile profile, bool is_backbone) {
    uint32_t d = geo_distance_m(a, b);
    if (profile == PROFILE_STROLLER) {
        return (d * 4) / 5;
    }
    return d / (is_backbone ? 33 : 25);
}

bool pathfinder_init(Pathfinder *pf, TileGraphCache *cache) {
    if (!pf || !cache) return false;
    pf->cache = cache;
    pf->profile = PROFILE_CAR;
    return true;
}

void pathfinder_set_profile(Pathfinder *pf, RoutingProfile profile) {
    if (!pf) return;
    pf->profile = profile;
}

void pathfinder_destroy(Pathfinder *pf) {
    if (pf) pf->cache = NULL;
}

void pathfinder_free_route(RouteResult *route) {
    if (!route) return;
    route->segment_count = 0;
    route->geom_point_count = 0;
    route->total_distance_m = 0;
    route->total_time_sec = 0;
    route->found = false;
    route->routing_mode = 0;
    route->routing_profile = 0;
}

static bool route_add_segment(RouteResult *route, const RouteSegment *seg) {
    if (!route || !seg) return false;
    if (route->segment_count >= ROUTE_MAX_SEGMENTS) {
        return false;
    }
    route->segments[route->segment_count++] = *seg;
    route->total_distance_m += seg->length_m;
    route->total_time_sec += seg->cost_sec;
    return true;
}

/* ========================================================================= */
/* Mode A: Local Multi-Tile A*                                               */
/* ========================================================================= */
static bool run_local_astar(Pathfinder *pf, GeoCoord start, GeoCoord goal,
                            int32_t start_glat, int32_t start_glon,
                            int32_t goal_glat, int32_t goal_glon,
                            RouteResult *out_route) {
    TileGraph *start_graph = tile_graph_cache_get_tile(pf->cache, start_glat, start_glon);
    if (!start_graph || start_graph->node_count == 0) return false;

    TileGraph *goal_graph = (start_glat == goal_glat && start_glon == goal_glon)
                            ? start_graph
                            : tile_graph_cache_get_tile(pf->cache, goal_glat, goal_glon);
    if (!goal_graph || goal_graph->node_count == 0) return false;

    int32_t start_node_idx = -1;
    int32_t goal_node_idx = -1;
    uint32_t dist_snap;

    if (!tile_graph_snap_node_for_profile(start_graph, start.lat_e7, start.lon_e7, 1, pf->profile, &start_node_idx, &dist_snap)) {
        return false;
    }
    if (!tile_graph_snap_node_for_profile(goal_graph, goal.lat_e7, goal.lon_e7, 1, pf->profile, &goal_node_idx, &dist_snap)) {
        return false;
    }

    SessionTile session[MAX_SESSION_TILES];
    int32_t session_count = 0;

    /* Slot 0 = Start Tile */
    tile_graph_pin(start_graph);
    session[0].graph = start_graph;
    session[0].grid_lat = start_glat;
    session[0].grid_lon = start_glon;
    session[0].g_scores = s_local_g_scores[0];
    session[0].came_from_keys = s_local_came_from_keys[0];
    session[0].came_from_edges = s_local_came_from_edges[0];
    memset(session[0].g_scores, 0xFF, (size_t)start_graph->node_count * sizeof(uint32_t));
    session_count = 1;

    /* Slot 1 = Goal Tile (if different) */
    int32_t goal_slot = 0;
    if (goal_graph != start_graph) {
        tile_graph_pin(goal_graph);
        session[1].graph = goal_graph;
        session[1].grid_lat = goal_glat;
        session[1].grid_lon = goal_glon;
        session[1].g_scores = s_local_g_scores[1];
        session[1].came_from_keys = s_local_came_from_keys[1];
        session[1].came_from_edges = s_local_came_from_edges[1];
        memset(session[1].g_scores, 0xFF, (size_t)goal_graph->node_count * sizeof(uint32_t));
        goal_slot = 1;
        session_count = 2;
    }

    uint32_t start_key = make_node_key(0, (uint32_t)start_node_idx);
    uint32_t target_goal_key = make_node_key(goal_slot, (uint32_t)goal_node_idx);

    session[0].g_scores[start_node_idx] = 0;

    astar_heap_init(&s_astar_heap);

    uint32_t initial_h = heuristic_for_profile(start, goal, pf->profile, false);
    astar_heap_push(&s_astar_heap, start_key, 0, initial_h);

    bool goal_reached = false;
    uint32_t winning_key = 0;

    while (!astar_heap_is_empty(&s_astar_heap)) {
        HeapEntry top;
        astar_heap_pop(&s_astar_heap, &top);

        int32_t u_slot = key_to_slot(top.node_key);
        uint32_t u_idx = key_to_idx(top.node_key);

        if (top.g_cost > session[u_slot].g_scores[u_idx]) {
            continue;
        }

        if (top.node_key == target_goal_key) {
            goal_reached = true;
            winning_key = target_goal_key;
            break;
        }

        const TileGraph *u_graph = session[u_slot].graph;
        const NodeRecord *u_node = &u_graph->nodes[u_idx];

        for (uint16_t e = 0; e < u_node->edge_count; ++e) {
            uint32_t edge_idx = u_node->first_edge + e;
            if (edge_idx >= u_graph->edge_count) continue;
            const EdgeRecord *edge = &u_graph->edges[edge_idx];
            if (!edge_allowed_for_profile(edge, pf->profile)) continue;

            uint32_t v_idx = edge->target_idx;
            if (v_idx >= u_graph->node_count) continue;

            const NodeRecord *v_node = &u_graph->nodes[v_idx];
            int32_t v_slot = u_slot;
            uint32_t v_local_idx = v_idx;

            /* Check if edge steps into a neighboring tile */
            int32_t v_glat, v_glon;
            geo_coord_to_grid(v_node->lat_scaled, v_node->lon_scaled, &v_glat, &v_glon);

            if (v_glat != session[u_slot].grid_lat || v_glon != session[u_slot].grid_lon) {
                int32_t found_slot = -1;
                for (int32_t s = 0; s < session_count; ++s) {
                    if (session[s].grid_lat == v_glat && session[s].grid_lon == v_glon) {
                        found_slot = s;
                        break;
                    }
                }

                if (found_slot < 0) {
                    if (session_count < MAX_SESSION_TILES) {
                        TileGraph *neighbor_graph = tile_graph_cache_get_tile(pf->cache, v_glat, v_glon);
                        if (neighbor_graph && neighbor_graph->node_count > 0) {
                            tile_graph_pin(neighbor_graph);
                            found_slot = session_count++;
                            session[found_slot].graph = neighbor_graph;
                            session[found_slot].grid_lat = v_glat;
                            session[found_slot].grid_lon = v_glon;
                            session[found_slot].g_scores = s_local_g_scores[found_slot];
                            session[found_slot].came_from_keys = s_local_came_from_keys[found_slot];
                            session[found_slot].came_from_edges = s_local_came_from_edges[found_slot];
                            memset(session[found_slot].g_scores, 0xFF, (size_t)neighbor_graph->node_count * sizeof(uint32_t));
                        }
                    }
                }

                if (found_slot >= 0) {
                    int32_t matched_idx = -1;
                    if (tile_graph_find_matching_node(session[found_slot].graph, v_node->lat_scaled, v_node->lon_scaled, &matched_idx)) {
                        v_slot = found_slot;
                        v_local_idx = (uint32_t)matched_idx;
                    }
                }
            }

            uint32_t step_cost = edge_search_cost(edge, pf->profile);
            uint32_t tentative_g = top.g_cost + step_cost;
            if (tentative_g < session[v_slot].g_scores[v_local_idx]) {
                session[v_slot].g_scores[v_local_idx] = tentative_g;
                session[v_slot].came_from_keys[v_local_idx] = top.node_key;
                session[v_slot].came_from_edges[v_local_idx] = edge_idx;

                GeoCoord v_coord = geo_coord_make(v_node->lat_scaled, v_node->lon_scaled);
                uint32_t h = heuristic_for_profile(v_coord, goal, pf->profile, false);
                uint32_t v_key = make_node_key(v_slot, v_local_idx);
                astar_heap_push(&s_astar_heap, v_key, tentative_g, tentative_g + h);
            }
        }
    }

    if (!goal_reached) {
        for (int k = 0; k < session_count; ++k) {
            tile_graph_unpin(session[k].graph);
        }
        return false;
    }

    /* Path reconstruction into s_temp_segs */
    int32_t seg_count = 0;
    uint32_t curr_key = winning_key;

    while (curr_key != start_key && seg_count < ROUTE_MAX_SEGMENTS) {
        int32_t c_slot = key_to_slot(curr_key);
        uint32_t c_idx = key_to_idx(curr_key);

        uint32_t p_key = session[c_slot].came_from_keys[c_idx];
        uint32_t e_idx = session[c_slot].came_from_edges[c_idx];

        int32_t p_slot = key_to_slot(p_key);
        uint32_t p_idx = key_to_idx(p_key);

        const TileGraph *p_graph = session[p_slot].graph;
        const NodeRecord *p_node = &p_graph->nodes[p_idx];
        const EdgeRecord *edge = &p_graph->edges[e_idx];
        const NodeRecord *end_node = &p_graph->nodes[edge->target_idx];

        RouteSegment *seg = &s_temp_segs[seg_count++];
        seg->start_lat_e7 = p_node->lat_scaled;
        seg->start_lon_e7 = p_node->lon_scaled;
        seg->end_lat_e7 = end_node->lat_scaled;
        seg->end_lon_e7 = end_node->lon_scaled;
        seg->length_m = edge->length_m;
        seg->cost_sec = edge_travel_time_sec(edge, pf->profile);
        seg->street_id = edge->street_id;
        seg->geom_offset = edge->geom_offset;
        seg->geom_count = edge->geom_count;
        seg->geom_path = p_graph->geom_path;

        curr_key = p_key;
    }

    /* Reverse segments into out_route */
    for (int32_t i = seg_count - 1; i >= 0; --i) {
        route_add_segment(out_route, &s_temp_segs[i]);
    }

    for (int k = 0; k < session_count; ++k) {
        tile_graph_unpin(session[k].graph);
    }

    out_route->found = true;
    out_route->routing_mode = 1;
    out_route->routing_profile = (int)pf->profile;
    return true;
}

/* ========================================================================= */
/* Mode B: Backbone Transit A*                                               */
/* ========================================================================= */
static bool run_backbone_astar(Pathfinder *pf, int32_t start_bb_node, int32_t goal_bb_node, RouteResult *out_route) {
    TileGraph *bb = tile_graph_cache_get_backbone_for_profile(pf->cache, pf->profile);
    if (!bb || !bb->loaded || bb->node_count == 0) return false;

    if (start_bb_node == goal_bb_node) return true;

    memset(s_bb_g_scores, 0xFF, (size_t)bb->node_count * sizeof(uint32_t));
    s_bb_g_scores[start_bb_node] = 0;

    GeoCoord goal_coord = geo_coord_make(bb->nodes[goal_bb_node].lat_scaled, bb->nodes[goal_bb_node].lon_scaled);

    astar_heap_init(&s_astar_heap);

    GeoCoord start_coord = geo_coord_make(bb->nodes[start_bb_node].lat_scaled, bb->nodes[start_bb_node].lon_scaled);
    uint32_t init_h = heuristic_for_profile(start_coord, goal_coord, pf->profile, true);
    astar_heap_push(&s_astar_heap, (uint32_t)start_bb_node, 0, init_h);

    bool reached = false;

    while (!astar_heap_is_empty(&s_astar_heap)) {
        HeapEntry top;
        astar_heap_pop(&s_astar_heap, &top);

        uint32_t u = top.node_key;
        if (top.g_cost > s_bb_g_scores[u]) continue;

        if (u == (uint32_t)goal_bb_node) {
            reached = true;
            break;
        }

        const NodeRecord *u_node = &bb->nodes[u];
        for (uint16_t e = 0; e < u_node->edge_count; ++e) {
            uint32_t edge_idx = u_node->first_edge + e;
            if (edge_idx >= bb->edge_count) continue;
            const EdgeRecord *edge = &bb->edges[edge_idx];
            if (!edge_allowed_for_profile(edge, pf->profile)) continue;

            uint32_t v = edge->target_idx;
            if (v >= bb->node_count) continue;

            uint32_t step_cost = edge_search_cost(edge, pf->profile);
            uint32_t tentative_g = top.g_cost + step_cost;
            if (tentative_g < s_bb_g_scores[v]) {
                s_bb_g_scores[v] = tentative_g;
                s_bb_came_from_nodes[v] = u;
                s_bb_came_from_edges[v] = edge_idx;

                GeoCoord v_coord = geo_coord_make(bb->nodes[v].lat_scaled, bb->nodes[v].lon_scaled);
                uint32_t h = heuristic_for_profile(v_coord, goal_coord, pf->profile, true);
                astar_heap_push(&s_astar_heap, v, tentative_g, tentative_g + h);
            }
        }
    }

    if (!reached) {
        return false;
    }

    /* Reconstruct backbone route into s_temp_segs */
    int32_t seg_count = 0;
    uint32_t curr = (uint32_t)goal_bb_node;

    while (curr != (uint32_t)start_bb_node && seg_count < ROUTE_MAX_SEGMENTS) {
        uint32_t p = s_bb_came_from_nodes[curr];
        uint32_t e_idx = s_bb_came_from_edges[curr];

        const NodeRecord *p_node = &bb->nodes[p];
        const EdgeRecord *edge = &bb->edges[e_idx];
        const NodeRecord *end_node = &bb->nodes[edge->target_idx];

        RouteSegment *seg = &s_temp_segs[seg_count++];
        seg->start_lat_e7 = p_node->lat_scaled;
        seg->start_lon_e7 = p_node->lon_scaled;
        seg->end_lat_e7 = end_node->lat_scaled;
        seg->end_lon_e7 = end_node->lon_scaled;
        seg->length_m = edge->length_m;
        seg->cost_sec = edge_travel_time_sec(edge, pf->profile);
        seg->street_id = edge->street_id;
        seg->geom_offset = edge->geom_offset;
        seg->geom_count = edge->geom_count;
        seg->geom_path = bb->geom_path;

        curr = p;
    }

    for (int32_t i = seg_count - 1; i >= 0; --i) {
        route_add_segment(out_route, &s_temp_segs[i]);
    }

    return true;
}

/* ========================================================================= */
/* Mode B: Full 3-Phase Hierarchical A*                                      */
/* ========================================================================= */
static bool run_hierarchical_astar(Pathfinder *pf, GeoCoord start, GeoCoord goal,
                                   int32_t start_glat, int32_t start_glon,
                                   int32_t goal_glat, int32_t goal_glon,
                                   RouteResult *out_route) {
    TileGraph *start_tile = tile_graph_cache_get_tile(pf->cache, start_glat, start_glon);
    TileGraph *goal_tile  = tile_graph_cache_get_tile(pf->cache, goal_glat, goal_glon);
    TileGraph *bb         = tile_graph_cache_get_backbone_for_profile(pf->cache, pf->profile);

    if (!start_tile || !goal_tile || !bb) return false;

    tile_graph_pin(start_tile);
    tile_graph_pin(goal_tile);
    tile_graph_pin(bb);

    /* 1. Find Entry Gateway near origin */
    int32_t gw_start_local = -1;
    uint32_t gw_start_dist = 0;
    if (!tile_graph_find_gateway_for_profile(start_tile, start.lat_e7, start.lon_e7, pf->profile, &gw_start_local, &gw_start_dist)) {
        int32_t bb_snap = -1;
        tile_graph_snap_node_for_profile(bb, start.lat_e7, start.lon_e7, 1, pf->profile, &bb_snap, &gw_start_dist);
        gw_start_local = -1;
    }

    /* 2. Find Exit Gateway near destination */
    int32_t gw_goal_local = -1;
    uint32_t gw_goal_dist = 0;
    if (!tile_graph_find_gateway_for_profile(goal_tile, goal.lat_e7, goal.lon_e7, pf->profile, &gw_goal_local, &gw_goal_dist)) {
        gw_goal_local = -1;
    }

    int32_t bb_start_node = -1;
    int32_t bb_goal_node = -1;

    if (gw_start_local >= 0) {
        int32_t lat_s = start_tile->nodes[gw_start_local].lat_scaled;
        int32_t lon_s = start_tile->nodes[gw_start_local].lon_scaled;
        tile_graph_find_matching_node(bb, lat_s, lon_s, &bb_start_node);
        if (bb_start_node >= 0 && bb->nodes[bb_start_node].edge_count == 0) {
            bb_start_node = -1;
        }
    }
    if (bb_start_node < 0) {
        tile_graph_snap_node_for_profile(bb, start.lat_e7, start.lon_e7, 1, pf->profile, &bb_start_node, NULL);
    }

    if (gw_goal_local >= 0) {
        int32_t lat_s = goal_tile->nodes[gw_goal_local].lat_scaled;
        int32_t lon_s = goal_tile->nodes[gw_goal_local].lon_scaled;
        tile_graph_find_matching_node(bb, lat_s, lon_s, &bb_goal_node);
    }
    if (bb_goal_node < 0) {
        tile_graph_snap_node_for_profile(bb, goal.lat_e7, goal.lon_e7, 1, pf->profile, &bb_goal_node, NULL);
    }

    if (bb_start_node < 0 || bb_goal_node < 0 || bb_start_node == bb_goal_node) {
        tile_graph_unpin(start_tile);
        tile_graph_unpin(goal_tile);
        tile_graph_unpin(bb);
        return false;
    }

    /* Phase 1: Local Origin -> Entry Gateway */
    GeoCoord gw_start_coord = geo_coord_make(bb->nodes[bb_start_node].lat_scaled, bb->nodes[bb_start_node].lon_scaled);
    uint32_t start_to_gw_m = geo_distance_m(start, gw_start_coord);
    bool p1_ok = true;
    if (start_to_gw_m > 50) {
        p1_ok = run_local_astar(pf, start, gw_start_coord, start_glat, start_glon, start_glat, start_glon, out_route);
    }
    if (!p1_ok && start_to_gw_m > 250) {
        pathfinder_free_route(out_route);
        tile_graph_unpin(start_tile);
        tile_graph_unpin(goal_tile);
        tile_graph_unpin(bb);
        return false;
    }

    /* Phase 2: Backbone Transit */
    int32_t segs_before_bb = out_route->segment_count;
    bool bb_ok = run_backbone_astar(pf, bb_start_node, bb_goal_node, out_route);
    if (!bb_ok || out_route->segment_count == segs_before_bb) {
        pathfinder_free_route(out_route);
        tile_graph_unpin(start_tile);
        tile_graph_unpin(goal_tile);
        tile_graph_unpin(bb);
        return false;
    }

    /* Phase 3: Exit Gateway -> Local Destination */
    GeoCoord gw_goal_coord = geo_coord_make(bb->nodes[bb_goal_node].lat_scaled, bb->nodes[bb_goal_node].lon_scaled);
    uint32_t gw_to_goal_m = geo_distance_m(gw_goal_coord, goal);
    bool p3_ok = true;
    if (gw_to_goal_m > 50) {
        p3_ok = run_local_astar(pf, gw_goal_coord, goal, goal_glat, goal_glon, goal_glat, goal_glon, out_route);
    }
    if (!p3_ok && gw_to_goal_m > 250) {
        pathfinder_free_route(out_route);
        tile_graph_unpin(start_tile);
        tile_graph_unpin(goal_tile);
        tile_graph_unpin(bb);
        return false;
    }

    tile_graph_unpin(start_tile);
    tile_graph_unpin(goal_tile);
    tile_graph_unpin(bb);

    out_route->found = (out_route->segment_count > 0);
    out_route->routing_mode = 2;
    out_route->routing_profile = (int)pf->profile;
    return out_route->found;
}

bool pathfinder_find_route(Pathfinder *pf, GeoCoord start, GeoCoord goal, RouteResult *out_route) {
    if (!pf || !out_route) return false;

    pathfinder_free_route(out_route);

    int32_t start_glat, start_glon, goal_glat, goal_glon;
    geo_coord_to_grid(start.lat_e7, start.lon_e7, &start_glat, &start_glon);
    geo_coord_to_grid(goal.lat_e7, goal.lon_e7, &goal_glat, &goal_glon);

    uint32_t dist_m = geo_distance_m(start, goal);
    bool adjacent = is_adjacent_grid(start_glat, start_glon, goal_glat, goal_glon);

    /* Threshold check: < 15 km or adjacent tile -> Mode A */
    if (dist_m < 15000 || adjacent) {
        if (run_local_astar(pf, start, goal, start_glat, start_glon, goal_glat, goal_glon, out_route)) {
            pathfinder_load_geometry(pf, out_route);
            return true;
        }
    }

    /* Mode B: Hierarchical 3-Phase A* */
    if (run_hierarchical_astar(pf, start, goal, start_glat, start_glon, goal_glat, goal_glon, out_route)) {
        pathfinder_load_geometry(pf, out_route);
        return true;
    }

    /* Fallback: try pure backbone snap only if pins are near backbone nodes */
    TileGraph *bb = tile_graph_cache_get_backbone_for_profile(pf->cache, pf->profile);
    if (bb) {
        int32_t s_idx = -1, g_idx = -1;
        uint32_t s_dist = 0xFFFFFFFF, g_dist = 0xFFFFFFFF;
        tile_graph_snap_node_for_profile(bb, start.lat_e7, start.lon_e7, 1, pf->profile, &s_idx, &s_dist);
        tile_graph_snap_node_for_profile(bb, goal.lat_e7, goal.lon_e7, 1, pf->profile, &g_idx, &g_dist);
        if (s_idx >= 0 && g_idx >= 0 && s_idx != g_idx && s_dist <= 300 && g_dist <= 300) {
            if (run_backbone_astar(pf, s_idx, g_idx, out_route) && out_route->segment_count > 0) {
                out_route->found = true;
                out_route->routing_mode = 2;
                out_route->routing_profile = (int)pf->profile;
                pathfinder_load_geometry(pf, out_route);
                return true;
            }
        }
    }

    pathfinder_free_route(out_route);
    return false;
}

bool pathfinder_load_geometry(Pathfinder *pf, RouteResult *out_route) {
    (void)pf;
    if (!out_route || out_route->segment_count == 0) return false;

    out_route->geom_point_count = 0;

    const char *current_open_path = NULL;
    FILE *f_geom = NULL;

    for (int32_t i = 0; i < out_route->segment_count; ++i) {
        const RouteSegment *seg = &out_route->segments[i];

        /* Append start point */
        if (out_route->geom_point_count < ROUTE_MAX_GEOM_POINTS) {
            GeomPoint pt_start;
            pt_start.lat_scaled = seg->start_lat_e7;
            pt_start.lon_scaled = seg->start_lon_e7;
            out_route->geom_points[out_route->geom_point_count++] = pt_start;
        }

        /* Read intermediate curves from *_geom.bin */
        if (seg->geom_count > 0 && seg->geom_path && seg->geom_path[0] != '\0') {
            if (!current_open_path || strcmp(current_open_path, seg->geom_path) != 0) {
                if (f_geom) fclose(f_geom);
                f_geom = fopen(seg->geom_path, "rb");
                current_open_path = seg->geom_path;
            }

            if (f_geom) {
                fseek(f_geom, (long)seg->geom_offset, SEEK_SET);
                int32_t target_idx = out_route->geom_point_count;
                int32_t to_read = seg->geom_count;
                if (target_idx + to_read > ROUTE_MAX_GEOM_POINTS - 1) {
                    to_read = (ROUTE_MAX_GEOM_POINTS - 1) - target_idx;
                }
                if (to_read > 0) {
                    size_t read_cnt = fread(&out_route->geom_points[target_idx], sizeof(GeomPoint), (size_t)to_read, f_geom);
                    out_route->geom_point_count += (int32_t)read_cnt;
                }
            }
        }

        /* Append end point */
        if (out_route->geom_point_count < ROUTE_MAX_GEOM_POINTS) {
            GeomPoint pt_end;
            pt_end.lat_scaled = seg->end_lat_e7;
            pt_end.lon_scaled = seg->end_lon_e7;
            out_route->geom_points[out_route->geom_point_count++] = pt_end;
        }
    }

    if (f_geom) {
        fclose(f_geom);
    }

    return (out_route->geom_point_count > 0);
}
