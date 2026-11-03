/**
 * @file test_geocoder.c
 * @brief Unit tests for Forward & Reverse Geocoding (street <-> coordinate).
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "geocoder.h"
#include "pathfinder.h"
#include "navigation.h"

static RouteResult s_test_route;

int main(void) {
    printf("==================================================\n");
    printf("  GEOCODER MODULE VERIFICATION SUITE\n");
    printf("==================================================\n");

    const char *bin_dir = "../binary_map_data";
    FILE *probe = fopen("../binary_map_data/names.idx", "rb");
    if (!probe) {
        bin_dir = "binary_map_data";
        probe = fopen("binary_map_data/names.idx", "rb");
    }
    if (probe) {
        fclose(probe);
    } else {
        printf("[!] binary_map_data not found. Skipping live binary lookup tests.\n");
        return 0;
    }

    /* 1. Test UTF-8 Normalization */
    char norm[128];
    geocoder_normalize_name("  Rue des Guillemins, Liège! ", norm, sizeof(norm));
    printf("[1] Normalize test: '%s'\n", norm);
    assert(strcmp(norm, "rue des guillemins liege") == 0);

    /* 2. Initialize Cache & Geocoder */
    TileGraphCache cache;
    bool cache_ok = tile_graph_cache_init(&cache, bin_dir, 4);
    assert(cache_ok);

    Geocoder geo;
    bool geo_ok = geocoder_init(&geo, &cache, bin_dir);
    assert(geo_ok);
    printf("[2] Loaded %u streets and %u towns from '%s'\n",
           geo.street_count, geo.town_count, bin_dir);

    /* Reference position: Liège-Guillemins (50.6242° N, 5.5666° E) */
    GeoCoord liege_ref = geo_coord_make(506242000, 55666000);

    /* 3. Reverse Geocoding: Coordinate -> Street Name */
    GeocoderResult rev_res;
    bool rev_ok = geocoder_coord_to_street(&geo, liege_ref, &rev_res);
    assert(rev_ok && rev_res.found);
    printf("[3] Reverse Geocode (50.6242, 5.5666):\n");
    printf("    -> Street : '%s' (ID %u)\n", rev_res.street_name, rev_res.street_id);
    printf("    -> Town   : '%s'\n", rev_res.town_name);
    printf("    -> Snapped: (%d, %d), dist = %u m\n",
           rev_res.coord.lat_e7, rev_res.coord.lon_e7, rev_res.distance_m);

    /* 4. Forward Geocoding WITH Town: "[street name], [town]" */
    GeocoderResult fwd_with_town;
    bool fwd1_ok = geocoder_street_to_coord(&geo, "Rue des Guillemins, Liège", liege_ref, &fwd_with_town);
    assert(fwd1_ok && fwd_with_town.found);
    printf("[4] Forward Geocode ('Rue des Guillemins, Liège'):\n");
    printf("    -> Street : '%s' (ID %u)\n", fwd_with_town.street_name, fwd_with_town.street_id);
    printf("    -> Town   : '%s'\n", fwd_with_town.town_name);
    printf("    -> Coord  : (%.5f, %.5f), dist from ref = %u m\n",
           fwd_with_town.coord.lat_e7 / 1e7, fwd_with_town.coord.lon_e7 / 1e7,
           fwd_with_town.distance_m);

    /* 5. Forward Geocoding WITHOUT Town: "[street name]" (closest to ref_coord) */
    GeocoderResult fwd_no_town;
    bool fwd2_ok = geocoder_street_to_coord(&geo, "Rue des Guillemins", liege_ref, &fwd_no_town);
    assert(fwd2_ok && fwd_no_town.found);
    printf("[5] Forward Geocode without town ('Rue des Guillemins' closest to ref):\n");
    printf("    -> Street : '%s' (ID %u)\n", fwd_no_town.street_name, fwd_no_town.street_id);
    printf("    -> Town   : '%s'\n", fwd_no_town.town_name);
    printf("    -> Coord  : (%.5f, %.5f), dist from ref = %u m\n",
           fwd_no_town.coord.lat_e7 / 1e7, fwd_no_town.coord.lon_e7 / 1e7,
           fwd_no_town.distance_m);

    /* 6. Round-trip verification: Reverse geocode the forward-geocoded coordinate */
    GeocoderResult roundtrip;
    bool rt_ok = geocoder_coord_to_street(&geo, fwd_with_town.coord, &roundtrip);
    assert(rt_ok && roundtrip.found);
    printf("[6] Round-trip Reverse Geocode:\n");
    printf("    -> Street : '%s', dist = %u m\n", roundtrip.street_name, roundtrip.distance_m);
    assert(roundtrip.distance_m <= 5);

    /* 7. Test On-the-Fly Navigation ("Lego" instruction builder) */
    Pathfinder pf;
    pathfinder_init(&pf, &cache);
    GeoCoord carre_goal = geo_coord_make(506428000, 55702000);
    bool route_ok = pathfinder_find_route(&pf, liege_ref, carre_goal, &s_test_route);
    assert(route_ok && s_test_route.found);
    pathfinder_load_geometry(&pf, &s_test_route);

    printf("[7] On-the-Fly Navigation Instructions (Liège-Guillemins -> Liège-Carré, %d segs):\n",
           s_test_route.segment_count);

    NavState nav;
    nav_init(&nav);

    int32_t last_maneuver_seg = -1;
    for (int32_t i = 0; i < s_test_route.segment_count; ++i) {
        GeoCoord pos = geo_coord_make(s_test_route.segments[i].start_lat_e7,
                                      s_test_route.segments[i].start_lon_e7);
        bool nav_ok = nav_update(&nav, &s_test_route, &geo, pos);
        assert(nav_ok);

        if (nav.next_step.maneuver_seg_idx != last_maneuver_seg || i == s_test_route.segment_count - 1) {
            char instr[192];
            nav_format_instruction(&nav.next_step, nav.cached_street_name, instr, sizeof(instr));
            printf("    [Seg %2d | angle %+4d deg] %s\n",
                   i, (int)nav.next_step.angle_deg, instr);
            last_maneuver_seg = nav.next_step.maneuver_seg_idx;
        }
    }

    tile_graph_cache_destroy(&cache);
    printf("\n[PASS] All Geocoder & Navigation tests passed!\n");
    return 0;
}
