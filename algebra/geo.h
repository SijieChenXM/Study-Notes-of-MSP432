/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file geo.c
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CONSTANTS_ONE_G                         9.80665f    // m/s^2
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C     1.225f      // kg/m^3
#define CONSTANTS_AIR_GAS_CONST                 287.1f      // J/(kg * K)
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS         -273.15f    // °C
#define CONSTANTS_RADIUS_OF_EARTH               6371000     // meters (m)

#define ERROR   -1
#define OK       0

#ifdef _cplusplus
extern "C" {
#endif

// XXX remove
struct crosstrack_error_s {
	bool past_end;		// Flag indicating we are past the end of the line/arc segment
	float distance;		// Distance in meters to closest point on line/arc
	float bearing;		// Bearing in radians to closest point on line/arc
} ;

/* lat/lon are in radians */
struct map_projection_reference_s {
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	bool init_done;
	uint64_t timestamp;
};

struct globallocal_converter_reference_s {
	float alt;
	bool init_done;
};

/**
 * Checks if global projection was initialized
 * @return true if map was initialized before, false else
 */
extern bool map_projection_global_initialized(void);

/**
 * Checks if projection given as argument was initialized
 * @return true if map was initialized before, false else
 */
extern bool map_projection_initialized(const struct map_projection_reference_s *ref);

/**
 * Get the timestamp of the global map projection
 * @return the timestamp of the map_projection
 */
extern uint64_t map_projection_global_timestamp(void);

/**
 * Get the timestamp of the map projection given by the argument
 * @return the timestamp of the map_projection
 */
extern uint64_t map_projection_timestamp(const struct map_projection_reference_s *ref);

/**
 * Writes the reference values of the global projection to ref_lat and ref_lon
 * @return 0 if map_projection_init was called before, -1 else
 */
extern int map_projection_global_reference(double *ref_lat_rad, double *ref_lon_rad);

/**
 * Writes the reference values of the projection given by the argument to ref_lat and ref_lon
 * @return 0 if map_projection_init was called before, -1 else
 */
extern int map_projection_reference(const struct map_projection_reference_s *ref, double *ref_lat_rad,
				      double *ref_lon_rad);

/**
 * Initializes the global map transformation.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
extern int map_projection_global_init(double lat_0, double lon_0, uint64_t timestamp);

/**
 * Initializes the map transformation given by the argument.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
extern int map_projection_init_timestamped(struct map_projection_reference_s *ref,
		double lat_0, double lon_0, uint64_t timestamp);

/**
 * Initializes the map transformation given by the argument and sets the timestamp to now.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
extern int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0, uint64_t time);

/**
 * Transforms a point in the geographic coordinate system to the local
 * azimuthal equidistant plane using the global projection
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
extern int map_projection_global_project(double lat, double lon, float *x, float *y);


/* Transforms a point in the geographic coordinate system to the local
 * azimuthal equidistant plane using the projection given by the argument
* @param x north
* @param y east
* @param lat in degrees (47.1234567°, not 471234567°)
* @param lon in degrees (8.1234567°, not 81234567°)
* @return 0 if map_projection_init was called before, -1 else
*/
extern int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x,
				    float *y);

/**
 * Transforms a point in the local azimuthal equidistant plane to the
 * geographic coordinate system using the global projection
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
extern int map_projection_global_reproject(float x, float y, double *lat, double *lon);

/**
 * Transforms a point in the local azimuthal equidistant plane to the
 * geographic coordinate system using the projection given by the argument
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
extern int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat,
				      double *lon);

/**
 * Get reference position of the global map projection
 */
extern int map_projection_global_getref(double *lat_0, double *lon_0);

/**
 * Initialize the global mapping between global position (spherical) and local position (NED).
 */
extern int globallocalconverter_init(double lat_0, double lon_0, float alt_0, uint64_t timestamp);

/**
 * Checks if globallocalconverter was initialized
 * @return true if map was initialized before, false else
 */
extern bool globallocalconverter_initialized(void);

/**
 * Convert from global position coordinates to local position coordinates using the global reference
 */
extern int globallocalconverter_tolocal(double lat, double lon, float alt, float *x, float *y, float *z);

/**
 * Convert from local position coordinates to global position coordinates using the global reference
 */
extern int globallocalconverter_toglobal(float x, float y, float z,  double *lat, double *lon, float *alt);

/**
 * Get reference position of the global to local converter
 */
extern int globallocalconverter_getref(double *lat_0, double *lon_0, float *alt_0);

/**
 * Returns the distance to the next waypoint in meters.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
extern float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);


/**
 * Creates a new waypoint C on the line of two given waypoints (A, B) at certain distance
 * from waypoint A
 *
 * @param lat_A waypoint A latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_A waypoint A longitude in degrees (8.1234567°, not 81234567°)
 * @param lat_B waypoint B latitude in degrees (47.1234567°, not 471234567°)
 * @param lon_B waypoint B longitude in degrees (8.1234567°, not 81234567°)
 * @param dist distance of target waypoint from waypoint A in meters (can be negative)
 * @param lat_target latitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint C in degrees (47.1234567°, not 471234567°)
 */
extern void create_waypoint_from_line_and_dist(double lat_A, double lon_A, double lat_B, double lon_B, float dist,
		double *lat_target, double *lon_target);

/**
 * Creates a waypoint from given waypoint, distance and bearing
 * see http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @param lat_start latitude of starting waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_start longitude of starting waypoint in degrees (8.1234567°, not 81234567°)
 * @param bearing in rad
 * @param distance in meters
 * @param lat_target latitude of target waypoint in degrees (47.1234567°, not 471234567°)
 * @param lon_target longitude of target waypoint in degrees (47.1234567°, not 471234567°)
 */
extern void waypoint_from_heading_and_distance(double lat_start, double lon_start, float bearing, float dist,
		double *lat_target, double *lon_target);

/**
 * Returns the bearing to the next waypoint in radians.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
extern float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

extern void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n,
		float *v_e);

extern void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next,
		float *v_n, float *v_e);

extern void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res,
		double *lon_res);

extern int get_distance_to_line(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
				  double lat_start, double lon_start, double lat_end, double lon_end);

extern int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now,
				 double lat_center, double lon_center,
				 float radius, float arc_start_bearing, float arc_sweep);

/*
 * Calculate distance in global frame
 */
extern float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z);

/*
 * Calculate distance in local frame (NED)
 */
extern float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
		float x_next, float y_next, float z_next,
		float *dist_xy, float *dist_z);

extern float _wrap_180(float bearing);
extern float _wrap_360(float bearing);
extern float _wrap_pi(float bearing);
extern float _wrap_2pi(float bearing);

#ifdef _cplusplus
}
#endif
