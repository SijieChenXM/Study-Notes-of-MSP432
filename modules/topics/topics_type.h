/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file topics_type.h
 */

#include <stdint.h>

// navigation state
enum nav_state_e {

    NAV_STATE_MANUAL = 0,
    NAV_STATE_STAB,
    NAV_STATE_ALTCTL,
    NAV_STATE_POSCTL,

    NAV_STATE_MISSION,
    NAV_STATE_LOITER,
    NAV_STATE_RTL,
    NAV_STATE_TAKEOFF,
    NAV_STATE_LAND,
    NAV_STATE_FOLLOW,
    NAV_STATE_LANDGPSFAIL,

    NAV_STATE_OFFBOARD,
};

typedef uint64_t time_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    uint8_t armed;                  // current arming state
    uint8_t landed;                 // landed state
    uint8_t ground_contact;         //
    uint8_t falling;                // falling
    enum nav_state_e nav_state;     // navigation state
    float voltage;
} status_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float x;                        // pitch control    -1..1
    float y;                        // roll control     -1..1
    float z;                        // throttle control  0..1
    float r;                        // yaw control      -1..1
    uint16_t aux[5];                // auxiliary channel    1000..2000
    uint8_t rssi;                   // signal strength      0..100
} manual_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame acceleration in m/s^s
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of acceleration
} accel_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame angular rate in rad/s
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of angular rate
} gyro_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float x, y, z;                  // body frame magetic field in Guass
    int16_t x_raw, y_raw, z_raw;    // raw sensor value of magnetic field
} mag_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float temperature;              // temperature in degrees celsius
    float pressure;                 // barometric pressure, already temp. comp.
    float altitude;                 // altitude, already temp. comp.
} baro_t;

typedef struct {
	uint64_t timestamp;             // update time in us
	float min_distance;             // valid minimal distance
    float max_distance;             // valid maximum distance
    float current_distance;         // current distance
    uint8_t orientation;
} sonar_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    uint64_t timestamp_time_relative;
    uint64_t time_utc_usec;         // timestamp (ms, UTC), this is the timestamp which comes from th gps module, It might be unvailable right after cold start, indicated by a value of 0
    int32_t lat;                    // latitude in 1e-7 degrees
    int32_t lon;                    // longitude in 1e-7 degrees
    int32_t alt;                    // altitude in 1e-3 meters above MSL (millimeters)
    int32_t alt_ellipsoid;          // altitude in 1e-3 meters blove Ellipsoid (millimeters)
    float s_variance_m_s;           // gps speed accuracy estimate (m/s)
    float c_variance_rad;           // gps course accuracy estimate (radians)
    uint8_t fix_type;               // 0-1: no fix   2: 2D fix   3: 3D fix   4: RTCM code diffenrential   5: RTK float   6: RTK fixed
    float eph;                      // gps horizontal position accuracy (meters)
    float epv;                      // gps vertical position accuracy (meters)
    float hdop;                     // horizontal dilution of precision
    float vdop;                     // vertical dilution of precision
    int32_t noise_per_ms;           // gps noise per millisecond
    int32_t jamming_indicator;      // indicates jamming is occurring
    float vel_m_s;                  // gps ground speed (m/s)
    float vel_n_m_s;                // gps north velocity (m/s)
    float vel_e_m_s;                // gps east velocity (m/s)
    float vel_d_m_s;                // gps down velocity (m/s)
    float cog_rad;                  // course over ground (not heading, but direction of movement), -PI..PI (radians)
    uint8_t vel_ned_valid;             // true if NED velocity is valid
    uint8_t satellites_used;        // number of satellites used
} gps_t;

typedef struct {
    uint64_t timestamp;             // update time (us)
    uint32_t integration_timespan;  // accumulation timespan (us)
    float x_rad;                    // accumulated optical flow around x axis (rad)
    float y_rad;                    // accumulated optical flow around y axis (rad)
    float x_rad_int;
    float y_rad_int;
    float x_rate_int;               // accumulated gyro value around x axis (rad)
    float y_rate_int;               // accumulated gyro value around y axis (rad)
    float z_rate_int;               // accumulated gyro value around z axis (rad)
    uint8_t quality;                // average of quality of accumulated frames, 0: bad quality, 255: maxium quality
} optical_flow_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float euler[3];                 // angle in rad
    float rate[3];                  // angular rate in rad/s
    float rate_bias[3];             // offsets of the body angular rates from zero
    float R[9];                     // rotation matrix, body to world
    float q[4];                     // quaternion (NED)
} attitude_t;

typedef struct {
    uint64_t timestamp;             // update time in us
    float roll_body;                // body frame angle setpoint in rad
    float pitch_body;               // body frame angle setpoint in rad
    float yaw_body;                 // body frame angle setpoint in rad
    float yaw_sp_move_rate;         // yaw rate setpoint in rad/s (commanded by user)

    float R_body[9];                // rotation matrix describing the setpoint as rotation from the current body frame

    float q_d[4];
    float thrust;                   // thrust in Newton the power system should generate

    uint8_t roll_reset_integral;    // reset roll integral part (navigation logic change)
    uint8_t pitch_reset_integral;   // reset pitch integral part (navigation logic change)
    uint8_t yaw_reset_integral;     // reset yaw integral part (navigation logic change)

    uint8_t apply_flaps;            // control heading with rudder (used for auto takeoff on runway)
} attitude_setpoint_t;

typedef struct {
    uint64_t timestamp;             // update time (us)
    uint64_t ref_timestamp;         // ref time
    double ref_lat, ref_lon;        // reference point in latitude & longitute
    float ref_alt;                  // reference altitude AMSL, must be set to current (not at reference point) ground level (m)
    float baro_offset;              // baro altitude offset
    float x, y, z;                  // position in NED earth-fixed frame (m)
    float vx, vy, vz;               // velocity in NED earth-fixed frame (m/s)
    float ax, ay, az;               // acceleration in NED earth-fixed frame (m/s^2)
    float dist_bottom;
    float dist_bottom_rate;
    float yaw;                      // euler yaw angle
    float eph;
    float epv;
    uint8_t xy_valid, z_valid;      // set to true if xy(z) position valid
    uint8_t v_xy_valid, v_z_valid;  // set to true if xy(z) velocity valid
    uint8_t dist_bottom_valid;
} local_position_t;

typedef struct {
    uint64_t timestamp;             // update time (us)
    float x, y, z;                  // position setpoint in NED earth-fixed frame (m)
    float yaw;                      // yaw setpoint in NED (rad) -PI..+PI
    float vx, vy, vz;               // velocity setpoint in NED earth-fixed frame (m/s)
    float acc_x, acc_y, acc_z;      // acceleration setpoint  in NED earth-fixed frame (m/s^2)
} local_position_setpoint_t;

typedef struct {
    uint64_t timestamp;
    uint8_t manual_enabled;
    uint8_t auto_enabled;
    uint8_t offboard_enabled;
    uint8_t rates_enabled;
    uint8_t attitude_enabled;
    uint8_t rattitude_enabled;
    uint8_t force_enabled;
    uint8_t acceleration_enabled;
    uint8_t velocity_enabled;
    uint8_t position_enabled;
    uint8_t altitude_enabled;
    uint8_t climb_rate_enabled;
    uint8_t termination_enabled;
    uint8_t fixed_hdg_enabled;
} control_mode_t;

typedef union {
    struct {
        uint16_t valid      : 1;    // 0 - true when the saturation status is used
        uint16_t motor_pos  : 1;    // 1 - true when any motor has saturated in the positive direction
        uint16_t motor_neg  : 1;    // 2 - true when any motor has saturated in the negative direction
        uint16_t roll_pos   : 1;    // 3 - true when a positive roll demand change will increase saturation
        uint16_t roll_neg   : 1;    // 4 - true when a negative roll demand change will increase saturation
        uint16_t pitch_pos  : 1;    // 5 - true when a positive pitch demand change will increase saturation
        uint16_t pitch_neg  : 1;    // 6 - true when a negative pitch demand change will increase saturation
        uint16_t yaw_pos    : 1;    // 7 - true when a positive yaw demand change will increase saturation
        uint16_t yaw_neg    : 1;    // 8 - true when a negative yaw demand change will increase saturation
        uint16_t thrust_pos : 1;    // 9 - true when a positive thrust demand change will increase saturation
        uint16_t thrust_neg : 1;    //10 - true when a negative thrust demand change will increase saturation
    } flags;

    uint16_t value;

} saturation_status_t;

typedef struct {
    uint64_t timestamp;         // update time (us)
    float control[4];           // actuator output [0]-[1]-[2]:roll-pitch-yaw -1..1    [3]:throttle 0..1
} actuator_controls_t;

typedef struct {
    uint64_t timestamp;         // update time (us)
    uint8_t rotor_count;        // effective rotor numbers
    float control[16];          // output data, in natual output uints
} actuator_output_t;

typedef struct {
	uint64_t timestamp;         // required for logger
	uint8_t text[50];
	uint8_t severity;
} mavlink_log_t;

typedef struct {
    uint64_t timestamp;
    uint32_t timespan;
    float rc_x;
    float rc_y;
    float rc_z;
    float rc_r;
    uint8_t rc_cmd;
    uint8_t valid;
    float x;
    float y;
} vision_t;
