/*
 * VehicleCore C API — the stable, language-neutral boundary of the simulator.
 *
 * Used by the SwiftUI app, and intended for Unreal Engine (link libvehiclecore + this header
 * from a ThirdParty module), Python (ctypes), or any other host. All structs are plain C,
 * all units SI, angles in radians, coordinates ISO 8855 (x forward, y left, z up).
 * A vd_sim is not thread-safe; use one instance per thread.
 */
#ifndef VD_CAPI_H
#define VD_CAPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VD_API_VERSION 1

typedef struct vd_sim vd_sim;

typedef struct vd_inputs {
  double steer;     /* steering-wheel angle [rad], positive = left */
  double throttle;  /* 0..1 */
  double brake;     /* 0..1 */
  double handbrake; /* 0..1 */
  int32_t gear;     /* 0 = automatic, >0 = force gear, -1 = neutral */
  int32_t reserved;
} vd_inputs;

typedef struct vd_wheel_telemetry {
  double fz, fx, fy, mz;        /* tire forces in the wheel frame [N, N m] */
  double slip_ratio, slip_angle;/* [-], [rad] */
  double omega, spin_angle;     /* [rad/s], [rad] */
  double steer, camber;         /* [rad] */
  double compression;           /* suspension compression from static [m] */
  double drive_torque, brake_torque;
} vd_wheel_telemetry;

typedef struct vd_telemetry {
  double time;
  double x, y, yaw;
  double heave, roll, pitch;
  double vx, vy, yaw_rate;
  double speed, ax, ay, sideslip;
  double steer, throttle, brake;
  double engine_rpm, engine_torque, clutch_torque;
  int32_t gear;
  int32_t reserved;
  double distance;
  vd_wheel_telemetry wheel[4]; /* FL, FR, RL, RR */
} vd_telemetry;

typedef struct vd_geometry {
  double wheelbase, a, b, track_front, track_rear, cg_height;
  double wheel_radius_front, wheel_radius_rear;
} vd_geometry;

typedef enum vd_param_kind { VD_PARAM_REAL = 0, VD_PARAM_INTEGER = 1, VD_PARAM_ENUM = 2 } vd_param_kind;

typedef struct vd_param_info {
  const char* key;     /* dotted key, e.g. "suspension.front.spring_rate" */
  const char* group;   /* UI group, e.g. "Suspension Front" */
  const char* label;
  const char* unit;
  const char* choices; /* '|' separated labels for VD_PARAM_ENUM, else "" */
  double min, max, step, default_value;
  int32_t kind;        /* vd_param_kind */
  int32_t reserved;
} vd_param_info;

int vd_api_version(void);

vd_sim* vd_create(void);
void vd_destroy(vd_sim* sim);
/* Last error message for this instance ("" if none). */
const char* vd_last_error(const vd_sim* sim);

/* Vehicle definition as JSON (see vehicles/<name>.json). Returns 0 on success. */
int vd_load_vehicle_json(vd_sim* sim, const char* json);
/* Returns a malloc'd JSON string of all parameters; release with vd_free_string. */
char* vd_export_vehicle_json(const vd_sim* sim);
void vd_free_string(char* s);

/* Parameter registry. Pointers in vd_param_info stay valid for the lifetime of `sim`. */
int vd_param_count(const vd_sim* sim);
int vd_param_info_at(const vd_sim* sim, int index, vd_param_info* out);
int vd_set_param(vd_sim* sim, const char* key, double value);
int vd_get_param(const vd_sim* sim, const char* key, double* value);
void vd_reset_params(vd_sim* sim); /* restore factory defaults */

/* Places the car at rest on its suspension at (x, y, yaw), rolling at `speed`. */
void vd_reset(vd_sim* sim, double x, double y, double yaw, double speed);
void vd_set_inputs(vd_sim* sim, const vd_inputs* inputs);

/* Scripted driver (maneuver JSON, see scenarios/<name>.json "maneuver"). Pass NULL to return to
 * manual inputs. Resets the vehicle to `initial_speed`. Returns 0 on success. */
int vd_set_maneuver(vd_sim* sim, const char* maneuver_json, double initial_speed);
int vd_has_maneuver(const vd_sim* sim);
/* Copies up to `max_points` reference-path points (x0,y0,x1,y1,...) and returns the count. */
int vd_maneuver_path(const vd_sim* sim, double* xy, int max_points);

/* Integrates `duration` seconds with the fixed internal step (remainders carry over). */
void vd_advance(vd_sim* sim, double duration);
void vd_get_telemetry(const vd_sim* sim, vd_telemetry* out);
void vd_get_geometry(const vd_sim* sim, vd_geometry* out);
double vd_time(const vd_sim* sim);

/* Evaluates the tire model of `axle` (0 front, 1 rear) for given load and slips. */
int vd_tire_eval(const vd_sim* sim, int axle, double fz, double slip_ratio, double slip_angle, double camber,
                 double* fx, double* fy, double* mz);

#ifdef __cplusplus
}
#endif

#endif /* VD_CAPI_H */
