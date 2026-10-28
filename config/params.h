/**
 * @file params.h
 * @brief Every tunable constant of the simulation in one place.
 *
 * Units are SI and are encoded in the name suffix:
 *   _S (seconds), _M (meters), _MPS (m/s), _MPS2 (m/s^2), _RAD (radians),
 *   _PX (pixels), _1PM (1/m, curvature).
 *
 * Coordinate frames follow ISO 8855 (x forward, y left, z up, yaw CCW+).
 */
#ifndef ADAS_PARAMS_H
#define ADAS_PARAMS_H

/* ======================================================================== */
/* Simulation timing                                                        */
/* ======================================================================== */
#define SIM_DT_S 0.01f        /* plant + low-level control: 100 Hz          */
#define PIPELINE_DECIMATION 5 /* sensors/perception/planning every N steps  */
#define PIPELINE_DT_S (SIM_DT_S * PIPELINE_DECIMATION) /* -> 20 Hz          */
#define SIM_DT_MS (SIM_DT_S * 1000.0f) /* base step in milliseconds         */
#define RNG_SEED 42u
#define CONSOLE_PRINT_PERIOD_S 2.0f

/* Scenario to run: one of the ScenarioId values in include/sim/scenario.h
 * (SCENARIO_HIGHWAY_STOP_AND_GO, SCENARIO_FREE_ROAD_CURVE, SCENARIO_CUT_IN,
 * SCENARIO_SLOW_LEAD_APPROACH, SCENARIO_LEAD_HARD_BRAKE,
 * SCENARIO_STOP_AND_GO_TRAFFIC). */
#define SCENARIO_SELECTED SCENARIO_HIGHWAY_STOP_AND_GO

/* ======================================================================== */
/* Output                                                                   */
/* ======================================================================== */
#define LOG_DIR "out"
/* CSV log per run: printf format, filled with the scenario name and
 * RNG_SEED, e.g. out/highway_follow_stop_and_go_seed42.csv.               */
#define LOG_PATH_FMT LOG_DIR "/%s_seed%u.csv"
#define LOG_PATH_MAX 256 /* buffer size for the formatted path              */

/* ======================================================================== */
/* Latency injection                                                        */
/* ======================================================================== */
/* A plan reaches the controllers SENSOR_LATENCY_MS + the pipeline's
 * critical path (sum of node latency_ms along it, adas_pipeline.c) after
 * its sensor capture, rounded up to whole SIM_DT_S steps.
 * 0 = the plan is applied in the step its sensors were captured.           */
#define LATENCY_INJECTION 1
/* Capture -> pipeline input: exposure, readout, ISP and transport.         */
#define SENSOR_LATENCY_MS 20.0f

/* ======================================================================== */
/* Front camera: pinhole, zero pitch/roll, mounted at the vehicle reference */
/* ======================================================================== */
#define CAM_W 160             /* image width  [px]                          */
#define CAM_H 120             /* image height [px]                          */
#define CAM_FX 139.0f         /* focal length [px] (~60 deg HFOV)           */
#define CAM_CU (CAM_W * 0.5f) /* principal point, column [px]               */
#define CAM_CV (CAM_H * 0.5f) /* principal point, row = horizon [px]        */
#define CAM_HEIGHT_M 1.3f     /* mounting height above ground               */
#define CAM_NOISE_STD 0.03f   /* additive Gaussian pixel noise              */
/* Synthetic scene intensities in [0,1]. */
#define CAM_I_SKY 0.5f /* overcast: brighter than road, darker than paint   */
#define CAM_I_ROAD 0.4f
#define CAM_I_MARKING 1.0f
#define CAM_I_VEHICLE 0.1f

/* ======================================================================== */
/* LiDAR: fixed scan pattern, mounted at the vehicle reference              */
/* ======================================================================== */
#define LIDAR_MAX_POINTS 512
#define LIDAR_MAX_RANGE_M 80.0f
#define LIDAR_AZ_RES_RAD 0.0035f /* horizontal angular resolution (0.2 deg) */
#define LIDAR_EL_RES_RAD 0.0175f /* vertical angular resolution (1 deg)     */
#define LIDAR_RANGE_NOISE_STD_M 0.02f
#define LIDAR_GROUND_NOISE_STD_M 0.03f
/* Ground returns are sampled on a polar grid (rings x azimuths). */
#define LIDAR_GROUND_R_MIN_M 4.0f
#define LIDAR_GROUND_R_STEP_M 3.0f
#define LIDAR_GROUND_N_RINGS 19
#define LIDAR_GROUND_AZ_MIN_RAD (-0.70f)
#define LIDAR_GROUND_AZ_STEP_RAD 0.0875f
#define LIDAR_GROUND_N_AZ 17
#define LIDAR_N_CLUTTER 20 /* random spurious returns per scan              */
#define LIDAR_MAX_COLS_PER_OBJ 20
#define LIDAR_MAX_ROWS_PER_OBJ 8

/* ======================================================================== */
/* Object / road priors (shared by the simulator and perception)            */
/* ======================================================================== */
#define VEHICLE_WIDTH_M 1.8f
#define VEHICLE_HEIGHT_M 1.5f
#define VEHICLE_BODY_Z_MIN_M 0.3f /* lowest body point hit by the LiDAR     */
#define LANE_WIDTH_NOMINAL_M 3.5f

/* ======================================================================== */
/* Perception DNNs                                                          */
/* ======================================================================== */
/* LaneSegNet: logit = W_BRIGHT*ReLU(I - LEVEL) + W_RIDGE*ReLU(LoG(I)) + BIAS */
#define LANESEG_BRIGHT_LEVEL 0.55f
#define LANESEG_W_BRIGHT 20.0f
#define LANESEG_W_RIDGE 10.0f
#define LANESEG_BIAS (-2.0f)

/* VehicleDetector2D: "dark blob" heat-map detector (CenterNet-style decode). */
#define DET2D_MAX_BOXES 4
/* Intensity below which a pixel is "vehicle-like". */
#define DET2D_DARK_LEVEL 0.25f
#define DET2D_DARK_GAIN 4.0f
#define DET2D_FILL_THRESH 0.3f /* min mean darkness in a 3x3 patch          */
#define DET2D_GAIN 20.0f
#define DET2D_SCORE_THRESH 0.5f
#define DET2D_MIN_SIZE_PX 2

/* PointFilterNet: per-point MLP that removes ground / out-of-band points. */
#define PFILTER_Z_MIN_M 0.3f
#define PFILTER_Z_MAX_M 2.5f
#define PFILTER_KEEP_THRESH 0.5f

/* VehicleDetector3D: bird's-eye-view (BEV) occupancy grid + conv. */
#define BEV_X_MIN_M 0.0f
#define BEV_Y_MIN_M (-10.0f)
#define BEV_RES_M 1.0f
#define BEV_NX 80 /* cells along x (0..80 m)                                */
#define BEV_NY 20 /* cells along y (-10..10 m)                              */
#define DET3D_MAX_BOXES 4
#define DET3D_MIN_POINTS 5

/* ======================================================================== */
/* State estimation                                                         */
/* ======================================================================== */
/* LaneGeometryNet: row anchors (image rows) used to sample the lane mask. */
#define LANE_N_ANCHORS 6
#define LANE_ANCHOR_ROWS {70, 72, 75, 79, 85, 95} /* ~17 m ... ~5 m ahead */
#define LANE_MIN_ROW_MASS 0.2f /* min mask "mass" to accept a marking       */
/* Rows that may be filled from the last estimate. */
#define LANE_MAX_IMPUTED_ROWS 3

/* LeadFusion: constant-velocity Kalman filter on [gap, relative speed]. */
/* Process noise (lead acceleration) [m/s^2]. */
#define FUSION_ACCEL_NOISE_STD 4.0f
#define FUSION_LIDAR_STD_M 0.15f
/* Camera range std = base + k * range. */
#define FUSION_CAM_STD_BASE_M 0.5f
#define FUSION_CAM_STD_PER_M 0.05f
#define FUSION_INIT_VREL_STD 5.0f
#define FUSION_GATE_CHI2 9.0f /* 3-sigma innovation gate                    */
#define FUSION_CONFIRM_HITS 3
#define FUSION_MAX_MISSES 10
/* Consecutive gated-out LiDAR updates before the state is re-seeded
 * (divergence guard). */
#define FUSION_LIDAR_REJECTS_REINIT 3

/* ======================================================================== */
/* Planning                                                                 */
/* ======================================================================== */
/* ACC: constant time-gap (CTG) spacing policy, d_des = D0 + T_gap * v. */
#define ACC_TIME_GAP_S 1.8f
#define ACC_D0_M 8.0f    /* standstill distance (ref point to lead rear)    */
#define ACC_K_SPEED 0.4f /* cruise: a = k_v * (v_set - v)                   */
#define ACC_K_GAP 0.25f  /* follow: a = k_d * (d - d_des) + k_vr * v_rel    */
#define ACC_K_VREL 0.8f
#define ACC_A_MIN_MPS2 (-3.5f) /* comfort limits                            */
#define ACC_A_MAX_MPS2 2.0f
/* AEB-style emergency braking inside the behavior planner. */
#define AEB_TTC_ENTER_S 1.8f
#define AEB_TTC_EXIT_S 3.0f
#define AEB_DECEL_MPS2 (-6.0f)
/* LKA */
#define LKA_MIN_SPEED_MPS 2.0f
#define LKA_KAPPA_LPF_ALPHA 0.2f /* first-order low-pass on curvature       */

/* ======================================================================== */
/* Low-level control                                                        */
/* ======================================================================== */
#define LON_KP 0.5f /* PID on acceleration error                            */
#define LON_KI 0.8f
#define LON_KD 0.0f
#define LON_I_LIMIT 2.0f
#define STANLEY_K 1.0f          /* cross-track gain                         */
#define STANLEY_K_SOFT_MPS 1.0f /* softening speed (low-speed stability)    */

/* ======================================================================== */
/* Vehicle plant: kinematic bicycle + first-order actuators                 */
/* ======================================================================== */
#define VEH_WHEELBASE_M 2.8f
/* Reference point (rear axle) -> front bumper. */
#define VEH_FRONT_OVERHANG_M 3.8f
#define VEH_TAU_ACCEL_S 0.3f /* powertrain/brake lag                   */
#define VEH_TAU_STEER_S 0.1f /* steering actuator lag                  */
#define VEH_STEER_MAX_RAD 0.5f
#define VEH_A_DRIVE_MAX_MPS2 3.0f /* throttle = 1                           */
#define VEH_A_BRAKE_MAX_MPS2 8.0f /* brake = 1                              */
#define VEH_C_DRAG_1PM 0.0004f    /* aero drag: a = c * v^2                 */
#define VEH_C_ROLL_MPS2 0.1f      /* rolling resistance                     */
#define EGO_V_NOISE_STD_MPS 0.05f /* wheel-speed sensor                     */
#define EGO_A_NOISE_STD_MPS2 0.1f /* IMU longitudinal accel                 */

#endif /* ADAS_PARAMS_H */
