CONFIG = {
    # Basic configuration
    "csv_path": "map/tamu_map.csv",
    "num_uav": 2,
    "num_ugv": 1,
    "num_charging_pads": 2,

    # UAV parameters
    "uav_height": 50,                   # m
    "uav_fov_angle": 94,                # degrees (diagonal FOV angle)
    "uav_aspect_ratio": 16/9,           # camera aspect ratio
    "uav_flight_speed": 10,             # m/s
    "uav_flight_time": 1200,            # sec (20 min)
    "uav_charge_time": 840,             # sec (14 min)
    "uav_takeoff_time": 30,             # sec
    "uav_landing_time": 30,             # sec
    "uav_min_overlap": 0.2,             # 20% minimum overlap between consecutive images
    "uav_speed": 10.0,  # UAV 이동 속도 (m/s)
    
    # UAV battery parameters
    "uav_battery_discharge_rate": 100/1200,  # %/sec (100% in 20 min)
    "uav_battery_charge_rate": 100/840,      # %/sec (100% in 14 min)
    
    # UGV parameters
    "ugv_drive_speed": 5,               # m/s
    "ugv_battery_swap_time": 300,       # sec
    "ugv_takeoffland_speed": 2,         # m/s (slower speed during UAV takeoff/landing)
    
    # Surveillance parameters
    "coverage_altitude": 50,            # m (optimal height for desired ground coverage)
    "ground_resolution": 0.05,          # m/pixel (desired ground sampling distance)
    "min_detection_pixel": 20,          # minimum pixels for target detection
    
    # Time management
    "uav_flight_charge_gcd": 120,       # sec (time slot duration)
    "simulation_time_step": 1,          # sec (simulation update interval)
    
    # Path planning parameters
    "major_axis": 20 * 60 * 10,  # 20분 * 60초 * 10m/s = 12000m (20분 동안 UAV가 이동 가능한 최대 거리)
    "step_two_max_outer_loop": 5,       # K: maximum outer loop iterations (reduced from 10)
    "step_two_max_inner_loop": 3,       # I: maximum inner loop iterations (reduced from 5)
    "step_two_no_improvement_limit": 3,  # iterations without improvement before termination (reduced from 5)
    
    # Optimization parameters
    "tabu_list_size": 5,                # maximum size of tabu list (reduced from 10)
    "tabu_max_iterations": 50,          # maximum tabu search iterations (reduced from 100)
    "tabu_no_improvement_limit": 10,    # iterations without improvement before termination (reduced from 20)
    "reinit_deviation_ratio": 0.05,     # reinitialization deviation ratio (increased from 0.025)
    "reinit_max_cycles": 2,             # maximum step 2-3 cycles (reduced from 3)
    
    # Coverage parameters
    "max_detour_ratio": 0.2,            # maximum path detour ratio (20%)
    "reattachment_distance_threshold": 50,  # m (maximum inter-node distance)
    "reattachment_min_nodes": 3,        # minimum nodes for path reattachment
    
    # Coverage quality parameters
    "min_coverage_ratio": 0.95,         # minimum area coverage ratio required
    "max_gap_size": 10,                 # m (maximum allowed gap in coverage)
    "redundancy_threshold": 0.3         # maximum allowed coverage overlap (30%)
}