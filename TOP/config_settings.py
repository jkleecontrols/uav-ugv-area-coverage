# 여기서 uav 이륙속도, 비행 속도, 착륙속도 그리고 배터리 소모양 등등을 설정할거야. p그리고 관련 데이터 처리는 models.vehicles

CONFIG = {
    "csv_path": "/Users/jackielee/PersistentServailence/environments/corridor_scene/corridor_scene.csv" ,
    "num_uav" : 3,
    "num_ugv" : 1,
    "num_charging_pads" : 2,
    "uav_takeoff_time": 30,             # sec
    "uav_flight_speed": 15,             # m/s
    "uav_flight_time": 1200,            # sec 20*60
    "uav_charge_time": 840,             # sec 14*60
    "uav_landing_time": 30,             # sec
    "uav_battery_discharge_rate": 4.5,  # % per sec
    "uav_battery_charge_rate": 6.5,     # % per sec
    "ugv_drive_speed": 5,               # m/s
    "ugv_takeoffland_speed": 1,
    "ugv_drive_time": 7200,             # sec
    "ugv_battery_swap_time": 300,       # sec
    "uav_flight_charge_gcd": 120,       # sec
    "major_axis": (1200 - 30 - 30) * 15,  # m (uav_flight_time - takeoff_time - landing_time) * flight_speed
    # StepTwo improvement phase parameters
    "step_two_max_outer_loop": 10,      # K: 최대 외부 루프 횟수
    "step_two_max_inner_loop": 5,       # I: 최대 내부 루프 횟수
    "step_two_no_improvement_limit": 5,  # 개선이 없을 때 루프 종료 기준
    # TabuSearch parameters
    "tabu_list_size": 10,               # Tabu 목록의 최대 크기
    "tabu_max_iterations": 100,         # Tabu Search 최대 반복 횟수
    "tabu_no_improvement_limit": 20,    # 개선이 없을 때 Tabu Search 종료 기준
    # StepReinit parameters
    "reinit_deviation_ratio": 0.025,    # Reinitialization deviation ratio (record의 2.5%)
    "reinit_max_cycles": 3,             # Step 2-3 순환 최대 횟수
    # StepReattachment parameters
    "max_detour_ratio": 0.2,            # 최대 우회 비율 (20%)
    "reattachment_distance_threshold": 50,  # 노드 간 최대 거리 (m)
    "reattachment_min_nodes": 3         # 최소 연결 노드 수
}