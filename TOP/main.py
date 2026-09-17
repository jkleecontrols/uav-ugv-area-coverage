from pipeline_manager import PipelineManager
from models_vehicle import UAV
from data_loader import DataLoader
from config_settings import CONFIG  # 설정값 따로 관리하는 경우
from help import SimulationManager
from reward_manager import RewardManager
import json
import time
from datetime import datetime

if __name__ == "__main__":
    total_start_time = time.time()
    
    print(f"\n=== TOP Algorithm 실행 시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    
    # CSV 파일로부터 좌표 불러오기
    loader = DataLoader(CONFIG["csv_path"])
    max_simulation_time = 2040  # 24시간을 초 단위로 설정
    time_step = 1  # 1 second time step
    coordinates = loader.get_coordinates()  # 경로 계획에 쓸 대상
    interest_points = loader.get_named_points_with_names()  # depot 등 나중에 시각화, 기록용
    loader.set_vertices(coordinates)
    env = loader
    # DataLoader 초기화 후
    reward_manager = RewardManager(CONFIG)
    reward_manager.initialize_rewards(loader.get_coordinates())

    uavs = [UAV(vehicle_id=i, start_node=None, schedule={}) for i in range(CONFIG["num_uav"])]

     # 1. Get time information
    takeoff_time = CONFIG["uav_takeoff_time"]
    landing_time = CONFIG["uav_landing_time"]
    flight_time = CONFIG["uav_flight_time"] - takeoff_time - landing_time

    # 2. UGV speed information
    ugv_slow = CONFIG["ugv_takeoffland_speed"]  # Takeoff/landing
    ugv_fast = CONFIG["ugv_drive_speed"]        # During UAV flight

    # 3. Calculate movement distance (total UGV movement distance during one UAV slot)
    distance_during_takeoff = ugv_slow * takeoff_time
    distance_during_flight  = ugv_fast * flight_time
    distance_during_landing = ugv_slow * landing_time

    total_distance = distance_during_takeoff + distance_during_flight + distance_during_landing

     # 여기서 interest_points 중 이름만 출력하고 시작 점 이름 입력 받기
    # print("\n🚀 사용 가능한 시작점 목록:")
    # for name, x, y in interest_points:
    #     print(f"- {name}")
    
    # # 시작점 입력 받기
    # while True:
    #     start_point_name = input("\n시작점 이름을 입력하세요: ")
    #     # 입력된 이름이 유효한지 확인
    #     valid_names = [name for name, _, _ in interest_points]
    #     if start_point_name in valid_names:
    #         break
    #     print(f"⚠️  잘못된 시작점 이름입니다. 사용 가능한 이름: {', '.join(valid_names)}")
    
    # 선택된 시작점의 좌표 찾기
    # initial_point = next((point for name, point in zip(valid_names, [point for _, point in interest_points]) if name == start_point_name))
    initial_point = (interest_points[0][1], interest_points[0][2])  # (x, y) 임시로 이렇게 설정

    # 스케줄 불러오기
    with open("uav_schedule.json", "r") as f:
        uav_schedule = json.load(f)

    # Process schedules
    total_slots = max(len(v) for v in uav_schedule.values()) * CONFIG["uav_flight_charge_gcd"]
    
    for uav_id in range(CONFIG["num_uav"]):
        uav_key = f"uav_{uav_id+1}"
        schedule = uav_schedule[uav_key]
        flight_slots = int(CONFIG["uav_flight_time"] / CONFIG["uav_flight_charge_gcd"])
        
        original_schedule = schedule.copy()
        
        for t in range(len(original_schedule)):
            if original_schedule[t] == 1:
                for i in range(flight_slots):
                    slot_idx = (t + i) % len(schedule)
                    schedule[slot_idx] = 1
                    
        uav_schedule[uav_key] = schedule
        print(f"Updated schedule for {uav_key}: {schedule}")

    # Run simulation
    simulation_start_time = time.time()
    simulation_manager = SimulationManager(
        CONFIG, 
        coordinates, 
        uav_schedule, 
        initial_point, 
        max_simulation_time,
        loader
    )
    
    # 시뮬레이션 실행
    for t in range(0, max_simulation_time, time_step):
        simulation_manager.step(time_step)
    
    ugv_simulation_data, uav_simulation_data = simulation_manager.get_simulation_data()
    
    simulation_time = time.time() - simulation_start_time
    print(f"\n⏱️ Simulation 실행 시간: {simulation_time:.2f}초")

    # Save simulation data
    with open("ugv_simulation_data.json", "w") as f:
        json.dump(ugv_simulation_data, f)
    print("✅ UGV simulation data saved to ugv_simulation_data.json")

    with open("uav_simulation_data.json", "w") as f:
        json.dump(uav_simulation_data, f)
    print("✅ UAV simulation data saved to uav_simulation_data.json")

    # Process UAV flight data and run TOP for each flight
    final_results = {}
    top_execution_times = {}  # 각 UAV의 TOP 실행 시간 저장

    for uav_index in range(CONFIG["num_uav"]):
        uav_start_time = time.time()
        uav_states = uav_simulation_data["uav_states"][uav_index]
        uav_start_points = uav_simulation_data["uav_start_points"][uav_index]
        uav_end_points = uav_simulation_data["uav_end_points"][uav_index]

        # Find all flight segments (from takeoff to landing)
        flight_segments = []
        current_segment = None
        
        for i in range(len(uav_states)):
            if uav_states[i] == "takeoff" and current_segment is None:
                current_segment = {
                    "start_point": uav_start_points[i],
                    "start_time": i,
                    "end_point": None,
                    "end_time": None
                }
            elif uav_states[i] == "landing" and current_segment is not None:
                current_segment["end_point"] = uav_end_points[i]
                current_segment["end_time"] = i
                flight_segments.append(current_segment)
                current_segment = None

        # Run TOP for each flight segment
        uav_results = []
        segment_times = []  # 각 세그먼트의 실행 시간 저장
        
        for segment_idx, segment in enumerate(flight_segments):
            segment_start_time = time.time()
            print(f"\n🚁 Processing flight segment {segment_idx + 1} for UAV {uav_index+1}:")
            print(f"Start point: ({float(segment['start_point'][0]):.2f}, {float(segment['start_point'][1]):.2f})")
            print(f"End point: ({float(segment['end_point'][0]):.2f}, {float(segment['end_point'][1]):.2f})")
            
            # Initialize PipelineManager for this flight segment
            pipeline_manager = PipelineManager(
                env=env,
                uavs=[uavs[uav_index]],
                start=segment["start_point"],
                end=segment["end_point"],
                config=CONFIG,
                reward_manager=reward_manager
            )
            
            # Run the pipeline steps
            pipeline_manager.run()
            result = pipeline_manager.get_result()
            
            # Store only the final results (after reattachment)
            if "step_reattachment" in result:
                segment_result = {
                    "segment_id": segment_idx + 1,
                    "start_point": [float(x) for x in segment["start_point"]],
                    "end_point": [float(x) for x in segment["end_point"]],
                    "start_time": segment["start_time"],
                    "end_time": segment["end_time"],
                    "paths": [[list(map(float, point)) for point in path] 
                             for paths in result["step_reattachment"]["reattached_paths"].values()
                             for path in paths]
                }
                uav_results.append(segment_result)
            
            segment_time = time.time() - segment_start_time
            segment_times.append(segment_time)
            print(f"⏱️ Segment {segment_idx + 1} 실행 시간: {segment_time:.2f}초")
        
        final_results[f"uav_{uav_index+1}"] = uav_results
        
        uav_time = time.time() - uav_start_time
        top_execution_times[f"uav_{uav_index+1}"] = {
            "total_time": uav_time,
            "segment_times": segment_times
        }
        print(f"\n⏱️ UAV {uav_index+1} 총 실행 시간: {uav_time:.2f}초")

    # Save final results and execution times
    print("\n💾 Saving final results and execution times...")
    
    results_with_times = {
        "results": final_results,
        "execution_times": {
            "total_time": time.time() - total_start_time,
            "simulation_time": simulation_time,
            "top_times": top_execution_times
        }
    }
    
    with open("top_final_results.json", "w") as f:
        json.dump(results_with_times, f, indent=2)
    print("✅ Final results and execution times saved to top_final_results.json")
    
    total_time = time.time() - total_start_time
    print(f"\n=== 전체 실행 완료: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
    print(f"⏱️ 총 실행 시간: {total_time:.2f}초")