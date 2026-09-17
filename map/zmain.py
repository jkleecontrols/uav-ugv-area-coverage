from zpipelinemanager import PipelineManager
from zmodelsvehicle import UAV, UGV
from zdataloader import DataLoader
from zconfig import CONFIG
from zhelp import SimulationManager
from zrewardmanager import RewardManager
import json
import time
from datetime import datetime
import numpy as np
from typing import List, Tuple

def load_environment():
    """환경 설정 및 데이터 로딩"""
    print("\n=== 환경 설정 및 데이터 로딩 ===")
    
    # CSV 파일로부터 좌표 불러오기
    loader = DataLoader(CONFIG["csv_path"])
    coordinates = [tuple(coord) for coord in loader.get_grid_points()]
    interest_points = loader.get_named_points()
    
    # 리워드 매니저 초기화
    reward_manager = RewardManager(CONFIG)
    reward_manager.initialize_rewards(coordinates)
    
    return loader, reward_manager, coordinates, interest_points

def initialize_vehicles():
    """UAV와 UGV 초기화"""
    print("\n=== 차량 초기화 ===")
    
    # CSV 파일로부터 좌표 불러오기
    loader = DataLoader(CONFIG["csv_path"])
    boundary_coordinates = loader.get_boundary_points()  # UGV 경로용 원본 좌표
    grid_coordinates = loader.get_grid_points()  # UAV 경로용 그리드 좌표
    
    # 시작점 설정 (73번 인덱스의 좌표)
    start_point = boundary_coordinates[73]  # CSV 파일의 73번 인덱스 좌표
    print(f"🚀 시작점 설정: ({start_point[0]:.2f}, {start_point[1]:.2f})")
    
    # 도착점 계산 (시작점에서 6000m 떨어진 지점)
    end_point = calculate_end_point(boundary_coordinates, start_point, 6000)
    print(f"🎯 도착점 계산: ({end_point[0]:.2f}, {end_point[1]:.2f})")
    
    # UGV 이동거리 정보 출력
    print("\n=== UGV 이동거리 정보 ===")
    print(f"시작점: ({start_point[0]:.2f}, {start_point[1]:.2f})")
    print(f"도착점: ({end_point[0]:.2f}, {end_point[1]:.2f})")
    print(f"목표 거리: 6000.00m")
    
    # UAV 초기화 (FOV 파라미터 포함)
    uavs = [UAV(vehicle_id=i, 
                start_node=start_point, 
                schedule={},
                height=CONFIG["uav_height"],
                fov_angle=np.radians(CONFIG["uav_fov_angle"]),
                aspect_ratio=CONFIG["uav_aspect_ratio"]) 
            for i in range(CONFIG["num_uav"])]
    
    # UGV 초기화
    ugvs = [UGV(vehicle_id=i,
                start_node=start_point,
                speed=CONFIG["ugv_drive_speed"])
            for i in range(CONFIG["num_ugv"])]
    
    return uavs, ugvs, start_point, end_point, grid_coordinates

def calculate_end_point(coordinates: List[Tuple[float, float]], start_point: Tuple[float, float], target_distance: float) -> Tuple[float, float]:
    """시작점에서 테두리를 따라 target_distance만큼 이동한 도착점 계산"""
    current_pos = start_point
    remaining_distance = target_distance
    current_index = 0
    
    while remaining_distance > 0:
        # 다음 좌표 찾기
        next_index = (current_index + 1) % len(coordinates)
        next_pos = coordinates[next_index]
        
        # 현재 좌표에서 다음 좌표까지의 거리 계산
        segment_distance = ((next_pos[0] - current_pos[0])**2 + (next_pos[1] - current_pos[1])**2)**0.5
        
        if segment_distance <= remaining_distance:
            # 전체 세그먼트를 이동
            remaining_distance -= segment_distance
            current_pos = next_pos
            current_index = next_index
        else:
            # 세그먼트의 일부만 이동
            ratio = remaining_distance / segment_distance
            end_x = current_pos[0] + (next_pos[0] - current_pos[0]) * ratio
            end_y = current_pos[1] + (next_pos[1] - current_pos[1]) * ratio
            return (end_x, end_y)
    
    return current_pos

def run_simulation(env, uavs, ugvs, start_point, end_point, grid_coordinates, max_simulation_time, reward_manager):
    """시뮬레이션 실행"""
    print("\n=== 시뮬레이션 실행 ===")
    
    # UAV 스케줄 생성 (20분 비행, 14분 충전)
    uav_schedule = {}
    flight_time = CONFIG["uav_flight_time"]  # 1200초 (20분)
    charge_time = CONFIG["uav_charge_time"]  # 840초 (14분)
    cycle_time = flight_time + charge_time   # 2040초 (34분)
    
    for uav in uavs:
        schedule = []
        remaining_time = max_simulation_time
        
        while remaining_time > 0:
            # 비행 시간 추가
            flight_segment = min(remaining_time, flight_time)
            schedule.extend([1] * flight_segment)  # 1은 비행 상태
            remaining_time -= flight_segment
            
            if remaining_time <= 0:
                break
                
            # 충전 시간 추가
            charge_segment = min(remaining_time, charge_time)
            schedule.extend([0] * charge_segment)  # 0은 충전 상태
            remaining_time -= charge_segment
        
        uav_schedule[f"uav_{uav.vehicle_id}"] = schedule
        print(f"UAV {uav.vehicle_id} schedule created: {len(schedule)} seconds "
              f"({len(schedule)//60} minutes)")
    
    simulation_start_time = time.time()
    simulation_manager = SimulationManager(
        CONFIG, 
        grid_coordinates,  # UAV 경로용 그리드 좌표
        uav_schedule,
        start_point,  # 모든 차량의 시작점
        max_simulation_time,
        env,
        reward_manager
    )
    
    # PipelineManager를 사용하여 UAV 경로 생성
    pipeline_manager = PipelineManager(
        env=env,
        uavs=uavs,
        start=start_point,  # 모든 차량의 시작점
        end=end_point,      # 모든 차량의 도착점
        config=CONFIG,
        reward_manager=reward_manager
    )
    
    # 경로 생성 및 최적화 실행
    pipeline_manager.run()
    result = pipeline_manager.get_result()
    
    # 각 UAV에 경로 설정
    for uav in uavs:
        if (result and 
            "best_solution" in result and 
            "paths" in result["best_solution"] and 
            f"uav_{uav.vehicle_id}" in result["best_solution"]["paths"]):
            uav_path = result["best_solution"]["paths"][f"uav_{uav.vehicle_id}"]
            if uav_path is not None and uav_path.size > 0:  # numpy 배열의 크기 확인
                uav.set_path(uav_path)
                print(f"UAV {uav.vehicle_id} 경로 설정 완료: {uav_path.shape[0]}개의 포인트")
            else:
                print(f"⚠️ UAV {uav.vehicle_id} 경로가 비어있습니다.")
                # 기본 경로 설정 (시작점에서 도착점으로)
                uav.set_path([start_point, end_point])
        else:
            print(f"⚠️ UAV {uav.vehicle_id} 경로를 찾을 수 없습니다.")
            # 기본 경로 설정 (시작점에서 도착점으로)
            uav.set_path([start_point, end_point])
    
    # 시뮬레이션 데이터 저장을 위한 리스트 초기화
    ugv_positions = []
    uav_positions = [[] for _ in range(CONFIG["num_uav"])]
    uav_states = [[] for _ in range(CONFIG["num_uav"])]
    
    # UGV 이동거리 계산을 위한 변수
    total_distance = 0.0
    last_position = start_point
    
    # 시뮬레이션 실행
    for t in range(0, max_simulation_time, CONFIG["simulation_time_step"]):
        simulation_manager.step(CONFIG["simulation_time_step"])
        
        # 현재 위치와 상태 저장
        current_position = simulation_manager.ugv.current_position
        ugv_positions.append(current_position)
        
        # UGV 이동거리 계산
        distance = ((current_position[0] - last_position[0])**2 + 
                   (current_position[1] - last_position[1])**2)**0.5
        total_distance += distance
        last_position = current_position
        
        for i, uav in enumerate(simulation_manager.uavs):
            uav_positions[i].append(uav.current_position)
            uav_states[i].append(uav.state)
        
        # UAV 상태 업데이트 및 출력
        for uav in uavs:
            print(f"UAV {uav.vehicle_id}: State={uav.state}, "
                  f"Remaining Flight Time={uav.remaining_flight_time:.1f}s, "
                  f"Position=({uav.current_position[0]:.2f}, {uav.current_position[1]:.2f})")
    
    # UGV 이동거리 출력
    print("\n=== UGV 이동거리 정보 ===")
    print(f"시작점: ({start_point[0]:.2f}, {start_point[1]:.2f})")
    print(f"도착점: ({end_point[0]:.2f}, {end_point[1]:.2f})")
    print(f"UGV 이동거리: {total_distance:.2f}m")
    print(f"목표 거리: 6000.00m")
    print(f"오차: {abs(total_distance - 6000):.2f}m")
    
    # 시뮬레이션 데이터 저장
    ugv_simulation_data = {
        "position": [[float(x) for x in pos] for pos in ugv_positions],
        "time": list(range(0, max_simulation_time, CONFIG["simulation_time_step"])),
        "total_distance": total_distance,
        "start_point": [float(x) for x in start_point],
        "end_point": [float(x) for x in end_point]
    }
    
    # UAV 시작점과 끝점을 Python 리스트로 변환
    uav_start_points = []
    uav_end_points = []
    for uav in uavs:
        uav_start_points.append([start_point])  # 모든 UAV의 시작점
        uav_end_points.append([end_point])      # 모든 UAV의 도착점
    
    uav_simulation_data = {
        "uav_positions": [[[float(x) for x in pos] + [uav.current_heading] for pos in uav_pos] for uav_pos, uav in zip(uav_positions, uavs)],
        "uav_states": uav_states,
        "uav_start_points": uav_start_points,
        "uav_end_points": uav_end_points,
        "time": list(range(0, max_simulation_time, CONFIG["simulation_time_step"]))
    }
    
    # JSON 파일로 저장
    with open("map/ugv_simulation_data.json", "w") as f:
        json.dump(ugv_simulation_data, f)
    print("✅ UGV simulation data saved to map/ugv_simulation_data.json")
    
    with open("map/uav_simulation_data.json", "w") as f:
        json.dump(uav_simulation_data, f)
    print("✅ UAV simulation data saved to map/uav_simulation_data.json")
    
    simulation_time = time.time() - simulation_start_time
    print(f"\n⏱️ Simulation 실행 시간: {simulation_time:.2f}초")
    return ugv_simulation_data, uav_simulation_data, simulation_time

def process_flight_segments(uav_index, uav_states, uav_start_points, uav_end_points):
    """비행 세그먼트 처리"""
    flight_segments = []
    current_segment = None
    
    for i in range(len(uav_states)):
        if uav_states[i] == "flying" and current_segment is None:
            current_segment = {
                "start_point": uav_start_points[i],
                "start_time": i,
                "end_point": None,
                "end_time": None
            }
        elif uav_states[i] == "charging" and current_segment is not None:
            current_segment["end_point"] = uav_end_points[i]
            current_segment["end_time"] = i
            flight_segments.append(current_segment)
            current_segment = None
    
    return flight_segments

def run_top_for_segment(env, uav, segment, reward_manager):
    """단일 비행 세그먼트에 대한 TOP 실행"""
    print(f"\n🚁 Processing flight segment:")
    print(f"Start point: ({float(segment['start_point'][0]):.2f}, {float(segment['start_point'][1]):.2f})")
    print(f"End point: ({float(segment['end_point'][0]):.2f}, {float(segment['end_point'][1]):.2f})")
    
    pipeline_manager = PipelineManager(
        env=env,
        uavs=[uav],
        start=segment["start_point"],
        end=segment["end_point"],
        config=CONFIG,
        reward_manager=reward_manager
    )
    
    pipeline_manager.run()
    return pipeline_manager.get_result()

def save_results(final_results, execution_times, total_time):
    """결과 저장"""
    print("\n💾 Saving final results and execution times...")
    
    results_with_times = {
        "results": final_results,
        "execution_times": execution_times,
        "total_time": total_time
    }
    
    with open("map/zz_result1.json", "w") as f:
        json.dump(results_with_times, f, indent=2)
    print("✅ Final results and execution times saved to map/zz_result.json")

def main():
    total_start_time = time.time()
    print(f"\n=== TOP Algorithm 실행 시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    
    # 1. 환경 설정 및 데이터 로딩
    env, reward_manager, coordinates, interest_points = load_environment()
    
    # 2. 차량 초기화
    uavs, ugvs, start_point, end_point, grid_coordinates = initialize_vehicles()
    print(f"🚀 시작점 설정: ({start_point[0]:.2f}, {start_point[1]:.2f})")
    print(f"🎯 도착점 계산: ({end_point[0]:.2f}, {end_point[1]:.2f})")
    
    # 3. 설정에 시작점과 도착점 추가
    CONFIG["start_point"] = start_point
    CONFIG["end_point"] = end_point
    
    # 4. 시뮬레이션 실행
    max_simulation_time = 1200  # 20분
    ugv_simulation_data, uav_simulation_data, simulation_time = run_simulation(
        env, uavs, ugvs, start_point, end_point, grid_coordinates, max_simulation_time, reward_manager
    )
    
    # 5. 각 UAV의 비행 세그먼트 처리 및 TOP 실행
    final_results = {}
    top_execution_times = {}
    
    for uav_index in range(CONFIG["num_uav"]):
        uav_start_time = time.time()
        uav_states = uav_simulation_data["uav_states"][uav_index]
        uav_start_points = uav_simulation_data["uav_start_points"][uav_index]
        uav_end_points = uav_simulation_data["uav_end_points"][uav_index]
        
        # 비행 세그먼트 처리
        flight_segments = process_flight_segments(
            uav_index, uav_states, uav_start_points, uav_end_points
        )
        
        # 각 세그먼트에 대해 TOP 실행
        uav_results = []
        segment_times = []
        
        for segment_idx, segment in enumerate(flight_segments):
            segment_start_time = time.time()
            
            # TOP 실행
            result = run_top_for_segment(env, uavs[uav_index], segment, reward_manager)
            
            # 결과 저장
            if "step_reattachment" in result:
                segment_result = {
                    "segment_id": segment_idx + 1,
                    "start_point": [float(x) for x in segment["start_point"]],
                    "end_point": [float(x) for x in segment["end_point"]],
                    "start_time": segment["start_time"],
                    "end_time": segment["end_time"],
                    "paths": [[list(map(float, point)) for point in path] 
                             for paths in result["step_reattachment"]["reattached_paths"].values()
                             for path in paths],
                    "fov_rotations": result.get("fov_rotations", [])  # FOV 회전각 정보 추가
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
    
    # 6. 실행 시간 정보 수집
    execution_times = {
        "total_time": time.time() - total_start_time,
        "simulation_time": simulation_time,
        "top_times": top_execution_times
    }
    
    # 7. 결과 저장
    save_results(final_results, execution_times, time.time() - total_start_time)
    
    print(f"\n=== 전체 실행 완료: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===")
    print(f"⏱️ 총 실행 시간: {execution_times['total_time']:.2f}초")

if __name__ == "__main__":
    main()