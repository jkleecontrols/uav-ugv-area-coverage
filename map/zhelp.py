import math
import numpy as np
from zmodelsvehicle import UGV, UAV
from typing import Tuple, List
import csv

def calculate_end_point(coordinates: List[Tuple[float, float]], start_point: Tuple[float, float], target_distance: float) -> Tuple[float, float]:
    """시작점(73번 인덱스)에서 시작하여 정확히 target_distance(6000m) 떨어진 도착점 계산"""
    # 시작점(73번 인덱스)부터 시작
    current_index = 73
    current_pos = coordinates[current_index]
    remaining_distance = target_distance
    
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
            # 세그먼트의 일부만 이동하여 정확히 target_distance가 되도록 함
            ratio = remaining_distance / segment_distance
            end_x = current_pos[0] + (next_pos[0] - current_pos[0]) * ratio
            end_y = current_pos[1] + (next_pos[1] - current_pos[1]) * ratio
            return (end_x, end_y)
    
    return current_pos

class DataLoader:
    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.coordinates = self._load_coordinates()
    
    def _load_coordinates(self):
        """CSV 파일에서 좌표를 로드합니다."""
        coordinates = []
        with open(self.csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # 헤더 건너뛰기
            for row in reader:
                x = float(row[3])  # x 좌표
                y = float(row[4])  # y 좌표
                coordinates.append((x, y))
        return coordinates
    
    def get_grid_points(self):
        """그리드 포인트를 반환합니다."""
        return self.coordinates

class SimulationManager:
    def __init__(self, config, coordinates, uav_schedule, start_point, max_simulation_time, env, reward_manager):
        self.config = config
        self.coordinates = coordinates
        self.uav_schedule = uav_schedule
        self.start_point = start_point
        self.max_simulation_time = max_simulation_time
        self.env = env
        self.reward_manager = reward_manager
        
        # UGV 초기화
        self.ugv = UGV(vehicle_id=0, start_node=start_point, speed=config["ugv_drive_speed"])
        
        # UAVs 초기화
        self.uavs = []
        for i in range(config["num_uav"]):
            uav = UAV(vehicle_id=i, 
                     start_node=start_point,
                     schedule=uav_schedule.get(f"uav_{i}", {}),
                     height=config["uav_height"],
                     fov_angle=np.radians(config["uav_fov_angle"]),
                     aspect_ratio=config["uav_aspect_ratio"])
            self.uavs.append(uav)
        
        # UGV 경로 계산
        self.ugv_path = self._calculate_ugv_path()
        self.current_coordinate_index = 0  # 현재 좌표 인덱스 초기화
        
        # 시뮬레이션 상태
        self.current_time = 0
        self.covered_points = set()
        
        # 시뮬레이션 결과 저장
        self.ugv_positions = []
        self.uav_positions = [[] for _ in range(config["num_uav"])]
        self.uav_states = [[] for _ in range(config["num_uav"])]
        
        # 보상 이력 초기화
        self.reward_history = []
        
    def _calculate_ugv_path(self):
        """UGV의 경로를 계산합니다."""
        # CSV 파일의 좌표 목록 가져오기
        loader = DataLoader(self.config["csv_path"])
        coordinates = [tuple(coord) for coord in loader.get_grid_points()]
        
        # 시작점과 도착점 설정
        start_point = self.start_point
        end_point = calculate_end_point(coordinates, start_point, self.config["ugv_drive_speed"] * 1200)  # 5m/s * 1200초 = 6000m
        
        # 시작점의 인덱스 찾기
        start_index = 0
        min_dist = float('inf')
        for i, coord in enumerate(coordinates):
            dist = ((coord[0] - start_point[0])**2 + (coord[1] - start_point[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                start_index = i
        
        # 도착점의 인덱스 찾기
        end_index = 0
        min_dist = float('inf')
        for i, coord in enumerate(coordinates):
            dist = ((coord[0] - end_point[0])**2 + (coord[1] - end_point[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
                end_index = i
        
        # 경로 생성
        path = []
        current_index = start_index
        target_index = end_index
        
        while current_index != target_index:
            path.append(coordinates[current_index])
            current_index = (current_index + 1) % len(coordinates)
        
        path.append(coordinates[target_index])
        return path
        
    def step(self, time_step):
        """시뮬레이션 한 스텝 실행"""
        # UGV 이동 (계산된 경로를 따라 이동)
        current_target = self.ugv_path[self.current_coordinate_index]
        next_target = self.ugv_path[(self.current_coordinate_index + 1) % len(self.ugv_path)]
        
        # 현재 위치에서 다음 목표까지의 거리 계산
        distance_to_next = self.calculate_distance(self.ugv.current_position, next_target)
        
        # 다음 위치 계산 (속도 × 시간 간격만큼 이동)
        next_position = self.get_next_position(
            self.ugv.current_position,
            next_target,
            self.config["ugv_drive_speed"],
            time_step
        )
        
        # UGV 위치 업데이트
        self.ugv.current_position = next_position
        self.ugv.current_node = next_position
        
        # 다음 좌표에 도달했으면 다음 좌표로 전환
        if distance_to_next < self.config["ugv_drive_speed"] * time_step:
            self.current_coordinate_index = (self.current_coordinate_index + 1) % len(self.ugv_path)
            print(f"🔄 UGV reached coordinate {self.current_coordinate_index}: {next_target}")
        
        # UAV 상태 업데이트
        for uav in self.uavs:
            uav.update_state(time_step)
            
            # UAV가 비행 중이면 위치 업데이트
            if uav.state == "flying":
                uav.move(time_step)
            # UAV가 충전 중이면 UGV와 함께 이동
            elif uav.state == "charging":
                uav.current_position = self.ugv.current_position
                uav.current_node = self.ugv.current_position
        
        # 리워드 업데이트
        current_reward = self.reward_manager.get_current_rewards()
        self.reward_history.append(current_reward)
        
        self.current_time += time_step
        
    def get_simulation_data(self):
        """시뮬레이션 데이터 반환"""
        return {
            "ugv_positions": [self.ugv.current_position],
            "uav_positions": [[uav.current_position] for uav in self.uavs],
            "uav_states": [[uav.state] for uav in self.uavs],
            "reward_history": self.reward_history
        }

    def calculate_distance(self, point1, point2):
        return ((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)**0.5

    def interpolate_position(self, start, end, ratio):
        return (
            round(start[0] + (end[0] - start[0]) * ratio, 2),
            round(start[1] + (end[1] - start[1]) * ratio, 2)
        )

    def get_next_position(self, current_pos, target_pos, speed, time_step):
        distance = self.calculate_distance(current_pos, target_pos)
        if distance <= speed * time_step:
            return (round(target_pos[0], 2), round(target_pos[1], 2))
        ratio = (speed * time_step) / distance
        return self.interpolate_position(current_pos, target_pos, ratio)

    def get_reward_history(self):
        """리워드 히스토리 반환"""
        return self.reward_history