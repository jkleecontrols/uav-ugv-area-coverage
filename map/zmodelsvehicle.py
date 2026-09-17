import math
from typing import List, Dict, Tuple, Optional
import numpy as np
from zconfig import CONFIG

def calculate_route(start_node, end_node):
    """
    Placeholder for route calculation between start_node and end_node.
    Replace with actual pathfinding logic (e.g., A*, Dijkstra).
    """
    return [start_node, end_node]

def get_nearest_node(current_node, node_list):
    """
    Returns the nearest node to current_node from node_list based on Euclidean distance.
    Assumes nodes are (x, y) tuples or have x and y attributes.
    """
    def distance(a, b):
        if isinstance(a, tuple):
            ax, ay = a
        else:
            ax, ay = a.x, a.y
        if isinstance(b, tuple):
            bx, by = b
        else:
            bx, by = b.x, b.y
        return math.sqrt((ax - bx)**2 + (ay - by)**2)

    return min(node_list, key=lambda node: distance(current_node, node))

class Node:
    def __init__(self, node_id, x, y, score):
        self.id = node_id
        self.x = x
        self.y = y
        self.score = score

class Vehicle:
    def __init__(self, vehicle_id, start_node, capacity):
        self.id = vehicle_id
        self.current_node = start_node
        self.route = [start_node]
        self.capacity = capacity
        self.score = 0

    def move_to(self, node):
        self.route.append(node)
        self.current_node = node
        self.score += node.score

class UAV(Vehicle):
    def __init__(self, vehicle_id, start_node=None, schedule=None, height=None, fov_angle=None, aspect_ratio=None):
        self.vehicle_id = vehicle_id
        self.start_node = start_node
        self.current_position = start_node if start_node else None
        self.current_heading = None
        self.current_rotation = 0
        self.height = height
        self.fov_angle = fov_angle
        self.aspect_ratio = aspect_ratio
        
        # 비행 시간 설정 (20분)
        self.flight_time = CONFIG["uav_flight_time"]  # 1200초
        self.remaining_flight_time = self.flight_time
        
        # 초기 상태 설정
        self.state = "flying"  # 시작하자마자 비행 시작
        
        # 경로 관련 변수
        self.path = []  # PipelineManager에서 생성된 경로
        self.current_path_index = 0  # 현재 경로의 인덱스
        
    def set_path(self, path: np.ndarray):
        """경로 설정"""
        if path is not None and path.size > 0:  # numpy 배열의 크기 확인
            self.path = path
            self.current_path_index = 0
            self.current_position = tuple(path[0])
        else:
            self.path = None
            self.current_path_index = 0
            self.current_position = None
        
    def update_state(self, time_step):
        """UAV의 상태를 업데이트"""
        if self.state == "flying":
            self.remaining_flight_time -= time_step
            if self.remaining_flight_time <= 0:
                self.state = "returning"  # 비행 시간이 끝나면 UGV로 복귀
                
    def move(self, time_step):
        """UAV 이동 (PipelineManager에서 생성된 경로 따라 이동)"""
        if self.state == "flying" and self.path is not None and len(self.path) > 0:
            if self.current_path_index < len(self.path) - 1:
                # 현재 위치와 다음 경로점
                current_point = self.path[self.current_path_index]
                next_point = self.path[self.current_path_index + 1]
                
                # 현재 위치에서 다음 경로점까지의 거리 계산
                distance = np.sqrt((next_point[0] - current_point[0])**2 + 
                                 (next_point[1] - current_point[1])**2)
                
                # 이동 가능한 거리 계산 (속도 × 시간)
                move_distance = CONFIG["uav_speed"] * time_step
                
                if move_distance >= distance:
                    # 다음 경로점으로 이동
                    self.current_position = tuple(next_point)
                    self.current_path_index += 1
                    
                    # 마지막 경로점에 도달했는지 확인
                    if self.current_path_index == len(self.path) - 1:
                        self.state = "returning"  # 경로 끝에 도달하면 복귀 상태로
                else:
                    # 경로를 따라 일부만 이동
                    ratio = move_distance / distance
                    self.current_position = (
                        current_point[0] + (next_point[0] - current_point[0]) * ratio,
                        current_point[1] + (next_point[1] - current_point[1]) * ratio
                    )
                
                # 진행 방향(헤딩) 업데이트
                if self.current_path_index < len(self.path) - 1:
                    next_point = self.path[self.current_path_index + 1]
                    dx = next_point[0] - self.current_position[0]
                    dy = next_point[1] - self.current_position[1]
                    self.current_heading = np.arctan2(dy, dx) * 180 / np.pi  # 라디안을 도로 변환
                else:
                    # 마지막 경로점에 도달했을 때의 헤딩
                    self.current_heading = self.current_heading  # 이전 헤딩 유지
                    
            else:
                # 경로의 끝에 도달
                self.state = "returning"
        elif self.state == "returning":
            # UGV로 복귀하는 로직 (현재는 단순화)
            self.current_position = self.start_node
            self.current_heading = None  # 복귀 중에는 헤딩 없음

class UGV(Vehicle):
    def __init__(self, vehicle_id, start_node, speed):
        super().__init__(vehicle_id, start_node, capacity=CONFIG["num_charging_pads"])
        self.speed = speed
        self.current_position = start_node  # current_node 대신 current_position 사용
        self.charging_pads = {}  # {pad_id: {"uav_id": None}}
        self.initialize_charging_pads()

    def initialize_charging_pads(self):
        """충전 패드 초기화"""
        for i in range(CONFIG["num_charging_pads"]):
            self.charging_pads[i] = {
                "uav_id": None
            }

    def assign_charging_pad(self, uav_id):
        """충전 패드 할당"""
        for pad_id, pad_info in self.charging_pads.items():
            if pad_info["uav_id"] is None:  # 빈 충전 패드 찾기
                pad_info["uav_id"] = uav_id
                return pad_id
        return None

    def release_charging_pad(self, pad_id):
        """충전 패드 해제"""
        if pad_id in self.charging_pads:
            self.charging_pads[pad_id]["uav_id"] = None
            return True
        return False

    def get_available_charging_pads(self):
        """사용 가능한 충전 패드 수 반환"""
        return sum(1 for pad_info in self.charging_pads.values() 
                  if pad_info["uav_id"] is None)

    def get_charging_status(self):
        """충전 상태 정보 반환"""
        return {
            "available_pads": self.get_available_charging_pads(),
            "charging_info": self.charging_pads
        }

    def move(self, time_step):
        """UGV 이동"""
        # 실제 이동은 SimulationManager에서 처리
        pass

class UAVBatteryManager:
    def __init__(self):
        self.battery_level = 100
        self.discharge_rate = CONFIG["uav_battery_discharge_rate"]
        self.charge_rate = CONFIG["uav_battery_charge_rate"]

    def discharge(self, time_spent):
        """배터리 방전"""
        self.battery_level = max(0, self.battery_level - time_spent * self.discharge_rate)

    def charge(self, time_spent):
        """배터리 충전"""
        self.battery_level = min(100, self.battery_level + time_spent * self.charge_rate)

    def get_uav_info(self):
        """UAV 배터리 정보 반환"""
        return {
            "battery_level": self.battery_level,
            "discharge_rate": self.discharge_rate,
            "charge_rate": self.charge_rate
        }

    def start_charging(self):
        """배터리 충전 시작"""
        self.battery_level = 100

    def end_charging(self):
        """배터리 충전 종료"""
        self.battery_level = 0

    def start_flying(self):
        """배터리 방전 시작"""
        self.battery_level = 0

    def end_flying(self):
        """배터리 방전 종료"""
        self.battery_level = 100

    def update(self, time_step):
        """배터리 상태 업데이트"""
        if self.battery_level > 0:
            self.battery_level -= time_step * self.discharge_rate
        else:
            self.battery_level = 0

    def is_fully_charged(self):
        """배터리가 완전히 충전되었는지 확인"""
        return self.battery_level == 100

class UAVRouteManager:
    def __init__(self, uav):
        self.uav = uav
        self._x = 0
        self._y = 0

    def get_route(self, destination):
        """경로 계산"""
        return calculate_route(self.uav.current_node, destination)

    def distance(self, node1, node2):
        """두 노드 간 거리 계산"""
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def get_uav_info(self):
        """UAV 정보 반환"""
        return {
            "battery": self.uav.get_battery_level(),
            "state": self.uav.state,
            "remaining_flight_time": self.uav.remaining_flight_time,
            "fov_rotation": self.uav.current_rotation
        }

    def find_nearest_node(self, nodes):
        """가장 가까운 노드 찾기"""
        return get_nearest_node(self.uav.current_node, nodes)

class UGVRouteManager:
    def __init__(self, ugv):
        self.ugv = ugv

    def get_route(self, destination):
        """경로 계산"""
        return calculate_route(self.ugv.current_node, destination)

    def find_nearest_node(self, nodes):
        """가장 가까운 노드 찾기"""
        return get_nearest_node(self.ugv.current_node, nodes)

    def get_ugv_info(self):
        """UGV 정보 반환"""
        return {
            "speed": self.ugv.speed,
            "charging_status": self.ugv.get_charging_status()
        }

class DistanceCalculator:
    @staticmethod
    def euclidean_distance(node1, node2):
        return math.sqrt(node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2