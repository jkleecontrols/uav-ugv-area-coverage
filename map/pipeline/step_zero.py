import numpy as np
from typing import Dict, List, Tuple, Any, Set
from matplotlib.path import Path
from .tabu_search import TabuSearch
from zmodelsvehicle import UAV
from zrewardmanager import RewardManager
from zhelp import SimulationManager

class StepZero:
    def __init__(self, env, uavs: List[UAV], config: Dict, reward_manager: RewardManager):
        self.env = env
        self.uavs = uavs
        self.config = config
        self.reward_manager = reward_manager
        
        # 시작점과 도착점 설정
        self.start = config["start_point"]
        self.end = config["end_point"]
        
        # FOV 관련 파라미터
        self.height = config["uav_height"]
        self.fov_angle = np.radians(config["uav_fov_angle"])
        self.aspect_ratio = config["uav_aspect_ratio"]
        
        # 타원 파라미터
        self.major_axis = config["major_axis"]  # UAV 최대 비행거리
        self.minor_axis = self.major_axis * 0.6  # 단축은 장축의 60%
        
        # 그리드 포인트
        self.grid_points = env.get_grid_points()
        
    def _calculate_ellipse_parameters(self) -> Tuple[Tuple[float, float], float, float, float]:
        """타원의 중심, 장축, 단축, 회전각 계산"""
        # 타원의 중심 계산
        center_x = (self.start[0] + self.end[0]) / 2
        center_y = (self.start[1] + self.end[1]) / 2
        center = (center_x, center_y)
        
        # 타원의 회전각 계산 (두 초점을 잇는 선의 각도)
        rotation = np.arctan2(self.end[1] - self.start[1], 
                            self.end[0] - self.start[0])
        
        return center, rotation, self.major_axis, self.minor_axis
        
    def _is_point_in_ellipse(self, point: Tuple[float, float], 
                           center: Tuple[float, float], 
                           rotation: float) -> bool:
        """점이 타원 내부에 있는지 확인"""
        # 점을 타원의 중심으로 이동
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        
        # 회전된 좌표계로 변환
        cos_r, sin_r = np.cos(rotation), np.sin(rotation)
        rotated_x = dx * cos_r + dy * sin_r
        rotated_y = -dx * sin_r + dy * cos_r
        
        # 타원 방정식 검사
        return (rotated_x / self.major_axis)**2 + (rotated_y / self.minor_axis)**2 <= 1
        
    def _get_available_guard_nodes(self) -> List[Tuple[float, float]]:
        """타원 내부의 가용가드 노드 찾기"""
        # 타원 파라미터 계산
        center, rotation, _, _ = self._calculate_ellipse_parameters()
        
        # 타원 내부의 점들 찾기
        available_nodes = []
        for point in self.grid_points:
            if self._is_point_in_ellipse(point, center, rotation):
                available_nodes.append(point)
        
        print(f"  - 타원 내부의 가용가드 노드 수: {len(available_nodes)}")
        return available_nodes
        
    def run(self) -> Dict[str, Any]:
        """가용가드 노드 식별 실행"""
        print("\n=== Step Zero: Available Guard Nodes Identification ===")
        
        # 가용가드 노드 식별
        available_nodes = self._get_available_guard_nodes()
        
        # 타원 파라미터 계산
        center, rotation, major_axis, minor_axis = self._calculate_ellipse_parameters()
        
        # 각 UAV에 대한 빈 경로 생성
        paths = {}
        for uav in self.uavs:
            paths[f"uav_{uav.vehicle_id}"] = np.array([self.start, self.end])
        
        return {
            "paths": paths,
            "available_nodes": available_nodes,
            "start_point": self.start,
            "end_point": self.end,
            "ellipse_parameters": {
                "center": center,
                "rotation": rotation,
                "major_axis": major_axis,
                "minor_axis": minor_axis
            }
        }