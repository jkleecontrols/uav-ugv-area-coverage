import numpy as np
from typing import Dict, List, Tuple, Any, Set
from matplotlib.path import Path
from .tabu_search import TabuSearch
from zmodelsvehicle import UAV
from zrewardmanager import RewardManager
import math

class StepOne:
    def __init__(self, env, initial_solution: Dict[str, Any], config: Dict, reward_manager: RewardManager):
        """
        Args:
            env: DataLoader 환경
            initial_solution: Step Zero의 결과 (가용가드 노드)
            config: 설정값
            reward_manager: 리워드 매니저
        """
        self.env = env
        self.initial_solution = initial_solution
        self.config = config
        self.reward_manager = reward_manager
        
        # FOV 관련 파라미터
        self.height = config["uav_height"]
        self.fov_angle = np.radians(config["uav_fov_angle"])
        self.aspect_ratio = config["uav_aspect_ratio"]
        
        # 그리드 포인트
        self.grid_points = env.get_grid_points()
        
        # 초기 정보
        self.available_nodes = initial_solution["available_nodes"]
        self.start = initial_solution["start_point"]
        self.end = initial_solution["end_point"]
        
        # UAV 수
        self.num_uavs = config["num_uav"]
        
    def _calculate_fov_area(self, uav_pos: Tuple[float, float], heading: float) -> Set[Tuple[float, float]]:
        """FOV 영역 내부의 그리드 포인트들 반환"""
        fov_width = 2 * self.height * np.tan(self.fov_angle/2)
        fov_length = fov_width * self.aspect_ratio
        
        cos_h, sin_h = np.cos(heading), np.sin(heading)
        
        # FOV 영역의 꼭지점 계산
        corners = [
            (uav_pos[0] + fov_length/2 * cos_h - fov_width/2 * sin_h,
             uav_pos[1] + fov_length/2 * sin_h + fov_width/2 * cos_h),
            (uav_pos[0] + fov_length/2 * cos_h + fov_width/2 * sin_h,
             uav_pos[1] + fov_length/2 * sin_h - fov_width/2 * cos_h),
            (uav_pos[0] - fov_length/2 * cos_h + fov_width/2 * sin_h,
             uav_pos[1] - fov_length/2 * sin_h - fov_width/2 * cos_h),
            (uav_pos[0] - fov_length/2 * cos_h - fov_width/2 * sin_h,
             uav_pos[1] - fov_length/2 * sin_h + fov_width/2 * cos_h)
        ]
        
        # FOV 영역 내부의 그리드 포인트 찾기
        fov_path = Path(corners)
        covered_points = set()
        for point in self.grid_points:
            if fov_path.contains_point(point):
                covered_points.add(tuple(point))
        
        return covered_points
        
    def _get_covered_points(self, uav_pos: Tuple[float, float], heading: float) -> Set[Tuple[float, float]]:
        """FOV 영역 내부의 그리드 포인트들 반환"""
        fov_corners = self._calculate_fov_area(uav_pos, heading)
        fov_path = Path(fov_corners)
        
        covered_points = set()
        for point in self.grid_points:
            if fov_path.contains_point(point):
                covered_points.add(tuple(point))
        return covered_points
        
    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """두 점 간의 거리 계산"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        
    def _calculate_path_length(self, path: List[Tuple[Tuple[float, float], float]]) -> float:
        """경로의 총 길이 계산"""
        return sum(self._calculate_distance(path[i][0], path[i+1][0]) 
                  for i in range(len(path)-1))
        
    def _find_next_position(self, current_pos, heading):
        """다음 위치를 찾습니다."""
        # 현재 위치에서 5m 이내의 도달 가능한 그리드 포인트 찾기
        reachable_points = []
        for point in self.grid_points:
            distance = ((point[0] - current_pos[0])**2 + (point[1] - current_pos[1])**2)**0.5
            if distance <= 5:  # 5m 이내
                # 현재 방향과의 각도 차이 계산
                angle = math.atan2(point[1] - current_pos[1], point[0] - current_pos[0])
                angle_diff = abs(angle - heading)
                if angle_diff > math.pi:
                    angle_diff = 2 * math.pi - angle_diff
                if angle_diff <= math.radians(5):  # ±5도 이내
                    reachable_points.append(point)
        
        if not reachable_points:
            return None
        
        # 각 후보 지점에 대해 FOV 영역 계산
        best_point = None
        max_new_points = -1
        
        for point in reachable_points:
            # FOV 영역 계산
            fov_points = self._calculate_fov_area(point, heading)
            
            # 새로운 포인트 수 계산 (available_nodes와의 차집합)
            available_nodes_set = set(tuple(node) for node in self.available_nodes)
            new_points = fov_points - available_nodes_set
            
            if len(new_points) > max_new_points:
                max_new_points = len(new_points)
                best_point = point
        
        return best_point
        
    def _generate_initial_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float]]:
        """초기 경로 생성"""
        path = [start]
        current_pos = start
        
        # 시작점에서 끝점까지의 방향 계산
        heading = np.arctan2(end[1] - start[1], end[0] - start[0])
        
        # 최대 경로 길이 설정 (UAV 비행 시간에 기반)
        max_path_length = self.config["uav_speed"] * self.config["uav_flight_time"]
        current_length = 0
        
        while current_length < max_path_length:
            # 다음 위치 찾기
            next_pos = self._find_next_position(current_pos, heading)
            if next_pos is None:
                break
                
            # 거리 계산 및 경로 길이 업데이트
            distance = self._calculate_distance(current_pos, next_pos)
            current_length += distance
            
            # 경로에 추가
            path.append(next_pos)
            current_pos = next_pos
            
            # 방향 업데이트
            heading = np.arctan2(end[1] - current_pos[1], end[0] - current_pos[0])
            
            # 끝점에 도달했는지 확인
            if self._calculate_distance(current_pos, end) < 5:
                break
        
        # 끝점 추가
        path.append(end)
        return path
        
    def run(self) -> Dict[str, Any]:
        """초기 경로 생성 실행"""
        print("\n=== Step One: Initial Path Generation ===")
        
        # 각 UAV에 대한 경로 생성
        paths = {}
        for i in range(self.num_uavs):
            uav_id = f"uav_{i}"
            print(f"\n  - {uav_id} 경로 생성 중...")
            
            # 초기 경로 생성
            path = self._generate_initial_path(self.start, self.end)
            paths[uav_id] = np.array(path)
            
            print(f"    - 경로 생성 완료: {len(path)}개의 포인트")
            print(f"    - 시작점: ({path[0][0]:.2f}, {path[0][1]:.2f})")
            print(f"    - 끝점: ({path[-1][0]:.2f}, {path[-1][1]:.2f})")
        
        return {
            "paths": paths,
            "available_nodes": self.available_nodes,
            "start_point": self.start,
            "end_point": self.end
        }
