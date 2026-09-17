import time
import numpy as np
from typing import Dict, Tuple, List, Set
from matplotlib.path import Path

class RewardManager:
    def __init__(self, config):
        self.config = config
        # 좌표별 리워드 상태 관리
        self.node_rewards: Dict[Tuple[float, float], float] = {}
        self.visit_count: Dict[Tuple[float, float], int] = {}
        self.last_visit_time: Dict[Tuple[float, float], float] = {}
        
        # 리워드 파라미터
        self.initial_reward = 1.0          # 초기/최대 리워드
        self.min_reward = 0.1              # 최소 리워드
        self.revisit_penalty = 0.5         # 재방문 패널티 (초기 리워드의 50%)
        self.recovery_rate = 0.0002        # 초당 리워드 회복률 (0.02%/초)
        self.recovery_threshold = 0.8      # 리워드 회복 최대치 (초기값의 80%)
        
        # FOV 관련 파라미터
        self.fov_bonus = 0.2              # FOV 내 미방문 점 비율에 따른 보너스
        
    def initialize_rewards(self, grid_points: List[Tuple[float, float]]):
        """
        초기 리워드 설정
        Args:
            grid_points: 1m 간격 그리드 포인트 리스트
        """
        current_time = time.time()
        for point in grid_points:
            self.node_rewards[point] = self.initial_reward
            self.visit_count[point] = 0
            self.last_visit_time[point] = current_time
            
    def calculate_fov_points(self, uav_pos: Tuple[float, float], heading: float) -> Set[Tuple[float, float]]:
        """
        UAV의 현재 위치와 방향에 따른 FOV 내부의 점들을 계산
        Args:
            uav_pos: UAV 현재 위치 (x, y)
            heading: UAV 진행 방향 (라디안)
        Returns:
            FOV 내부의 그리드 포인트들
        """
        # FOV 파라미터
        height = self.config["uav_height"]
        fov_angle = np.radians(self.config["uav_fov_angle"])
        aspect_ratio = self.config["uav_aspect_ratio"]
        
        # FOV 영역 계산
        fov_width = 2 * height * np.tan(fov_angle/2)
        fov_length = fov_width * aspect_ratio
        
        # FOV 꼭지점 계산
        cos_h, sin_h = np.cos(heading), np.sin(heading)
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
        
        # FOV 영역 내부의 점 찾기
        fov_path = Path(corners)
        return {point for point in self.node_rewards.keys() 
                if fov_path.contains_point(point)}
                
    def update_rewards(self, fov_points: Set[Tuple[float, float]]) -> float:
        """
        FOV 내부 점들의 리워드 업데이트 및 총 리워드 계산
        Args:
            fov_points: FOV 내부의 그리드 포인트들
        Returns:
            획득한 총 리워드
        """
        current_time = time.time()
        total_reward = 0
        unvisited_count = 0
        
        for point in fov_points:
            # 시간에 따른 리워드 회복
            time_passed = current_time - self.last_visit_time[point]
            recovered_reward = min(
                self.recovery_threshold * self.initial_reward,
                self.node_rewards[point] + time_passed * self.recovery_rate
            )
            
            # 방문 횟수에 따른 리워드 계산
            if self.visit_count[point] == 0:
                point_reward = recovered_reward
                unvisited_count += 1
            else:
                point_reward = recovered_reward * (1 - self.revisit_penalty)
            
            # 리워드 업데이트 및 누적
            total_reward += point_reward
            self.node_rewards[point] = self.min_reward
            self.visit_count[point] += 1
            self.last_visit_time[point] = current_time
            
        # FOV 내 미방문 점 비율에 따른 보너스
        if fov_points:
            unvisited_ratio = unvisited_count / len(fov_points)
            total_reward *= (1 + self.fov_bonus * unvisited_ratio)
            
        return total_reward
        
    def get_point_reward(self, point: Tuple[float, float]) -> float:
        """특정 점의 현재 리워드 값 반환"""
        if point not in self.node_rewards:
            return 0.0
            
        current_time = time.time()
        time_passed = current_time - self.last_visit_time[point]
        
        # 시간에 따른 리워드 회복
        recovered_reward = min(
            self.recovery_threshold * self.initial_reward,
            self.node_rewards[point] + time_passed * self.recovery_rate
        )
        
        # 방문 횟수에 따른 리워드 조정
        if self.visit_count[point] == 0:
            return recovered_reward
        return recovered_reward * (1 - self.revisit_penalty)
        
    def get_all_rewards(self) -> Dict[Tuple[float, float], float]:
        """모든 점의 현재 리워드 상태 반환"""
        return {point: self.get_point_reward(point) 
                for point in self.node_rewards}
                
    def get_visit_counts(self) -> Dict[Tuple[float, float], int]:
        """모든 점의 방문 횟수 반환"""
        return self.visit_count.copy()
        
    def reset(self):
        """리워드 매니저 초기화"""
        current_time = time.time()
        for point in self.node_rewards:
            self.node_rewards[point] = self.initial_reward
            self.visit_count[point] = 0
            self.last_visit_time[point] = current_time

    def get_current_rewards(self):
        """현재 리워드 값 반환"""
        return self.node_rewards.copy()
