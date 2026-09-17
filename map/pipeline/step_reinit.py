from typing import Dict, List, Tuple, Any, Set
import numpy as np
from scipy.spatial import distance
from matplotlib.path import Path
from zmodelsvehicle import UAV
from zrewardmanager import RewardManager
from .step_zero import StepZero
from .step_one import StepOne
from .step_two import StepTwo

class StepReinit:
    def __init__(self, env, step_two_result: Dict, config: Dict, reward_manager: RewardManager):
        self.env = env
        self.paths = step_two_result["paths"]
        self.start = step_two_result["start_point"]
        self.end = step_two_result["end_point"]
        self.config = config
        self.reward_manager = reward_manager
        
        # UAVs 초기화
        self.uavs = [UAV(vehicle_id=int(uav_id.split('_')[1]), 
                        start_node=self.start,
                        schedule={}) 
                    for uav_id in self.paths.keys()]
        
        # FOV 관련 파라미터
        self.height = config["uav_height"]
        self.fov_angle = np.radians(config["uav_fov_angle"])
        self.aspect_ratio = config["uav_aspect_ratio"]
        
        # Reinit 파라미터
        self.max_reinit_iterations = 5  # 최대 Reinit 횟수
        self.improvement_threshold = 0.01  # 1% 개선 임계값
        self.stall_iterations = 3  # Reinit 트리거를 위한 연속 개선 실패 횟수
        self.reward_recovery_interval = 3  # reward 회복 주기
        
        # 상태 변수
        self.best_reward = step_two_result.get("final_reward", 0)
        self.reinit_iteration = 0
        self.stall_count = 0
        self.visit_counts = {}  # 그리드 포인트별 방문 횟수 추적
        
    def _calculate_team_reward(self, paths: Dict[str, np.ndarray]) -> float:
        """전체 UAV 팀의 총 reward 계산"""
        total_reward = 0
        covered_points = set()
        
        for uav_id, path in paths.items():
            for i in range(len(path) - 1):
                pos = path[i]
                next_pos = path[i + 1]
                heading = np.arctan2(next_pos[1] - pos[1], next_pos[0] - pos[0])
                
                fov_points = self._get_covered_points(tuple(pos), heading)
                new_points = fov_points - covered_points
                
                if new_points:
                    # 방문 횟수 업데이트
                    for point in new_points:
                        self.visit_counts[point] = self.visit_counts.get(point, 0) + 1
                    
                    # reward 계산 (방문 횟수에 따른 패널티 적용)
                    reward = self._calculate_penalized_reward(new_points)
                    total_reward += reward
                    covered_points.update(new_points)
                    
        return total_reward
        
    def _calculate_penalized_reward(self, points: Set[Tuple[float, float]]) -> float:
        """방문 횟수에 따른 패널티를 적용한 reward 계산"""
        total_reward = 0
        
        for point in points:
            visit_count = self.visit_counts.get(point, 0)
            base_reward = self.reward_manager.get_point_reward(point)
            
            # 방문 횟수에 따른 reward multiplier (지수적 감소)
            multiplier = 1.0 / (2 ** (visit_count - 1)) if visit_count > 0 else 1.0
                
            total_reward += base_reward * multiplier
            
        return total_reward
        
    def _recover_rewards(self):
        """Reward multiplier 회복"""
        if self.reinit_iteration % self.reward_recovery_interval == 0:
            for point in self.visit_counts:
                if self.visit_counts[point] > 1:
                    # 방문 횟수 1 감소 (최소 1)
                    self.visit_counts[point] = max(1, self.visit_counts[point] - 1)
                    
    def _should_reinitialize(self, current_reward: float) -> bool:
        """Reinitialization 필요 여부 판단"""
        # 개선도 계산
        improvement = (current_reward - self.best_reward) / self.best_reward if self.best_reward > 0 else 0
        
        if improvement > self.improvement_threshold:
            self.stall_count = 0
            self.best_reward = current_reward
            return False
        else:
            self.stall_count += 1
            return self.stall_count >= self.stall_iterations
            
    def _get_covered_points(self, uav_pos: Tuple[float, float], heading: float) -> Set[Tuple[float, float]]:
        """FOV 영역 내부의 그리드 포인트들 반환"""
        fov_width = 2 * self.height * np.tan(self.fov_angle/2)
        fov_length = fov_width * self.aspect_ratio
        
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
        
        fov_path = Path(corners)
        covered_points = set()
        for point in self.env.get_grid_points():
            if fov_path.contains_point(point):
                covered_points.add(tuple(point))
        return covered_points
        
    def run(self) -> Dict[str, Any]:
        """경로 재초기화 실행"""
        print("\n=== Step Reinit: Reward Reinitialization and Local Optima Escape ===")
        
        # Step Zero 재실행
        step_zero = StepZero(
            env=self.env,
            uavs=self.uavs,
            config=self.config,
            reward_manager=self.reward_manager
        )
        zero_result = step_zero.run()
        
        # Step One 재실행
        step_one = StepOne(
            env=self.env,
            initial_solution=zero_result,
            config=self.config,
            reward_manager=self.reward_manager
        )
        one_result = step_one.run()
        
        # Step Two 재실행
        step_two = StepTwo(
            env=self.env,
            improved_solution=one_result,
            config=self.config,
            reward_manager=self.reward_manager
        )
        two_result = step_two.run()
        
        return {
            "paths": two_result["paths"],
            "available_nodes": zero_result["available_nodes"],
            "start_point": zero_result["start_point"],
            "end_point": zero_result["end_point"]
        }
