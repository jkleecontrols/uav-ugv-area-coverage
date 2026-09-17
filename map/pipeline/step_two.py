from typing import Dict, List, Tuple, Any, Set
import numpy as np
from matplotlib.path import Path
from .tabu_search import TabuSearch
from zmodelsvehicle import UAV
from zrewardmanager import RewardManager

class StepTwo:
    def __init__(self, env, improved_solution: Dict, config: Dict, reward_manager: RewardManager):
        self.env = env
        self.paths = improved_solution["paths"]
        self.start = improved_solution["start_point"]
        self.end = improved_solution["end_point"]
        self.config = config
        self.reward_manager = reward_manager
        
        # FOV 관련 파라미터
        self.height = config["uav_height"]
        self.fov_angle = np.radians(config["uav_fov_angle"])
        self.aspect_ratio = config["uav_aspect_ratio"]
        
        # 최적화 파라미터
        self.max_rounds = 10  # 최대 라운드 수
        self.no_improvement_limit = 3  # 개선 없이 허용되는 연속 라운드 수
        self.neighborhood_radius = 5  # m
        self.max_subsequence_length = 5  # two-opt에서 고려할 최대 subsequence 길이
        
        # 그리드 포인트
        self.grid_points = env.get_grid_points()
        
    def _calculate_fov_area(self, uav_pos: Tuple[float, float], heading: float) -> List[Tuple[float, float]]:
        """FOV 영역의 꼭지점 계산"""
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
        return corners
        
    def _get_covered_points(self, uav_pos: Tuple[float, float], heading: float) -> Set[Tuple[float, float]]:
        """FOV 영역 내부의 그리드 포인트들 반환"""
        fov_corners = self._calculate_fov_area(uav_pos, heading)
        fov_path = Path(fov_corners)
        
        covered_points = set()
        for point in self.grid_points:
            if fov_path.contains_point(point):
                covered_points.add(tuple(point))
        return covered_points
        
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
                    reward = self.reward_manager.update_rewards(new_points)
                    total_reward += reward
                    covered_points.update(new_points)
                    
        return total_reward
        
    def _try_two_point_exchange(self, paths: Dict[str, np.ndarray]) -> bool:
        """Two-point exchange 시도"""
        best_reward = self._calculate_team_reward(paths)
        improved = False
        
        # UAV 간 교환 시도
        uav_ids = list(paths.keys())
        for i in range(len(uav_ids)):
            for j in range(i + 1, len(uav_ids)):
                path1 = paths[uav_ids[i]]
                path2 = paths[uav_ids[j]]
                
                for idx1 in range(1, len(path1) - 1):
                    for idx2 in range(1, len(path2) - 1):
                        # FOV 겹침 확인
                        pos1, pos2 = tuple(path1[idx1]), tuple(path2[idx2])
                        fov1 = self._get_covered_points(pos1, np.arctan2(path1[idx1+1][1]-pos1[1], path1[idx1+1][0]-pos1[0]))
                        fov2 = self._get_covered_points(pos2, np.arctan2(path2[idx2+1][1]-pos2[1], path2[idx2+1][0]-pos2[0]))
                        
                        if len(fov1 & fov2) > 0:  # FOV가 겹치는 경우
                            # 교환 시도
                            new_paths = paths.copy()
                            new_paths[uav_ids[i]] = np.vstack([path1[:idx1], path2[idx2:idx2+1], path1[idx1+1:]])
                            new_paths[uav_ids[j]] = np.vstack([path2[:idx2], path1[idx1:idx1+1], path2[idx2+1:]])
                            
                            new_reward = self._calculate_team_reward(new_paths)
                            if new_reward > best_reward:
                                paths.update(new_paths)
                                best_reward = new_reward
                                improved = True
                                
        return improved
        
    def _try_one_point_movement(self, paths: Dict[str, np.ndarray]) -> bool:
        """One-point movement 시도"""
        best_reward = self._calculate_team_reward(paths)
        improved = False
        
        for uav_id, path in paths.items():
            for i in range(1, len(path) - 1):
                current_pos = tuple(path[i])
                
                # 이웃 그리드 포인트 찾기
                neighbors = [pt for pt in self.grid_points 
                           if np.sqrt((pt[0]-current_pos[0])**2 + (pt[1]-current_pos[1])**2) < self.neighborhood_radius]
                
                # heading angle 후보
                current_heading = np.arctan2(path[i+1][1]-current_pos[1], path[i+1][0]-current_pos[0])
                angle_candidates = [current_heading - np.radians(5), 
                                  current_heading, 
                                  current_heading + np.radians(5)]
                
                for new_pos in neighbors:
                    for new_heading in angle_candidates:
                        # 새로운 경로 생성
                        new_path = path.copy()
                        new_path[i] = new_pos
                        
                        # heading에 따른 다음 위치 조정
                        if i < len(path) - 1:
                            new_path[i+1] = (new_pos[0] + np.cos(new_heading) * self.neighborhood_radius,
                                           new_pos[1] + np.sin(new_heading) * self.neighborhood_radius)
                        
                        new_paths = paths.copy()
                        new_paths[uav_id] = new_path
                        
                        new_reward = self._calculate_team_reward(new_paths)
                        if new_reward > best_reward:
                            paths.update(new_paths)
                            best_reward = new_reward
                            improved = True
                            
        return improved
        
    def _try_two_opt_optimization(self, paths: Dict[str, np.ndarray]) -> bool:
        """Two-opt optimization 시도"""
        best_reward = self._calculate_team_reward(paths)
        improved = False
        
        for uav_id, path in paths.items():
            for i in range(1, len(path) - 2):
                for j in range(i + 1, min(i + self.max_subsequence_length, len(path) - 1)):
                    # subsequence 반전
                    new_path = np.vstack([path[:i], path[i:j+1][::-1], path[j+1:]])
                    
                    new_paths = paths.copy()
                    new_paths[uav_id] = new_path
                    
                    new_reward = self._calculate_team_reward(new_paths)
                    if new_reward > best_reward:
                        paths.update(new_paths)
                        best_reward = new_reward
                        improved = True
                        
        return improved
        
    def run(self) -> Dict[str, Any]:
        """경로 개선 실행"""
        print("\n=== Step Two: Iterative Path Improvement ===")
        
        best_paths = self.paths.copy()
        best_reward = self._calculate_team_reward(best_paths)
        no_improvement_count = 0
        
        for round in range(self.max_rounds):
            print(f"\n  라운드 {round + 1}/{self.max_rounds}")
            updated = False
            
            # 1. Two-point exchange
            print("  - Two-point exchange 시도...")
            if self._try_two_point_exchange(self.paths):
                updated = True
                print("    * 개선 발견!")
            
            # 2. One-point movement
            print("  - One-point movement 시도...")
            if self._try_one_point_movement(self.paths):
                updated = True
                print("    * 개선 발견!")
            
            # 3. Two-opt optimization
            print("  - Two-opt optimization 시도...")
            if self._try_two_opt_optimization(self.paths):
                updated = True
                print("    * 개선 발견!")
            
            # 현재 reward 계산
            current_reward = self._calculate_team_reward(self.paths)
            print(f"  - 현재 reward: {current_reward:.2f}")
            
            # 개선 여부 확인
            if not updated:
                no_improvement_count += 1
                if no_improvement_count >= self.no_improvement_limit:
                    print("  - 연속된 개선 없음. 최적화 종료.")
                    break
            else:
                no_improvement_count = 0
                if current_reward > best_reward:
                    best_paths = self.paths.copy()
                    best_reward = current_reward
                    print("  - 새로운 최선의 경로 발견!")
        
        return {
            "paths": best_paths,
            "start_point": self.start,
            "end_point": self.end,
            "final_reward": best_reward
        }
