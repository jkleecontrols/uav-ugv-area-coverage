from typing import Dict, List, Tuple, Any, Set
import numpy as np
from pipeline.step_zero import StepZero
from pipeline.step_one import StepOne
from pipeline.step_two import StepTwo
from pipeline.step_reinit import StepReinit
from matplotlib.path import Path

# pipeline_manager.py
class PipelineManager:
    def __init__(self, env, uavs, start, end, config, reward_manager):
        """
        Args:
            env: DataLoader 환경
            uavs: UAV 객체 리스트
            start: 시작점 좌표 (x, y)
            end: 종료점 좌표 (x, y)
            config: 설정값
            reward_manager: 리워드 매니저
        """
        self.env = env
        self.uavs = uavs
        self.start = start
        self.end = end
        self.config = config
        self.reward_manager = reward_manager
        self.state = {}
        self.best_solution = None
        self.best_score = float('-inf')
        
        # FOV 관련 파라미터
        self.height = config["uav_height"]
        self.fov_angle = np.radians(config["uav_fov_angle"])
        self.aspect_ratio = config["uav_aspect_ratio"]
        
        # FOV 최적화 파라미터
        self.rotation_step = np.radians(15)  # 회전 각도 단계 (15도)
        self.max_rotation = np.radians(45)   # 최대 회전 각도 (±45도)
        
        # 비행 시간 관련 파라미터
        self.flight_time = config["uav_flight_time"]  # 20분 (1200초)
        self.charging_time = config["uav_charge_time"]  # 14분 (840초)
        
    def _calculate_fov_area(self, uav_pos: Tuple[float, float], heading: float, 
                          rotation: float = 0) -> List[Tuple[float, float]]:
        """UAV의 위치, 방향, 회전각에 따른 FOV 영역의 꼭지점 계산"""
        # FOV 영역 계산
        fov_width = 2 * self.height * np.tan(self.fov_angle/2)
        fov_length = fov_width * self.aspect_ratio
        
        # FOV 꼭지점 계산 (회전각 적용)
        cos_h, sin_h = np.cos(heading), np.sin(heading)
        cos_r, sin_r = np.cos(rotation), np.sin(rotation)
        
        # 회전된 FOV 사각형의 꼭지점 계산
        corners = [
            (uav_pos[0] + fov_length/2 * cos_h * cos_r - fov_width/2 * sin_h * cos_r,
             uav_pos[1] + fov_length/2 * sin_h * cos_r + fov_width/2 * cos_h * cos_r),
            (uav_pos[0] + fov_length/2 * cos_h * cos_r + fov_width/2 * sin_h * cos_r,
             uav_pos[1] + fov_length/2 * sin_h * cos_r - fov_width/2 * cos_h * cos_r),
            (uav_pos[0] - fov_length/2 * cos_h * cos_r + fov_width/2 * sin_h * cos_r,
             uav_pos[1] - fov_length/2 * sin_h * cos_r - fov_width/2 * cos_h * cos_r),
            (uav_pos[0] - fov_length/2 * cos_h * cos_r - fov_width/2 * sin_h * cos_r,
             uav_pos[1] - fov_length/2 * sin_h * cos_r + fov_width/2 * cos_h * cos_r)
        ]
        return corners
        
    def _get_covered_points(self, uav_pos: Tuple[float, float], heading: float, 
                          rotation: float = 0) -> Set[Tuple[float, float]]:
        """FOV 영역 내부의 그리드 포인트들 반환 (회전각 고려)"""
        fov_corners = self._calculate_fov_area(uav_pos, heading, rotation)
        fov_path = Path(fov_corners)
        
        # grid_points를 튜플로 변환하여 집합 연산 가능하게 함
        grid_points_tuples = {tuple(point) for point in self.env.get_grid_points()}
        return {point for point in grid_points_tuples 
                if fov_path.contains_point(point)}
                
    def _find_optimal_rotation(self, uav_pos: Tuple[float, float], heading: float) -> float:
        """최적의 FOV 회전각 찾기"""
        best_rotation = 0
        max_coverage = 0
        
        # 가능한 회전각 범위에서 최적의 각도 탐색
        for rotation in np.arange(-self.max_rotation, self.max_rotation + self.rotation_step, 
                                self.rotation_step):
            covered_points = self._get_covered_points(uav_pos, heading, rotation)
            coverage = len(covered_points)
            
            if coverage > max_coverage:
                max_coverage = coverage
                best_rotation = rotation
                
        return best_rotation
        
    def run(self) -> Dict[str, Any]:
        """
        Team Orienteering 알고리즘 실행 (FOV 회전각 최적화)
        Returns:
            Dict: 최적화된 경로 및 결과
        """
        print("\n=== Phase 1: Initial Path Generation (FOV 회전각 최적화) ===")
        self._run_step_zero()
        
        print("\n=== Phase 2: Two-point Exchange (FOV 회전각 최적화) ===")
        self._run_step_one()
        
        print("\n=== Phase 3: Path Optimization (FOV 회전각 최적화) ===")
        self._run_step_two()
        
        print("\n=== Phase 4: Path Reinitialization (FOV 회전각 최적화) ===")
        self._run_reinit()
        
        return self.get_result()
        
    def _run_step_zero(self):
        """Step Zero 실행"""
        print("\n=== Step Zero 실행 ===")
        
        # Step Zero 초기화
        step = StepZero(
            env=self.env,
            uavs=self.uavs,
            config=self.config,
            reward_manager=self.reward_manager
        )
        
        # Step Zero 실행
        result = step.run()
        
        # 결과 저장
        self.initial_solution = result
        self.available_nodes = result["available_nodes"]
        self.start_point = result["start_point"]
        self.end_point = result["end_point"]
        self.state["step_zero"] = result  # state 딕셔너리에 결과 저장
        
        print(f"✅ Step Zero 완료: {len(self.available_nodes)}개의 가용가드 노드 식별")
        
    def _run_step_one(self):
        """Two-point exchange를 통한 경로 개선 (FOV 회전각 최적화)"""
        step = StepOne(
            env=self.env,
            initial_solution=self.state["step_zero"],
            config=self.config,
            reward_manager=self.reward_manager
        )
        self.state["step_one"] = step.run()
        self._update_best_solution(self.state["step_one"])
        
    def _run_step_two(self):
        """추가 경로 최적화 (FOV 회전각 최적화)"""
        step = StepTwo(
            env=self.env,
            improved_solution=self.state["step_one"],
            config=self.config,
            reward_manager=self.reward_manager
        )
        self.state["step_two"] = step.run()
        self._update_best_solution(self.state["step_two"])
        
    def _run_reinit(self):
        """경로 재초기화 단계 (FOV 회전각 최적화)"""
        step = StepReinit(
            env=self.env,
            step_two_result=self.state["step_two"],
            config=self.config,
            reward_manager=self.reward_manager
        )
        self.state["reinit"] = step.run()
        self._update_best_solution(self.state["reinit"])
        
    def _update_best_solution(self, current_solution: Dict[str, Any]):
        """현재 해가 더 좋으면 최적해 업데이트 (FOV 회전각 최적화)"""
        current_score = self._calculate_solution_score(current_solution)
        if current_score > self.best_score:
            self.best_score = current_score
            self.best_solution = current_solution.copy()
            print(f"🎯 New best solution found! Score: {current_score:.2f}")
            
    def _calculate_solution_score(self, solution: Dict[str, Any]) -> float:
        """해의 품질 점수 계산"""
        total_reward = 0
        covered_points = set()  # 이미 커버된 포인트 추적
        
        for uav_id, path in solution["paths"].items():
            # 경로상의 각 위치에서의 리워드 계산
            for i in range(len(path) - 1):
                pos = path[i]
                next_pos = path[i + 1]
                
                # 진행 방향 계산
                heading = np.arctan2(next_pos[1] - pos[1], next_pos[0] - pos[0])
                
                # FOV 영역 내부의 점들 찾기
                fov_points = self._get_covered_points(pos, heading)
                
                # 아직 커버되지 않은 점들만 고려
                new_points = fov_points - covered_points
                if new_points:
                    reward = self.reward_manager.update_rewards(new_points)
                    total_reward += reward
                    covered_points.update(new_points)
                    
        # 정규화된 최종 점수 계산
        path_length = sum(len(path) for path in solution["paths"].values())
        if path_length == 0:
            return 0
            
        normalized_score = (total_reward / path_length) * (len(covered_points) / len(self.env.get_grid_points()))
        return normalized_score
        
    def get_result(self) -> Dict[str, Any]:
        """최종 결과 반환"""
        return {
            "best_solution": self.best_solution,
            "best_score": self.best_score,
            "execution_trace": {
                "step_zero": self.state.get("step_zero", {}),
                "step_one": self.state.get("step_one", {}),
                "step_two": self.state.get("step_two", {}),
                "reinit": self.state.get("reinit", {})
            }
        }