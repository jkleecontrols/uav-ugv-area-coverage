import time
from typing import Dict, Tuple, List

class RewardManager:
    def __init__(self, config):
        self.config = config
        self.node_rewards: Dict[Tuple[float, float], float] = {}  # {(x, y): reward}
        self.last_visit_time: Dict[Tuple[float, float], float] = {}  # {(x, y): timestamp}
        self.base_rewards: Dict[Tuple[float, float], float] = {}  # 기본 리워드 값 저장
        
        # 리워드 관련 설정
        self.min_reward = 0.1  # 최소 리워드
        self.max_reward = 1.0  # 최대 리워드
        self.reward_recovery_rate = 0.1  # 단위 시간당 리워드 회복률
        
    def initialize_rewards(self, nodes: List[Tuple[float, float]]):
        """
        초기 리워드 설정
        Args:
            nodes: 좌표 리스트 [(x1, y1), (x2, y2), ...]
        """
        current_time = time.time()
        for node in nodes:
            self.base_rewards[node] = self.max_reward
            self.node_rewards[node] = self.max_reward
            self.last_visit_time[node] = current_time
            
    def update_reward(self, node: Tuple[float, float]):
        """
        노드 방문 시 리워드 업데이트
        Args:
            node: 방문한 노드 좌표 (x, y)
        """
        current_time = time.time()
        self.node_rewards[node] = self.min_reward
        self.last_visit_time[node] = current_time
        
    def get_current_reward(self, node: Tuple[float, float]) -> float:
        """
        현재 노드의 리워드 값 계산
        Args:
            node: 노드 좌표 (x, y)
        Returns:
            float: 현재 리워드 값
        """
        if node not in self.node_rewards:
            return 0.0
            
        current_time = time.time()
        time_since_last_visit = current_time - self.last_visit_time[node]
        
        # 시간에 따른 리워드 회복 계산
        recovered_reward = self.min_reward + (
            time_since_last_visit * self.reward_recovery_rate
        )
        
        # 기본 리워드 값을 넘지 않도록 제한
        return min(recovered_reward, self.base_rewards[node])
        
    def get_all_rewards(self) -> Dict[Tuple[float, float], float]:
        """
        모든 노드의 현재 리워드 값 반환
        Returns:
            Dict: {(x, y): current_reward, ...}
        """
        current_rewards = {}
        for node in self.node_rewards:
            current_rewards[node] = self.get_current_reward(node)
        return current_rewards
        
    def reset_rewards(self):
        """모든 노드의 리워드를 초기 상태로 리셋"""
        current_time = time.time()
        for node in self.node_rewards:
            self.node_rewards[node] = self.base_rewards[node]
            self.last_visit_time[node] = current_time
