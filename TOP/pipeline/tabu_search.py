import math
from collections import deque

class TabuSearch:
    def __init__(self, env, paths, config):
        self.env = env
        self.paths = paths
        self.config = config
        self.major_axis = config["major_axis"]
        self.tabu_list = deque(maxlen=config.get("tabu_list_size", 10))
        self.best_solution = None
        self.best_score = float('-inf')

    def calculate_distance(self, point1, point2):
        """두 점 사이의 유클리드 거리 계산"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def route_distance(self, path):
        """경로의 총 거리 계산"""
        return sum(self.calculate_distance(path[i], path[i+1]) for i in range(len(path)-1))

    def calculate_score(self, paths):
        """전체 경로의 점수 계산"""
        visited_nodes = set()
        for path in paths:
            visited_nodes.update(path[1:-1])  # 시작점과 끝점 제외
        return len(visited_nodes)

    def is_tabu(self, solution):
        """해당 해결책이 tabu list에 있는지 확인"""
        solution_hash = str(sorted([str(path) for path in solution]))
        return solution_hash in self.tabu_list

    def add_to_tabu(self, solution):
        """해결책을 tabu list에 추가"""
        solution_hash = str(sorted([str(path) for path in solution]))
        self.tabu_list.append(solution_hash)

    def get_neighbors(self, paths):
        """현재 해결책의 이웃해 생성"""
        neighbors = []
        
        # 1. 두 경로 간 노드 교환
        for i in range(len(paths)):
            for j in range(i+1, len(paths)):
                for k in range(1, len(paths[i])-1):
                    for l in range(1, len(paths[j])-1):
                        new_paths = [path[:] for path in paths]
                        new_paths[i][k], new_paths[j][l] = new_paths[j][l], new_paths[i][k]
                        
                        # major_axis 제약 확인
                        if (self.route_distance(new_paths[i]) <= self.major_axis and 
                            self.route_distance(new_paths[j]) <= self.major_axis):
                            neighbors.append(new_paths)
        
        # 2. 노드 이동
        for i in range(len(paths)):
            for j in range(len(paths)):
                if i != j:
                    for k in range(1, len(paths[i])-1):
                        node = paths[i][k]
                        for l in range(1, len(paths[j])):
                            new_paths = [path[:] for path in paths]
                            new_paths[i].pop(k)
                            new_paths[j].insert(l, node)
                            
                            # major_axis 제약 확인
                            if (self.route_distance(new_paths[i]) <= self.major_axis and 
                                self.route_distance(new_paths[j]) <= self.major_axis):
                                neighbors.append(new_paths)
        
        return neighbors

    def run(self, max_iterations=100):
        """Tabu Search 실행"""
        print("\n=== Tabu Search 시작 ===")
        
        current_solution = self.paths
        self.best_solution = current_solution
        self.best_score = self.calculate_score(current_solution)
        
        no_improvement = 0
        iteration = 0
        
        while iteration < max_iterations and no_improvement < 20:
            neighbors = self.get_neighbors(current_solution)
            best_neighbor = None
            best_neighbor_score = float('-inf')
            
            for neighbor in neighbors:
                if not self.is_tabu(neighbor):
                    score = self.calculate_score(neighbor)
                    if score > best_neighbor_score:
                        best_neighbor = neighbor
                        best_neighbor_score = score
            
            if best_neighbor is None:
                print("  ⚠️ 가능한 이웃해가 없습니다.")
                break
            
            current_solution = best_neighbor
            self.add_to_tabu(current_solution)
            
            if best_neighbor_score > self.best_score:
                self.best_solution = best_neighbor
                self.best_score = best_neighbor_score
                no_improvement = 0
                print(f"  🎉 새로운 최고 점수 발견: {self.best_score}")
            else:
                no_improvement += 1
            
            iteration += 1
            if iteration % 10 == 0:
                print(f"  반복 {iteration}: 현재 최고 점수 = {self.best_score}")
        
        print(f"\nTabu Search 완료:")
        print(f"- 총 반복 횟수: {iteration}")
        print(f"- 최종 점수: {self.best_score}")
        
        return {
            "best_solution": self.best_solution,
            "best_score": self.best_score
        } 