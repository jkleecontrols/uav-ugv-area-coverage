import math
from .tabu_search import TabuSearch

class StepReinit:
    def __init__(self, env, step_two_result, config):
        self.env = env
        self.step_two_result = step_two_result
        self.config = config
        self.major_axis = config["major_axis"]
        self.p = 2.5  # deviation parameter
        self.deviation = config.get("reinit_deviation_ratio", 0.025)  # config에서 값 가져오기
        self.record = 0  # best score achieved
        self.paths_top = []  # top M scoring paths
        self.paths_ntop = []  # alternative paths
        self.updated_paths = {}

    def calculate_distance(self, point1, point2):
        """두 점 사이의 유클리드 거리 계산"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def route_distance(self, path):
        """경로의 총 거리 계산"""
        total_distance = 0
        for i in range(len(path)-1):
            total_distance += self.calculate_distance(path[i], path[i+1])
        return total_distance

    def calculate_score(self, path):
        """경로의 점수 계산 (방문한 노드 수)"""
        return len(path) - 2  # 시작점과 끝점 제외

    def calculate_insertion_cost(self, path, point_idx):
        """특정 노드를 제거했을 때의 삽입 비용 계산"""
        if point_idx <= 0 or point_idx >= len(path)-1:  # 시작점이나 끝점은 제거하지 않음
            return float('inf')
        
        original_distance = self.route_distance(path)
        new_path = path[:point_idx] + path[point_idx+1:]
        new_distance = self.route_distance(new_path)
        insertion_cost = original_distance - new_distance
        
        # 삽입 비용이 0이면 매우 작은 값 반환 (0으로 나누기 방지)
        return max(insertion_cost, 1e-10)

    def find_cheapest_insertion(self, point, paths):
        """주어진 노드를 가장 효율적인 위치에 삽입"""
        best_path_idx = -1
        best_position = -1
        min_distance_increase = float('inf')
        
        for path_idx, path in enumerate(paths):
            for position in range(1, len(path)):
                # 새로운 경로 생성
                new_path = path[:position] + [point] + path[position:]
                new_distance = self.route_distance(new_path)
                
                # major_axis 제약 확인
                if new_distance <= self.major_axis:
                    distance_increase = new_distance - self.route_distance(path)
                    if distance_increase < min_distance_increase:
                        min_distance_increase = distance_increase
                        best_path_idx = path_idx
                        best_position = position
        
        return best_path_idx, best_position

    def select_top_paths(self, paths, M):
        """상위 M개의 경로 선택"""
        scored_paths = [(path, self.calculate_score(path)) for path in paths]
        scored_paths.sort(key=lambda x: x[1], reverse=True)
        return [path for path, _ in scored_paths[:M]]

    def reinitialize(self):
        """Reinitialization II 과정 실행"""
        print("\n=== Reinitialization II 시작 ===")
        
        # 1. 현재 경로들을 top과 ntop으로 분리
        all_paths = []
        for uav_id, paths in self.step_two_result["optimized_paths"].items():
            all_paths.extend(paths)
        
        M = len(self.step_two_result["optimized_paths"])  # UAV 수만큼의 top 경로
        self.paths_top = self.select_top_paths(all_paths, M)
        self.paths_ntop = [path for path in all_paths if path not in self.paths_top]
        
        # 2. record 점수 업데이트
        self.record = sum(self.calculate_score(path) for path in self.paths_top)
        self.deviation = self.config["reinit_deviation_ratio"] * self.record
        
        print(f"초기 record 점수: {self.record}")
        print(f"Deviation: {self.deviation}")
        
        # 3. 각 top 경로의 노드에 대해 score-to-insertion-cost ratio 계산
        points_to_remove = []
        for path in self.paths_top:
            for i in range(1, len(path)-1):  # 시작점과 끝점 제외
                insertion_cost = self.calculate_insertion_cost(path, i)
                if insertion_cost != float('inf'):
                    score = self.calculate_score([path[i]])
                    ratio = score / insertion_cost
                    points_to_remove.append((path[i], ratio, path))
                    print(f"  노드 {path[i]}: 점수 = {score}, 삽입 비용 = {insertion_cost:.6f}, 비율 = {ratio:.6f}")
        
        if not points_to_remove:
            print("  ⚠️ 제거할 수 있는 노드가 없습니다.")
            return {
                "reinitialized_paths": self.step_two_result["optimized_paths"],
                "record": self.record
            }
        
        # 4. 가장 작은 ratio를 가진 k개의 점 선택
        k = max(1, int(len(points_to_remove) * (self.p / 100)))  # 최소 1개는 선택
        points_to_remove.sort(key=lambda x: x[1])
        selected_points = points_to_remove[:k]
        
        print(f"제거할 점 수: {k}")
        
        # 5. 선택된 점들을 top 경로에서 제거
        for point, ratio, path in selected_points:
            if point in path:
                path.remove(point)
                print(f"  노드 {point} 제거됨 (비율: {ratio:.6f})")
        
        # 6. 제거된 점들을 ntop 경로에 재삽입
        for point, _, _ in selected_points:
            best_path_idx, best_position = self.find_cheapest_insertion(point, self.paths_ntop)
            if best_path_idx != -1:
                self.paths_ntop[best_path_idx].insert(best_position, point)
                print(f"  노드 {point}가 ntop 경로 {best_path_idx}에 재삽입됨")
        
        # 7. 새로운 top 경로 재구성
        all_paths = self.paths_top + self.paths_ntop
        
        # 8. Tabu Search 적용
        print("\n=== Tabu Search 적용 ===")
        tabu_search = TabuSearch(self.env, all_paths, self.config)
        tabu_result = tabu_search.run(self.config["tabu_max_iterations"])
        
        # 9. Tabu Search 결과로 경로 업데이트
        all_paths = tabu_result["best_solution"]
        self.paths_top = self.select_top_paths(all_paths, M)
        self.paths_ntop = [path for path in all_paths if path not in self.paths_top]
        
        # 10. 결과 업데이트
        new_record = tabu_result["best_score"]
        print(f"새로운 record 점수: {new_record}")
        
        # UAV별로 경로 할당
        for uav_id in self.step_two_result["optimized_paths"].keys():
            self.updated_paths[uav_id] = []
            for path in self.paths_top:
                if path not in self.updated_paths[uav_id]:
                    self.updated_paths[uav_id].append(path)
                    break
        
        return {
            "reinitialized_paths": self.updated_paths,
            "record": new_record
        }

    def run(self):
        """메인 실행 함수"""
        return self.reinitialize()
