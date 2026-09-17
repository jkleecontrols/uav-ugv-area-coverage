import numpy as np
import math

class StepTwo:
    def __init__(self, env, step_one_result, start, end, config):
        self.env = env
        self.step_one_result = step_one_result
        self.start = start
        self.end = end
        self.config = config
        self.major_axis = config["major_axis"]
        self.optimized_paths = {}

    def calculate_distance(self, point1, point2):
        """두 점 사이의 유클리드 거리 계산"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def route_distance(self, path):
        """경로의 총 거리 계산"""
        total_distance = 0
        for i in range(len(path)-1):
            total_distance += self.calculate_distance(path[i], path[i+1])
        return total_distance

    def validate_path(self, path):
        """path가 타원 제약을 만족하는지 검사"""
        path_length = self.route_distance(path)
        if path_length > self.major_axis:
            print(f"  ⚠️ 경고: path 길이({path_length:.2f}m)가 major_axis({self.major_axis}m)를 초과했습니다!")
            return False
        return True

    def one_point_insertion(self, paths):
        """가장 효율적인 위치에 노드 삽입"""
        max_iterations = 100
        iteration = 0
        improved = True
        
        while improved and iteration < max_iterations:
            improved = False
            iteration += 1
            print(f"  One-point insertion 반복 {iteration}회")
            
            for path_idx, path in enumerate(paths):
                nodes_to_move = []
                for i in range(1, len(path)-1):
                    nodes_to_move.append((i, path[i]))
                
                for original_idx, node in nodes_to_move:
                    if node in path:
                        path.remove(node)
                        
                        best_path_idx = path_idx
                        best_position = 1
                        min_distance_increase = float('inf')
                        
                        for other_path_idx, other_path in enumerate(paths):
                            for position in range(1, len(other_path)):
                                original_distance = self.route_distance(other_path)
                                new_path = other_path[:position] + [node] + other_path[position:]
                                new_distance = self.route_distance(new_path)
                                distance_increase = new_distance - original_distance
                                
                                if distance_increase < min_distance_increase:
                                    min_distance_increase = distance_increase
                                    best_path_idx = other_path_idx
                                    best_position = position
                        
                        # 타원 제약 확인
                        new_path = paths[best_path_idx][:best_position] + [node] + paths[best_path_idx][best_position:]
                        if self.validate_path(new_path):
                            paths[best_path_idx].insert(best_position, node)
                            if best_path_idx != path_idx:
                                improved = True
                                print(f"    노드 이동 성공: 경로 {path_idx+1} -> 경로 {best_path_idx+1}")
                                print(f"    경로 {best_path_idx+1} 길이: {self.route_distance(paths[best_path_idx]):.2f}m")
                        else:
                            # 제약을 만족하지 않으면 원래 경로에 다시 삽입
                            path.insert(original_idx, node)
                            print(f"    ⚠️ 노드 이동 실패: 타원 제약 위반")
            
            if iteration >= max_iterations:
                print("  ⚠️ 최대 반복 횟수 도달. One-point insertion 종료")
        
        return paths

    def two_point_exchange(self, paths):
        """두 경로 간의 노드 교환"""
        max_iterations = 100
        iteration = 0
        improved = True
        
        while improved and iteration < max_iterations:
            improved = False
            iteration += 1
            print(f"  Two-point exchange 반복 {iteration}회")
            
            for i in range(len(paths)):
                for j in range(i+1, len(paths)):
                    for k in range(1, len(paths[i])-1):
                        for l in range(1, len(paths[j])-1):
                            # 노드 교환
                            temp = paths[i][k]
                            paths[i][k] = paths[j][l]
                            paths[j][l] = temp
                            
                            # 타원 제약 확인
                            if self.validate_path(paths[i]) and self.validate_path(paths[j]):
                                improved = True
                                print(f"    경로 {i+1}과 {j+1} 간 노드 교환 성공")
                                print(f"    경로 {i+1} 길이: {self.route_distance(paths[i]):.2f}m")
                                print(f"    경로 {j+1} 길이: {self.route_distance(paths[j]):.2f}m")
                            else:
                                # 제약을 만족하지 않으면 원래대로
                                temp = paths[i][k]
                                paths[i][k] = paths[j][l]
                                paths[j][l] = temp
                                print(f"    ⚠️ 경로 {i+1}과 {j+1} 간 노드 교환 실패: 타원 제약 위반")
            
            if iteration >= max_iterations:
                print("  ⚠️ 최대 반복 횟수 도달. Two-point exchange 종료")
        
        return paths

    def two_opt_swap(self, path, i, j):
        """2-opt 스왑 수행"""
        new_path = path[:i] + path[i:j+1][::-1] + path[j+1:]
        return new_path

    def two_opt_optimization(self, path):
        """2-opt 알고리즘으로 경로 최적화"""
        best_path = path
        best_distance = self.route_distance(path)
        improved = True
        iteration = 0
        max_iterations = 100  # 최대 반복 횟수 제한
        
        while improved and iteration < max_iterations:
            improved = False
            for i in range(1, len(path)-2):
                for j in range(i+1, len(path)-1):
                    new_path = self.two_opt_swap(best_path, i, j)
                    new_distance = self.route_distance(new_path)
                    
                    if new_distance < best_distance:
                        best_path = new_path
                        best_distance = new_distance
                        improved = True
                        break
                if improved:
                    break
            iteration += 1
            
            if iteration >= max_iterations:
                print("  ⚠️ 2-opt: 최대 반복 횟수 도달")
        
        return best_path

    def calculate_team_score(self, paths):
        """팀 점수 계산: 방문한 모든 노드의 수"""
        visited_nodes = set()
        for path in paths:
            visited_nodes.update(path[1:-1])  # 시작점과 끝점 제외
        return len(visited_nodes)

    def rearrange(self, paths):
        """경로 재배치: 짧은 경로의 노드를 긴 경로로 이동"""
        max_iterations = 100
        iteration = 0
        improved = True
        
        while improved and iteration < max_iterations:
            improved = False
            iteration += 1
            print(f"  Rearrange 반복 {iteration}회")
            
            sorted_paths = sorted(paths, key=lambda x: self.route_distance(x))
            
            for i in range(len(sorted_paths)-1):
                short_path = sorted_paths[i]
                long_path = sorted_paths[-1]
                
                for node in short_path[1:-1]:
                    short_path.remove(node)
                    
                    best_position = 1
                    min_distance_increase = float('inf')
                    
                    for position in range(1, len(long_path)):
                        original_distance = self.route_distance(long_path)
                        new_path = long_path[:position] + [node] + long_path[position:]
                        new_distance = self.route_distance(new_path)
                        distance_increase = new_distance - original_distance
                        
                        if distance_increase < min_distance_increase:
                            min_distance_increase = distance_increase
                            best_position = position
                    
                    # 타원 제약 확인
                    new_path = long_path[:best_position] + [node] + long_path[best_position:]
                    if self.validate_path(new_path):
                        long_path.insert(best_position, node)
                        improved = True
                        print(f"    노드 이동 성공: 짧은 경로 -> 긴 경로")
                        print(f"    긴 경로 길이: {self.route_distance(long_path):.2f}m")
                    else:
                        short_path.insert(1, node)
                        # print(f"    ⚠️ 노드 이동 실패: 타원 제약 위반")
            
            if iteration >= max_iterations:
                print("  ⚠️ 최대 반복 횟수 도달. Rearrange 종료")
        
        return paths

    def print_path_info(self, paths, title="경로 정보"):
        """경로 정보 출력"""
        print(f"\n=== {title} ===")
        total_nodes = 0
        for i, path in enumerate(paths):
            path_length = self.route_distance(path)
            nodes_count = len(path) - 2  # 시작점과 끝점 제외
            total_nodes += nodes_count
            print(f"Path {i+1}:")
            print(f"  - 노드 수: {nodes_count}개")
            print(f"  - 경로 길이: {path_length:.2f}m")
            print(f"  - 타원 제약: {'만족' if path_length <= self.major_axis else '위반'}")
            print(f"  - 방문 노드: {path[1:-1]}")
        print(f"총 방문 노드 수: {total_nodes}개")
        return total_nodes

    def remove_empty_paths(self, paths):
        """노드가 없는 path(시작점과 끝점만 있는 path)를 제거"""
        filtered_paths = [path for path in paths if len(path) > 2]
        removed_count = len(paths) - len(filtered_paths)
        if removed_count > 0:
            print(f"  ⚠️ {removed_count}개의 빈 path 제거됨")
        return filtered_paths

    def get_team_score(self):
        """현재 팀 점수 반환"""
        total_score = 0
        for uav_id, paths in self.optimized_paths.items():
            for path in paths:
                total_score += len(path) - 2  # 시작점과 끝점 제외
        return total_score

    def run(self):
        """메인 실행 함수"""
        for uav_id, initial_paths in self.step_one_result["paths"].items():
            print(f"\n{'='*50}")
            print(f"UAV {uav_id} - Improvement phase 시작")
            print(f"{'='*50}")
            
            K = self.config["step_two_max_outer_loop"]
            I = self.config["step_two_max_inner_loop"]
            no_improvement_limit = self.config["step_two_no_improvement_limit"]
            
            best_team_score = float('-inf')
            no_improvement_count = 0
            current_paths = initial_paths.copy()
            
            # 초기 상태 출력
            print("\n초기 경로 상태:")
            self.print_path_info(current_paths)
            
            for k in range(1, K+1):
                print(f"\n{'='*30}")
                print(f"Outer loop {k}/{K}")
                print(f"{'='*30}")
                
                for i in range(1, I+1):
                    print(f"\nInner loop {i}/{I}")
                    
                    # 1. Two-point exchange
                    print("\n1. Two-point exchange 실행")
                    current_paths = self.two_point_exchange(current_paths)
                    current_paths = self.remove_empty_paths(current_paths)
                    self.print_path_info(current_paths, "Two-point exchange 후")
                    
                    # 2. One-point movement
                    print("\n2. One-point movement 실행")
                    current_paths = self.one_point_insertion(current_paths)
                    current_paths = self.remove_empty_paths(current_paths)
                    self.print_path_info(current_paths, "One-point movement 후")
                    
                    # 3. Route optimization (2-opt)
                    print("\n3. 2-opt optimization 실행")
                    current_paths = [self.two_opt_optimization(path) for path in current_paths]
                    current_paths = self.remove_empty_paths(current_paths)
                    self.print_path_info(current_paths, "2-opt optimization 후")
                    
                    # 4. Rearrange
                    print("\n4. Rearrange 실행")
                    current_paths = self.rearrange(current_paths)
                    current_paths = self.remove_empty_paths(current_paths)
                    self.print_path_info(current_paths, "Rearrange 후")
                    
                    # 현재 team score 계산
                    current_team_score = self.calculate_team_score(current_paths)
                    print(f"\n현재 team score: {current_team_score}")
                    print(f"이전 최고 team score: {best_team_score}")
                    
                    if current_team_score > best_team_score:
                        best_team_score = current_team_score
                        no_improvement_count = 0
                        print("🎉 개선 발견! 새로운 최고 점수 달성")
                    else:
                        no_improvement_count += 1
                        print(f"😐 개선 없음 (연속 {no_improvement_count}회)")
                    
                    if no_improvement_count >= no_improvement_limit:
                        print(f"\n⚠️ {no_improvement_limit}회 연속 개선 없음. Inner loop 종료")
                        break
                
                if no_improvement_count >= no_improvement_limit:
                    print(f"\n⚠️ {no_improvement_limit}회 연속 개선 없음. Outer loop 종료")
                    break
            
            self.optimized_paths[uav_id] = current_paths
            print(f"\n{'='*50}")
            print(f"UAV {uav_id} 최종 결과:")
            print(f"{'='*50}")
            self.print_path_info(current_paths, "최종 경로")
            print(f"최종 team score: {best_team_score}")
        
        return {
            "optimized_paths": self.optimized_paths
        }
