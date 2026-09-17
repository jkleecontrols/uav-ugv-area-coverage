import math

class StepReattachment:
    def __init__(self, env, step_reinit_result, config):
        self.env = env
        self.step_reinit_result = step_reinit_result
        self.config = config
        self.major_axis = config["major_axis"]
        self.max_detour_ratio = config.get("max_detour_ratio", 0.2)  # 최대 우회 비율
        self.distance_threshold = config.get("reattachment_distance_threshold", 50)  # 노드 간 최대 거리
        self.reattached_paths = {}

    def calculate_distance(self, point1, point2):
        """두 점 사이의 유클리드 거리 계산"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def route_distance(self, path):
        """경로의 총 거리 계산"""
        return sum(self.calculate_distance(path[i], path[i+1]) for i in range(len(path)-1))

    def find_nearby_nodes(self, point, all_nodes, visited_nodes):
        """주어진 점 근처의 방문하지 않은 노드 찾기"""
        nearby = []
        for node in all_nodes:
            if node not in visited_nodes:
                distance = self.calculate_distance(point, node)
                if distance <= self.distance_threshold:
                    nearby.append((node, distance))
        return sorted(nearby, key=lambda x: x[1])  # 거리순 정렬

    def can_add_node(self, path, new_node, insert_idx):
        """노드 추가 가능 여부 확인"""
        new_path = path[:insert_idx] + [new_node] + path[insert_idx:]
        new_distance = self.route_distance(new_path)
        original_distance = self.route_distance(path)
        
        # 1. major_axis 제약 확인
        if new_distance > self.major_axis:
            return False
        
        # 2. 우회 비율 확인
        detour_ratio = (new_distance - original_distance) / original_distance
        if detour_ratio > self.max_detour_ratio:
            return False
        
        return True

    def find_best_insertion_point(self, path, node):
        """최적의 노드 삽입 위치 찾기"""
        best_position = -1
        min_distance_increase = float('inf')
        
        for i in range(1, len(path)):
            if self.can_add_node(path, node, i):
                new_path = path[:i] + [node] + path[i:]
                distance_increase = self.route_distance(new_path) - self.route_distance(path)
                
                if distance_increase < min_distance_increase:
                    min_distance_increase = distance_increase
                    best_position = i
        
        return best_position, min_distance_increase

    def reattach_nodes(self, path):
        """경로에 누락된 노드 재연결"""
        print(f"\n경로 재연결 시작 (현재 노드 수: {len(path)})")
        
        all_nodes = set(self.env.get_all_nodes())
        visited_nodes = set(path)
        modified = False
        original_path = path.copy()  # 원본 경로 저장
        
        for i in range(len(path)-1):
            current_node = path[i]
            next_node = path[i+1]
            segment_distance = self.calculate_distance(current_node, next_node)
            
            # 두 노드 사이 거리가 너무 멀면 중간 노드 검사
            if segment_distance > self.distance_threshold:
                print(f"  긴 세그먼트 발견: {segment_distance:.2f}m")
                nearby_nodes = self.find_nearby_nodes(current_node, all_nodes, visited_nodes)
                
                for node, distance in nearby_nodes:
                    # 다음 노드까지의 거리도 확인
                    if self.calculate_distance(node, next_node) < segment_distance:
                        position, distance_increase = self.find_best_insertion_point(path, node)
                        
                        if position > 0:
                            path.insert(position, node)
                            visited_nodes.add(node)
                            modified = True
                            print(f"  노드 추가됨: {node}, 거리 증가: {distance_increase:.2f}m")
                            break
        
        # 최종 경로 검증
        final_distance = self.route_distance(path)
        if final_distance > self.major_axis:
            print(f"  ⚠️ 경고: 최종 경로 길이({final_distance:.2f}m)가 major_axis({self.major_axis}m)를 초과했습니다!")
            print("  원본 경로로 복구합니다.")
            return original_path
        
        if modified:
            print(f"경로 재연결 완료 (최종 노드 수: {len(path)})")
            print(f"최종 경로 길이: {final_distance:.2f}m")
        else:
            print("변경 사항 없음")
        
        return path

    def run(self):
        """메인 실행 함수"""
        print("\n=== Reattachment Phase 시작 ===")
        
        for uav_id, paths in self.step_reinit_result["reinitialized_paths"].items():
            print(f"\nUAV {uav_id} 경로 처리 중...")
            self.reattached_paths[uav_id] = []
            
            for path in paths:
                reattached_path = self.reattach_nodes(path.copy())
                self.reattached_paths[uav_id].append(reattached_path)
        
        return {
            "reattached_paths": self.reattached_paths
        }
