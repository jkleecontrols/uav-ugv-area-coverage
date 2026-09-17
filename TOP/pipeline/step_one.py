import math
import random

class StepOne:
    def __init__(self, env, uavs, start, end, config, reward_manager):
        self.env = env
        self.uavs = uavs
        self.start = start
        self.end = end
        self.config = config
        self.major_axis = config["major_axis"]
        self.reward_manager = reward_manager
        self.assignments = {}
        self.paths = {}
        self.visited_nodes = set()  # 모든 UAV가 방문한 노드 추적

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def get_valid_points(self):
        """타원 내부에 있는 점들을 반환 (두 초점: start, end, major_axis: config에서 설정)"""
        valid_points = []
        available_points = self.env.get_all_nodes()
        
        # 타원의 두 초점
        focus1 = self.start
        focus2 = self.end
        
        for point in available_points:
            # 시작점, 종료점, 이미 방문한 점은 제외
            if point == focus1 or point == focus2 or point in self.visited_nodes:
                continue
            
            # 타원 내부 점 판별: 두 초점까지의 거리 합이 major_axis보다 작아야 함
            distance_to_focus1 = self.calculate_distance(point, focus1)
            distance_to_focus2 = self.calculate_distance(point, focus2)
            total_distance = distance_to_focus1 + distance_to_focus2
            
            if total_distance <= self.major_axis:
                valid_points.append(point)
            
        print(f"\n타원 내부 점 분석:")
        print(f"- 출발점: {focus1}")
        print(f"- 도착점: {focus2}")
        print(f"- Major axis: {self.major_axis}m")
        print(f"- 타원 내부 점 수: {len(valid_points)}")
        
        return valid_points

    def two_opt(self, path):
        """2-opt 알고리즘을 사용하여 경로 최적화"""
        improved = True
        iteration = 0
        max_iterations = 100  # 최대 반복 횟수 제한
        
        while improved and iteration < max_iterations:
            improved = False
            for i in range(1, len(path) - 2):
                for j in range(i + 1, len(path) - 1):
                    # 현재 경로의 거리
                    current_distance = (self.calculate_distance(path[i-1], path[i]) +
                                      self.calculate_distance(path[j], path[j+1]))
                    # 교환 후 경로의 거리
                    new_distance = (self.calculate_distance(path[i-1], path[j]) +
                                  self.calculate_distance(path[i], path[j+1]))
                    
                    # 거리가 개선되면 경로 교환
                    if new_distance < current_distance:
                        path[i:j+1] = path[i:j+1][::-1]
                        improved = True
            # major_axis 제약 확인
            total_distance = sum(self.calculate_distance(path[i], path[i+1]) 
                               for i in range(len(path)-1))
            if total_distance > self.major_axis:
                # 제약을 위반하면 마지막으로 추가된 노드 제거
                path.pop(-2)
                break
            iteration += 1
            
            if iteration >= max_iterations:
                print("  ⚠️ 2-opt: 최대 반복 횟수 도달")
        
        return path

    def generate_random_path(self, uav_id):
        """가용 노드가 모두 소진될 때까지 major_axis 제약을 만족하는 path들을 생성"""
        print(f"\n🚁 UAV {uav_id} 경로 생성 시작")
        print(f"시작점: {self.start}")
        print(f"종료점: {self.end}")
        
        paths = []
        while True:
            # 유효한 점들 가져오기
            valid_points = self.get_valid_points()
            if not valid_points:
                print("더 이상 가용 노드가 없습니다.")
                break
                
            print(f"\n새로운 경로 생성 중... (남은 가용 노드: {len(valid_points)}개)")
            current_path = [self.start]
            remaining_points = valid_points.copy()
            random.shuffle(remaining_points)
            
            # 랜덤하게 점을 선택하면서 major_axis 제약 확인
            for point in remaining_points:
                # 현재 path에 점을 추가했을 때의 총 거리 계산
                temp_path = current_path + [point] + [self.end]
                total_distance = sum(self.calculate_distance(temp_path[i], temp_path[i+1]) 
                                   for i in range(len(temp_path)-1))
                
                if total_distance <= self.major_axis:
                    current_path.append(point)
                    reward = self.reward_manager.get_current_reward(point)
                    print(f"선택된 점 {point}: 리워드 = {reward:.2f}")
                    self.visited_nodes.add(point)
                    self.reward_manager.update_reward(point)
            
            # 마지막에 도착점 추가
            current_path.append(self.end)
            
            # 2-opt 최적화 적용
            print("2-opt 최적화 적용 중...")
            optimized_path = self.two_opt(current_path.copy())
            
            # path의 총 거리 계산
            path_distance = sum(self.calculate_distance(optimized_path[i], optimized_path[i+1]) 
                              for i in range(len(optimized_path)-1))
            
            print(f"경로 생성 완료")
            print(f"- 방문 점 수: {len(optimized_path)}")
            print(f"- 총 거리: {path_distance:.2f}m")
            print(f"- 경로: {optimized_path}")
            
            paths.append(optimized_path)
        
        return paths

    def run(self):
        for uav in self.uavs:
            # 각 UAV마다 한 번만 경로 생성 (generate_random_path 내부에서 여러 path 생성)
            self.paths[uav.id] = self.generate_random_path(uav.id)
            
        return {
            "paths": self.paths
        }
