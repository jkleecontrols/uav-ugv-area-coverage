import math

def calculate_route(start_node, end_node):
    """
    Placeholder for route calculation between start_node and end_node.
    Replace with actual pathfinding logic (e.g., A*, Dijkstra).
    """
    return [start_node, end_node]

def get_nearest_node(current_node, node_list):
    """
    Returns the nearest node to current_node from node_list based on Euclidean distance.
    Assumes nodes are (x, y) tuples or have x and y attributes.
    """
    def distance(a, b):
        if isinstance(a, tuple):
            ax, ay = a
        else:
            ax, ay = a.x, a.y
        if isinstance(b, tuple):
            bx, by = b
        else:
            bx, by = b.x, b.y
        return math.sqrt((ax - bx)**2 + (ay - by)**2)

    return min(node_list, key=lambda node: distance(current_node, node))

class SimulationManager:
    def __init__(self, config, coordinates, uav_schedule, initial_point, max_simulation_time, loader):
        self.config = config
        self.coordinates = coordinates
        self.uav_schedule = uav_schedule
        self.initial_point = initial_point
        self.max_simulation_time = max_simulation_time
        self.loader = loader
        self.ugv_simulation_data = {"time": [], "position": [], "state": []}
        self.uav_simulation_data = {
            "uav_states": [[] for _ in range(config["num_uav"])],
            "uav_positions": [[] for _ in range(config["num_uav"])],
            "uav_start_points": [[] for _ in range(config["num_uav"])],
            "uav_end_points": [[] for _ in range(config["num_uav"])]
        }

    def calculate_distance(self, point1, point2):
        return ((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)**0.5

    def interpolate_position(self, start, end, ratio):
        return (
            round(start[0] + (end[0] - start[0]) * ratio, 2),
            round(start[1] + (end[1] - start[1]) * ratio, 2)
        )

    def get_next_position(self, current_pos, target_pos, speed, time_step):
        distance = self.calculate_distance(current_pos, target_pos)
        if distance <= speed * time_step:
            return (round(target_pos[0], 2), round(target_pos[1], 2))
        ratio = (speed * time_step) / distance
        return self.interpolate_position(current_pos, target_pos, ratio)

    def select_next_path(self):
        """ 현재 위치와 목표를 기반으로 다음 경로 선택 """
        # 분기점에 도달했는지 확인
        if self.calculate_distance(self.ugv_current_pos, self.branching_point) < 10:
            # 가장 가까운 미방문 AOI 찾기
            nearest_aoi = self.find_nearest_aoi()
            if nearest_aoi:
                # AOI 방향에 따라 경로 선택
                selected_path = self.data_loader.get_next_path(self.ugv_current_pos, nearest_aoi)
                if selected_path:
                    self.current_path = self.paths[selected_path]
                    self.path_index = 0
                    print(f"🔄 Selected {selected_path} path towards AOI at {nearest_aoi}")

    def find_nearest_aoi(self):
        """ 현재 위치에서 가장 가까운 미방문 AOI 찾기 """
        aois = self.data_loader.get_aoi_coordinates()
        
        min_dist = float('inf')
        nearest_aoi = None
        
        for aoi in aois:
            dist = self.calculate_distance(
                self.ugv_current_pos,
                aoi  # 이미 미터 단위로 변환된 좌표
            )
            if dist < min_dist:
                min_dist = dist
                nearest_aoi = aoi
                
        return nearest_aoi

    def update_ugv_target(self):
        """ UGV의 다음 목표 위치 업데이트 """
        if self.current_path and self.path_index < len(self.current_path):
            # 현재 경로에서 다음 좌표 선택
            self.ugv_target_pos = self.current_path[self.path_index]
            self.path_index += 1
        else:
            # 경로가 없거나 끝에 도달한 경우
            self.select_next_path()
            if self.current_path:
                self.ugv_target_pos = self.current_path[0]
                self.path_index = 1
            else:
                # 기본 경로로 돌아가기
                self.ugv_target_index = (self.ugv_target_index + 1) % len(self.coordinates)
                self.ugv_target_pos = self.coordinates[self.ugv_target_index]

    def step(self, time_step):
        """시뮬레이션 한 스텝 진행"""
        # 현재 시간의 리워드 값 기록
        current_rewards = self.reward_manager.get_all_rewards()
        self.reward_history.append({
            "time": self.ugv_simulation_data["time"][-1] if self.ugv_simulation_data["time"] else 0,
            "rewards": {
                "points": list(current_rewards.keys()),
                "values": list(current_rewards.values())
            }
        })
        
        # 기존 시뮬레이션 로직 실행
        time_step = 1
        
        for t in range(0, self.max_simulation_time, time_step):
            current_time = t
            self.ugv_simulation_data["time"].append(current_time)
            
            # Update UGV position
            ugv_speed = self.config["ugv_takeoffland_speed"] if any(
                uav["state"] in ["takeoff", "landing"] for uav in self.uav_states
            ) else self.config["ugv_drive_speed"]
            
            self.ugv_current_pos = self.get_next_position(
                self.ugv_current_pos,
                self.ugv_target_pos,
                ugv_speed,
                time_step
            )
            self.ugv_simulation_data["position"].append(self.ugv_current_pos)
            
            # Check if UGV reached target and update next target
            if self.calculate_distance(self.ugv_current_pos, self.ugv_target_pos) < 0.1:
                self.update_ugv_target()
                print(f"🚗 [time {current_time:.1f}s] UGV reached target, moving to next coordinate: {self.ugv_target_pos}")
            
            # Update each UAV
            for uav_index in range(self.config["num_uav"]):
                uav_key = f"uav_{uav_index + 1}"
                uav = self.uav_states[uav_index]
                
                # Check if UAV should start flying
                schedule_index = (t // self.config["uav_flight_charge_gcd"]) % len(self.uav_schedule[uav_key])
                if (t % self.config["uav_flight_charge_gcd"] == 0 and 
                    self.uav_schedule[uav_key][schedule_index] == 1 and 
                    uav["state"] == "charging"):
                    uav["state"] = "takeoff"
                    uav["flight_start_time"] = current_time
                    uav["flight_end_time"] = current_time + self.config["uav_flight_time"]
                    uav["start_point"] = self.ugv_current_pos
                    print(f"🚁 [time {current_time:.1f}s] UAV {uav_index+1} starts takeoff")
                
                # Update UAV state and position
                if uav["state"] == "charging":
                    uav["position"] = self.ugv_current_pos
                elif uav["state"] == "takeoff":
                    takeoff_progress = (current_time - uav["flight_start_time"]) / self.config["uav_takeoff_time"]
                    if takeoff_progress >= 1.0:
                        uav["state"] = "flying"
                        flight_distance = self.config["ugv_drive_speed"] * (
                            self.config["uav_flight_time"] - 
                            self.config["uav_takeoff_time"] - 
                            self.config["uav_landing_time"]
                        )
                        uav["end_point"] = (
                            round(uav["start_point"][0] + flight_distance, 2),
                            round(uav["start_point"][1], 2)
                        )
                        print(f"🚁 [time {current_time:.1f}s] UAV {uav_index+1} starts flying")
                    uav["position"] = self.ugv_current_pos
                elif uav["state"] == "flying":
                    uav["position"] = uav["start_point"]
                elif uav["state"] == "landing":
                    if current_time >= uav["flight_end_time"]:
                        uav["state"] = "charging"
                        print(f"🚁 [time {current_time:.1f}s] UAV {uav_index+1} finished landing")
                    uav["position"] = self.ugv_current_pos
                
                # Only store UAV data when state changes
                if uav["state"] != uav["previous_state"]:
                    self.uav_simulation_data["uav_states"][uav_index].append(uav["state"])
                    self.uav_simulation_data["uav_start_points"][uav_index].append(uav["start_point"])
                    self.uav_simulation_data["uav_end_points"][uav_index].append(uav["end_point"])
                    uav["previous_state"] = uav["state"]
                
                # Check if UAV should start landing
                if uav["state"] == "flying" and current_time >= uav["flight_end_time"] - self.config["uav_landing_time"]:
                    uav["state"] = "landing"
                    print(f"🚁 [time {current_time:.1f}s] UAV {uav_index+1} starts landing")
        
        return self.ugv_simulation_data, self.uav_simulation_data

    def get_simulation_data(self):
        """시뮬레이션 데이터 반환"""
        return self.ugv_simulation_data, self.uav_simulation_data

    def get_reward_history(self):
        """리워드 히스토리 반환"""
        return self.reward_history