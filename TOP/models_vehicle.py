import math
from config_settings import CONFIG
from help import calculate_route, get_nearest_node

class Node:
    def __init__(self, node_id, x, y, score):
        self.id = node_id
        self.x = x
        self.y = y
        self.score = score

class Vehicle:
    def __init__(self, vehicle_id, start_node, capacity):
        self.id = vehicle_id
        self.current_node = start_node
        self.route = [start_node]
        self.capacity = capacity
        self.score = 0

    def move_to(self, node):
        self.route.append(node)
        self.current_node = node
        self.score += node.score

class UAV(Vehicle):
    def __init__(self, vehicle_id, start_node, schedule):
        max_flight_time = CONFIG["uav_flight_time"]
        super().__init__(vehicle_id, start_node, capacity=max_flight_time)
        self.remaining_flight_time = max_flight_time
        self.schedule = schedule
        self.battery_manager = UAVBatteryManager()
        self.route_manager = UAVRouteManager(self)

    def update_state(self, time_spent, current_time):
        """ 비행 시간을 소모하고 충전이 필요한지 체크 """
        self.remaining_flight_time -= time_spent
        self.battery_manager.discharge(time_spent)

        # 현재 시간이 충전 시간(p_matrix)인지 체크
        if self.schedule["p_matrix"][self.id][current_time] == 1:
            self.recharge()

        if self.remaining_flight_time <= 0:
            raise Exception(f"UAV {self.id} must return to UGV for charging!")

    def recharge(self):
        """ 충전 후 비행 시간 리셋 """
        self.remaining_flight_time = self.capacity
        self.battery_manager.charge()

    def get_battery_status(self):
        return self.battery_manager.get_uav_info()

class UGV(Vehicle):
    def __init__(self, vehicle_id, start_node):
        charge_pads = CONFIG.get("ugv_charge_pads", 1)
        super().__init__(vehicle_id, start_node, capacity=charge_pads)
        self.charge_schedule = []  # UAV 충전 스케줄 저장
        self.route_manager = UGVRouteManager(self)

    def schedule_charge(self, uav_id, start_time, end_time):
        self.charge_schedule.append((uav_id, start_time, end_time))

    def can_charge(self, time):
        """ 현재 시간에 충전 가능한지 확인 """
        active_charges = sum(1 for _, s, e in self.charge_schedule if s <= time < e)
        return active_charges < self.capacity

class UAVBatteryManager:
    discharge_rate = CONFIG["uav_battery_discharge_rate"]
    charge_rate = CONFIG["uav_battery_charge_rate"]
    starting_battery = 100  # Example starting battery level

    def __init__(self):
        self.battery_level = self.starting_battery

    def discharge(self, time_spent):
        self.battery_level -= time_spent * self.discharge_rate
        if self.battery_level < 0:
            self.battery_level = 0

    def charge(self):
        self.battery_level += self.charge_rate
        if self.battery_level > self.starting_battery:
            self.battery_level = self.starting_battery

    def get_uav_info(self):
        return {
            "battery_level": self.battery_level,
            "discharge_rate": self.discharge_rate,
            "charge_rate": self.charge_rate,
            "time_to_charge": self.get_time_charge()
        }

    def get_time_charge(self):
        return self.battery_level / self.charge_rate

    def uav_stat(self):
        return f"Battery Level: {self.battery_level}"

    def uav_bat_list(self):
        return [self.battery_level]

class UAVRouteManager:
    def __init__(self, uav):
        self.uav = uav
        self._x = 0
        self._y = 0

    def get_route(self, destination):
        return calculate_route(self.uav.current_node, destination)  # Using external route calculation

    def distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def get_uav_info(self):
        return self.uav.get_battery_status()

    def get_time_info(self):
        return self.uav.remaining_flight_time

    def get_charge_info(self):
        return self.uav.battery_manager.get_time_charge()

    def find_nearest_node(self, nodes):
        return get_nearest_node(self.uav.current_node, nodes)  # Using external nearest node function

class UAV3RouteManager:
    def __init__(self, uav):
        self.uav = uav

    def get_route(self, destination):
        # Implement the logic for UAV3 route calculation
        return calculate_route(self.uav.current_node, destination)  # Placeholder for actual logic

    def find_nearest_node(self, nodes):
        return get_nearest_node(self.uav.current_node, nodes)  # Placeholder for actual logic

class UGVRouteManager:
    def __init__(self, ugv):
        self.ugv = ugv

    def get_route(self, destination):
        # Implement the logic for UGV route calculation
        return calculate_route(self.ugv.current_node, destination)  # Placeholder for actual logic

    def find_nearest_node(self, nodes):
        return get_nearest_node(self.ugv.current_node, nodes)  # Placeholder for actual logic

class DistanceCalculator:
    @staticmethod
    def euclidean_distance(node1, node2):
        return math.sqrt(node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2