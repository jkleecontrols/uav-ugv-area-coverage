import pandas as pd
import numpy as np
import json

class DataLoader:
    """ 데이터 로드 및 좌표 변환을 담당하는 클래스 """
    def __init__(self, file_path="environments/corridor_scene/corridor_scene.csv"):
        self.file_path = file_path
        self._data = None  # 내부 변수로 데이터 저장
        self._x_data = None
        self._y_data = None
        self._branching_point = None
        self._left_path = []
        self._center_path = []
        self._right_path = []
        self.load_data()  # 초기화 시 데이터 자동 로드

    def load_data(self):
        """ CSV 데이터를 로드하고 미터 단위로 변환 """
        self._data = pd.read_csv(self.file_path)

        if "Name" in self._data.columns:
            self._named_data = self._data[self._data["Name"].notna()].copy()
            self._visit_data = self._data[self._data["Name"].isna()].copy()
        else:
            self._named_data = pd.DataFrame()
            self._visit_data = self._data.copy()

        self._x_data = 1000 * self._data["X (km)"].values
        self._y_data = 1000 * self._data["Y (km)"].values

        # Identify branching point and separate paths
        self._identify_paths()

    def _identify_paths(self):
        """ 경로를 분기점을 기준으로 세 갈래로 분리 """
        # 분기점은 (6.29, 11.14) 근처로 설정
        branching_x = 6.29 * 1000
        branching_y = 11.14 * 1000
        
        # 모든 좌표를 순회하면서 경로 분리
        current_path = []
        for x, y in zip(self._x_data, self._y_data):
            # 분기점에 도달하면 현재 경로를 저장하고 새로운 경로 시작
            if abs(x - branching_x) < 10 and abs(y - branching_y) < 10:
                if not self._branching_point:
                    self._branching_point = (x, y)
                    self._center_path = current_path.copy()
                    current_path = []
                else:
                    # 두 번째로 만나는 분기점에서 경로 분리
                    if len(current_path) > 0:
                        if current_path[0][0] < branching_x:  # 왼쪽 경로
                            self._left_path = current_path
                        else:  # 오른쪽 경로
                            self._right_path = current_path
                    current_path = []
            else:
                current_path.append((x, y))

    @property
    def data(self):
        """ 데이터 접근 메서드 """
        return self._data

    @property
    def x_data(self):
        """ X 좌표 리스트 반환 """
        return self._x_data

    @property
    def y_data(self):
        """ Y 좌표 리스트 반환 """
        return self._y_data
    
    def get_named_points(self):
        """ depot, area of interest 등 이름이 있는 포인트 반환 """
        return list(zip(1000 * self._named_data["X (km)"], 1000 * self._named_data["Y (km)"]))

    def get_named_points_with_names(self):
        """이름과 좌표를 함께 반환 (이름, x, y) 형태의 리스트로"""
        return list(zip(
            self._named_data["Name"],
            1000 * self._named_data["X (km)"],
            1000 * self._named_data["Y (km)"]
        ))

    def get_aoi_coordinates(self):
        """AOI 좌표만 반환 (x, y) 형태의 리스트로"""
        aoi_data = self._named_data[self._named_data["Name"].str.startswith("AOI")]
        return list(zip(
            1000 * aoi_data["X (km)"],
            1000 * aoi_data["Y (km)"]
        ))

    def get_coordinates(self):
        """ 방문할 좌표 (이름 없는 것들)만 반환하되, 중복 제거 """
        # 중복 제거를 위한 허용 오차 (미터 단위)
        tolerance = 0.1  # 10cm
        
        # 좌표를 그리드로 그룹화하여 중복 제거
        grid = {}
        for x, y in zip(self._x_data, self._y_data):
            # 그리드 셀 인덱스 계산 (tolerance 단위로 반올림)
            grid_x = round(x / tolerance) * tolerance
            grid_y = round(y / tolerance) * tolerance
            grid_key = (grid_x, grid_y)
            
            # 해당 그리드 셀에 아직 좌표가 없으면 추가
            if grid_key not in grid:
                grid[grid_key] = (x, y)
        
        # 중복이 제거된 좌표 리스트 반환
        return list(grid.values())

    def get_all_nodes(self):
        """모든 가능한 노드(좌표)를 반환"""
        return self.get_coordinates()

    def set_vertices(self, coordinates=None):
        if coordinates:
            self._coordinates = coordinates
        else:
            self._coordinates = self.get_coordinates()
        self.graph = self.build_graph()

    def build_graph(self):
        return {"nodes": self._coordinates}

    def get_random_node(self):
        import random
        return random.choice(self._coordinates)

    def compute_path(self, start, end):
        return [start, end]

    def get_branching_point(self):
        """ 분기점 좌표 반환 """
        return self._branching_point

    def get_paths(self):
        """ 세 갈래 경로 반환 """
        return {
            "left": self._left_path,
            "center": self._center_path,
            "right": self._right_path
        }

    def get_next_path(self, current_pos, target_aoi):
        """ 현재 위치와 목표 AOI를 기반으로 다음 경로 선택 """
        # 목표 AOI의 좌표
        target_x = target_aoi[0] * 1000
        target_y = target_aoi[1] * 1000
        
        # 분기점까지의 거리 계산
        dist_to_branch = ((self._branching_point[0] - current_pos[0])**2 + 
                         (self._branching_point[1] - current_pos[1])**2)**0.5
        
        # 분기점에 가까우면 경로 선택
        if dist_to_branch < 10:
            if target_x < self._branching_point[0]:
                return "left"
            elif target_x > self._branching_point[0]:
                return "right"
            else:
                return "center"
        return None

class ConfigManager:
    """ 설정 값을 관리하는 클래스 """
    def __init__(self, config_path="config.json"):
        self.config_path = config_path
        self.config = self.load_config()

    def load_config(self):
        """ JSON에서 설정 값을 불러옴 """
        with open(self.config_path, "r") as f:
            return json.load(f)

    def get(self, key, default=None):
        """ 설정 값 조회 """
        return self.config.get(key, default)
