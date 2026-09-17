import pandas as pd
import numpy as np
from matplotlib.path import Path
import json
from typing import List, Tuple, Dict

class DataLoader:
    """ 데이터 로드 및 좌표 변환을 담당하는 클래스 """
    def __init__(self, file_path="map/tamu_map.csv"):
        self.file_path = file_path
        self._data = None
        self._boundary_points = []
        self._grid_points = []
        self._road_points = []
        self._named_points = {}
        self._paths = {}
        self.load_data()
        
    def load_data(self):
        """CSV 데이터를 로드하고 경계점과 그리드점 생성"""
        self._data = pd.read_csv(self.file_path)
        
        # 경계점 추출
        boundary_coords = self._data[self._data['feature_id'] == 0]
        self._boundary_points = list(zip(boundary_coords['x'], boundary_coords['y']))
        
        # 폴리곤이 닫혀있지 않으면 닫아주기
        if self._boundary_points[0] != self._boundary_points[-1]:
            self._boundary_points.append(self._boundary_points[0])
            
        # 1m 간격의 그리드 포인트 생성
        self._generate_grid_points()
        
        # 도로망 포인트 생성 (보간된 1m 간격)
        self._generate_road_points()
        
        # 기본 named points 설정 (예시 위치 - 실제 환경에 맞게 조정 필요)
        self._set_default_named_points()

    def _set_default_named_points(self):
        """기본 named points 설정 (DEPOT, AOI 등)"""
        # DEPOT: 시작점을 경계의 중심점으로 설정
        x_coords, y_coords = zip(*self._boundary_points)
        depot_x = sum(x_coords) / len(x_coords)
        depot_y = sum(y_coords) / len(y_coords)
        self._named_points['DEPOT'] = [(depot_x, depot_y)]
        
        # AOI: 예시로 4개의 관심 지점 설정
        # 실제 환경에 맞게 조정 필요
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        
        aoi_points = [
            (x_min + (x_max - x_min) * 0.25, y_min + (y_max - y_min) * 0.25),
            (x_min + (x_max - x_min) * 0.75, y_min + (y_max - y_min) * 0.25),
            (x_min + (x_max - x_min) * 0.25, y_min + (y_max - y_min) * 0.75),
            (x_min + (x_max - x_min) * 0.75, y_min + (y_max - y_min) * 0.75)
        ]
        self._named_points['AOI'] = aoi_points
        
    def _generate_grid_points(self):
        """10m 간격의 그리드 포인트 생성"""
        # 경계 영역 계산
        x_coords, y_coords = zip(*self._boundary_points)
        x_min, x_max = int(np.floor(min(x_coords))), int(np.ceil(max(x_coords)))
        y_min, y_max = int(np.floor(min(y_coords))), int(np.ceil(max(y_coords)))
        
        # matplotlib Path 객체 생성
        boundary_path = Path(self._boundary_points)
        
        # 10m 간격 그리드 생성
        x_grid = np.arange(x_min, x_max + 1, 10)  # 10m 간격으로 변경
        y_grid = np.arange(y_min, y_max + 1, 10)  # 10m 간격으로 변경
        X, Y = np.meshgrid(x_grid, y_grid)
        points = np.vstack((X.ravel(), Y.ravel())).T
        
        # 경계 내부의 점만 선택
        inside_mask = boundary_path.contains_points(points)
        self._grid_points = points[inside_mask].tolist()
        
    def _generate_road_points(self):
        """도로망 포인트를 1m 간격으로 보간하여 생성"""
        road_segments = self._data[self._data['feature_id'] != 0]
        self._road_points = []
        
        for _, group in road_segments.groupby('feature_id'):
            points = list(zip(group['x'], group['y']))
            interpolated_points = self._interpolate_points(points, interval=1.0)
            self._road_points.extend(interpolated_points)
            
    def _interpolate_points(self, points: List[Tuple[float, float]], interval: float) -> List[Tuple[float, float]]:
        """점들을 주어진 간격으로 보간"""
        interpolated = []
        for i in range(len(points) - 1):
            p1, p2 = points[i], points[i + 1]
            distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            num_points = max(2, int(np.ceil(distance / interval)))
            
            for t in np.linspace(0, 1, num_points):
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                interpolated.append((round(x, 3), round(y, 3)))
                
        return interpolated
        
    def get_grid_points(self) -> List[Tuple[float, float]]:
        """1m 간격의 그리드 포인트 반환"""
        return self._grid_points
        
    def get_road_points(self) -> List[Tuple[float, float]]:
        """도로망 포인트 반환"""
        return self._road_points
        
    def get_boundary_points(self) -> List[Tuple[float, float]]:
        """경계 포인트 반환"""
        return self._boundary_points
        
    def get_named_points(self, point_type: str = None) -> Dict[str, List[Tuple[float, float]]]:
        """명명된 포인트 반환"""
        if point_type:
            return {point_type: self._named_points.get(point_type, [])}
        return self._named_points
        
    def get_depot_location(self) -> Tuple[float, float]:
        """Depot 위치 반환"""
        depot_points = self._named_points.get('DEPOT', [])
        if not depot_points:
            raise ValueError("No depot location found in the data")
        return depot_points[0]
        
    def get_aoi_locations(self) -> List[Tuple[float, float]]:
        """AOI 위치들 반환"""
        return self._named_points.get('AOI', [])
        
    def is_point_inside_boundary(self, point: Tuple[float, float]) -> bool:
        """주어진 점이 경계 내부에 있는지 확인"""
        return Path(self._boundary_points).contains_point(point)
        
    def get_nearest_road_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """주어진 점에서 가장 가까운 도로 포인트 반환"""
        distances = [(p, np.sqrt((p[0]-point[0])**2 + (p[1]-point[1])**2)) 
                    for p in self._road_points]
        return min(distances, key=lambda x: x[1])[0]
        
    def save_processed_data(self, output_file: str):
        """처리된 데이터를 JSON 형식으로 저장"""
        data = {
            'boundary_points': self._boundary_points,
            'grid_points': self._grid_points,
            'road_points': self._road_points,
            'named_points': self._named_points
        }
        with open(output_file, 'w') as f:
            json.dump(data, f)
            
    def load_processed_data(self, input_file: str):
        """저장된 처리 데이터 로드"""
        with open(input_file, 'r') as f:
            data = json.load(f)
            self._boundary_points = data['boundary_points']
            self._grid_points = data['grid_points']
            self._road_points = data['road_points']
            self._named_points = data['named_points']

    def get_first_coordinate(self) -> Tuple[float, float]:
        """CSV 파일의 첫 번째 좌표 반환"""
        if self._data is None or self._data.empty:
            raise ValueError("No data loaded")
        first_row = self._data.iloc[0]
        return (first_row['x'], first_row['y'])

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
