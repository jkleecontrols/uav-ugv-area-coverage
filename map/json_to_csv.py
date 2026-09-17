import geopandas as gpd
import matplotlib.pyplot as plt
from shapely import affinity
import csv

# 1) GeoJSON (위/경도) 로드 -> GeoDataFrame
gdf = gpd.read_file("map/newtamu.geojson", encoding='utf-8')

# 2) UTM Zone 15N으로 변환 (단위: m)
gdf_utm = gdf.to_crs(epsg=32615)

# 3) (178750, 3390500)을 새 원점(0,0)으로 평행이동
SHIFT_X = 176500
SHIFT_Y = 3389000

def shift_geometry(geom, shift_x, shift_y):
    # (shift_x, shift_y)를 (0,0)으로 만들기 위해 음의 방향 이동
    return affinity.translate(geom, xoff=-shift_x, yoff=-shift_y)

gdf_utm['geometry'] = gdf_utm['geometry'].apply(lambda g: shift_geometry(g, SHIFT_X, SHIFT_Y))

# 4) 시각화 (옵션)
# fig, ax = plt.subplots(figsize=(8,8))
# gdf_utm.plot(ax=ax, alpha=0.5, edgecolor='black')
# ax.set_aspect('equal', 'box')
# ax.set_xlim(0, 2100)
# ax.set_ylim(0, 2100)
# plt.title("Map with New Origin in EPSG:32615")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.grid(True)
# plt.show()

# 5) CSV로 저장
#    - 모든 geometry를 순회하면서 좌표를 행별로 기록
#    - geometry 타입에 따라 분기 처리
output_csv = "map/newtamu.csv"
with open(output_csv, 'w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    # 헤더: feature_id, part_id, coord_index, x, y
    writer.writerow(["feature_id", "part_id", "coord_index", "x", "y"])
    
    for fid, geom in enumerate(gdf_utm['geometry']):
        geom_type = geom.geom_type
        
        if geom_type == "LineString":
            # 단일 라인
            coords = list(geom.coords)
            for cid, (x, y) in enumerate(coords):
                writer.writerow([fid, 0, cid, x, y])

        elif geom_type == "Polygon":
            # 폴리곤 외곽만 저장 (필요 시 내부 링도 추가)
            exterior_coords = list(geom.exterior.coords)
            for cid, (x, y) in enumerate(exterior_coords):
                writer.writerow([fid, 0, cid, x, y])

        elif geom_type == "MultiLineString":
            # 복수 라인
            for part_id, line in enumerate(geom.geoms):
                coords = list(line.coords)
                for cid, (x, y) in enumerate(coords):
                    writer.writerow([fid, part_id, cid, x, y])

        elif geom_type == "MultiPolygon":
            # 복수 폴리곤
            for part_id, polygon in enumerate(geom.geoms):
                exterior_coords = list(polygon.exterior.coords)
                for cid, (x, y) in enumerate(exterior_coords):
                    writer.writerow([fid, part_id, cid, x, y])

        elif geom_type == "Point":
            # 점
            (x, y) = geom.coords[0]
            writer.writerow([fid, 0, 0, x, y])

        # 필요에 따라 "MultiPoint", "GeometryCollection" 등 다른 타입도 처리 가능

print(f"CSV 저장 완료: {output_csv}")