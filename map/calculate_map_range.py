import geopandas as gpd
from pyproj import CRS

# GeoJSON 파일 읽기
gdf = gpd.read_file('map.geojson')

# 원본 좌표계 확인 (WGS84 경위도)
print("Original CRS:", gdf.crs)

# UTM 좌표계로 변환 (자동 zone 탐지)
utm_zone = int((gdf.total_bounds[0] + 180) // 6 + 1)
utm_crs = CRS.from_dict({'proj':'utm', 'zone':utm_zone, 'ellps':'WGS84'})
gdf_utm = gdf.to_crs(utm_crs)

# 면적 계산 (제곱미터)
area_m2 = gdf_utm.geometry.area.values[0]

# 단위 변환
area_km2 = area_m2 / 1_000_000
area_ha = area_m2 / 10_000

# 결과 출력
print(f"면적: {area_m2:.2f} m²")
print(f"       {area_ha:.2f} 헥타르")
print(f"       {area_km2:.6f} km²")

# 원본 CRS로 다시 변환 (옵션)
gdf = gdf_utm.to_crs(gdf.crs)