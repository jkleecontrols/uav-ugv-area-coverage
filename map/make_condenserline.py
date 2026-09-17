import pandas as pd
import numpy as np

# CSV 파일 읽기
df = pd.read_csv('condensedmap.csv')

# 보간 함수 (두 점 사이에 num_points 개의 점 추가)
def interpolate_points(start, end, num_points):
    x_vals = np.linspace(start['x'], end['x'], num_points + 2)
    y_vals = np.linspace(start['y'], end['y'], num_points + 2)
    return [(x, y) for x, y in zip(x_vals[1:-1], y_vals[1:-1])]

# 각 그룹별 보간 적용
interpolated_rows = []
grouped = df.groupby(['feature_id', 'part_id'])

for (feature_id, part_id), group in grouped:
    coords = group.sort_values('coord_index').to_dict('records')
    new_coords = []
    
    for i in range(len(coords)-1):
        # 현재 점과 다음 점
        current = coords[i]
        next_pt = coords[i+1]
        
        # 현재 점 추가
        new_coords.append(current)
        
        # 두 점 사이에 5개의 점 보간 (값 조절 가능)
        interpolated = interpolate_points(current, next_pt, num_points=5)
        for x, y in interpolated:
            new_row = {
                'feature_id': feature_id,
                'part_id': part_id,
                'x': x,
                'y': y,
                'coord_index': None  # 나중에 재할당
            }
            new_coords.append(new_row)
    
    # 마지막 점 추가
    new_coords.append(coords[-1])
    
    # coord_index 재할당
    for idx, row in enumerate(new_coords):
        row['coord_index'] = idx
    interpolated_rows.extend(new_coords)

# 새로운 DataFrame 생성
new_df = pd.DataFrame(interpolated_rows)
new_df = new_df.sort_values(['feature_id', 'part_id', 'coord_index'])

# CSV 저장
new_df.to_csv('interpolated_map.csv', index=False)