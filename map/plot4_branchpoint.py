import pandas as pd
import numpy as np
from scipy.spatial.distance import cdist

# 데이터 로드
df = pd.read_csv('interpolated_map.csv')

# 1. 중복 좌표 기반 분기점 탐지 함수
def detect_branch_points(coords, tolerance=5.0):
    dist_matrix = cdist(coords, coords)
    np.fill_diagonal(dist_matrix, np.inf)
    close_pairs = np.argwhere(dist_matrix < tolerance)
    return coords[np.unique(close_pairs[:,0])]

# 2. 변화량 분석 기반 분기점 탐지 함수
def detect_abrupt_points(coords, window=5, sigma=2.0):
    diffs = np.linalg.norm(np.diff(coords, axis=0), axis=1)
    rolling_mean = np.convolve(diffs, np.ones(window)/window, mode='valid')
    threshold = rolling_mean.mean() + sigma * rolling_mean.std()
    abrupt_idx = np.where(diffs > threshold)[0] + 1
    return coords[abrupt_idx]

# 전체 좌표 추출
all_coords = df[['x', 'y']].values

# 분기점 후보 결합 및 필터링
branch_candidates = np.vstack([
    detect_branch_points(all_coords),
    detect_abrupt_points(all_coords)
])
branch_points = np.unique(branch_candidates.round(decimals=2), axis=0)

# 3. CSV 저장
branch_df = pd.DataFrame(branch_points, columns=['x', 'y'])
branch_df.to_csv('detected_branch_points.csv', index=False)

print(f"총 {len(branch_df)}개의 분기점이 저장되었습니다.")
print(branch_df.head())