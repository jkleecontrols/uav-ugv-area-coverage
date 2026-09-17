import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# 데이터 설정
methods = ['Point-Based', 'Naive Area-Based', 'FOV-Aware (Proposed)']
metrics = ['Coverage (%)', 'Redundancy (%)', 'Avg UAV Delay (s)']

data = {
    'Point-Based': [57.3, 28.1, 38.2],
    'Naive Area-Based': [85.1, 21.6, 42.3],
    'FOV-Aware (Proposed)': [92.4, 13.8, 27.6]
}

# 색상 설정 (논문용으로 적합한 색상 팔레트)
colors = sns.color_palette("muted", n_colors=len(metrics))
proposed_color = '#d62728'  # 제안 방법은 빨간색으로 강조

# 그래프 스타일 설정 - 사용 가능한 스타일 중에서 선택
plt.style.use('seaborn-v0_8-paper')  # 논문용으로 적합한 스타일
sns.set_context("paper", font_scale=1.3, rc={"lines.linewidth": 2.5})

# 3개의 서브플롯 생성
fig, axes = plt.subplots(1, 3, figsize=(15, 5), sharey=False)
fig.tight_layout(pad=4.0)

# 각 메트릭별로 그래프 생성
for i, metric in enumerate(metrics):
    # 데이터 추출
    values = [data[method][i] for method in methods]
    
    # 막대 그래프 생성
    bars = axes[i].bar(methods, values, color=['#1f77b4', '#ff7f0e', proposed_color])
    
    # 제안 방법 강조
    bars[-1].set_edgecolor('black')
    bars[-1].set_linewidth(1.5)
    
    # 축 및 제목 설정
    axes[i].set_title(metric, pad=10)
    axes[i].set_ylim(0, max(values)*1.15)
    axes[i].grid(axis='y', linestyle='--', alpha=0.7)
    
    # x축 레이블 회전 (긴 레이블 방지)
    axes[i].tick_params(axis='x', rotation=15)
    
    # 각 막대 위에 값 표시
    for bar in bars:
        height = bar.get_height()
        axes[i].text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.1f}',
                    ha='center', va='bottom', fontsize=12)

# 전체 제목
fig.suptitle('Performance Comparison of Different Methods', y=1.05, fontsize=16, fontweight='bold')

# 레이아웃 조정
plt.tight_layout()

# 저장 (논문 삽입용 고해상도 이미지)
plt.savefig('method_comparison.pdf', bbox_inches='tight', dpi=300)
plt.savefig('method_comparison.png', bbox_inches='tight', dpi=300)

plt.show()