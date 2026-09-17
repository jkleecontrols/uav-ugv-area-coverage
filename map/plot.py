# 다시 한번 그래프 생성 시도

import networkx as nx
import matplotlib.pyplot as plt

# 상태(State) 정의
states = ["Idle", "Follow-Wall", "Detect-Ice", "Spread-CaCl₂", "Obstacle-Avoidance"]

# 상태 전이(Transitions) 정의
transitions = [
    ("Idle", "Follow-Wall", "Snow detected"),
    ("Follow-Wall", "Detect-Ice", "Ice Detected "),
    ("Follow-Wall", "Obstacle-Avoidance", "Obstacle Detected"),
    ("Detect-Ice", "Spread-CaCl₂", "Ice & no human"),
    ("Detect-Ice", "Follow-Wall", "no Ice or Human"),
    ("Spread-CaCl₂", "Follow-Wall", "CaCl2 Done"),
    ("Obstacle-Avoidance", "Follow-Wall", "Steer away")
]

# 유한 상태 기계(FSA) 그래프 생성
G = nx.DiGraph()

# 노드 추가
for state in states:
    G.add_node(state)

# 엣지 추가 (전이)
for from_state, to_state, condition in transitions:
    G.add_edge(from_state, to_state, label=condition)

# 그래프 레이아웃 설정
pos = nx.spring_layout(G, seed=42)  # 자동 레이아웃 설정

# 그래프 그리기
plt.figure(figsize=(10, 6))
nx.draw(G, pos, with_labels=True, node_size=3000, node_color="lightblue", edge_color="gray", font_size=10, font_weight="bold", arrows=True)

# 엣지 라벨 추가
edge_labels = {(from_state, to_state): condition for from_state, to_state, condition in transitions}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=9)

# 다이어그램 저장
file_path = "fsa_snow_removal_robot.png"
plt.title("Finite State Automata (FSA) for Snow Removal Robot")
plt.savefig(file_path)
plt.show()

# 파일 경로 반환
file_path