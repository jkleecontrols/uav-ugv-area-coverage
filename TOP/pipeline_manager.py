from pipeline.step_one import StepOne
from pipeline.step_two import StepTwo
from pipeline.step_reinit import StepReinit
from pipeline.step_reattachment import StepReattachment

# pipeline_manager.py
class PipelineManager:
    def __init__(self, env, uavs, start, end, config, reward_manager):
        self.env = env
        self.uavs = uavs
        self.start = start
        self.end = end
        self.config = config
        self.reward_manager = reward_manager
        self.state = {}
        self.max_reinit_cycles = 3  # 최대 재시도 횟수
        self.best_score = float('-inf')

    def run_step_one(self):
        step = StepOne(
            env=self.env, 
            uavs=self.uavs,
            start=self.start,
            end=self.end,
            config=self.config,
            reward_manager=self.reward_manager
        )
        self.state["step1"] = step.run()

    def run_step_two(self):
        step = StepTwo(
            env=self.env, 
            step_one_result=self.state["step1"],
            start=self.start,
            end=self.end,
            config=self.config
        )
        self.state["step2"] = step.run()
        return step.get_team_score()  # team score 반환

    def run_step_reinit(self):
        step = StepReinit(
            env=self.env, 
            step_two_result=self.state["step2"],
            config=self.config
        )
        reinit_result = step.run()
        self.state["step_reinit"] = reinit_result
        return reinit_result["record"]  # record score 반환

    def run_step_reattachment(self):
        step = StepReattachment(
            env=self.env,
            step_reinit_result=self.state["step_reinit"],
            config=self.config
        )
        self.state["step_reattachment"] = step.run()

    def run(self):
        """전체 파이프라인 실행"""
        print("\n=== Step 1: Initialization ===")
        self.run_step_one()
        
        reinit_count = 0
        while reinit_count < self.max_reinit_cycles:
            print(f"\n=== Cycle {reinit_count + 1} ===")
            
            print("\n=== Step 2: Improvement ===")
            current_score = self.run_step_two()
            
            print("\n=== Step 3: Reinitialization & Tabu Search ===")
            new_score = self.run_step_reinit()
            
            if new_score > self.best_score:
                self.best_score = new_score
                print(f"🎉 새로운 최고 점수 발견: {new_score}")
                reinit_count = 0  # 개선이 있으면 카운터 리셋
            else:
                reinit_count += 1
                print(f"😐 개선 없음 (시도 {reinit_count}/{self.max_reinit_cycles})")
        
        print("\n=== Step 4: Reattachment ===")
        self.run_step_reattachment()
        
        return self.get_result()

    def get_result(self):
        return self.state