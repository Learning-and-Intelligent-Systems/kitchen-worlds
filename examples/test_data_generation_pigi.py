import config

from cogarch_tools.cogarch_run import run_agent
from cogarch_tools.processes.pddlstream_agent import PDDLStreamAgent


if __name__ == '__main__':
    seed = 378277
    goal_variations = [3]
    run_agent(
        agent_class=PDDLStreamAgent, config='config_pigi.yaml',
        goal_variations=goal_variations, seed=seed
    )

