import config

from cogarch_tools.cogarch_run import run_agent
from cogarch_tools.processes.pddlstream_agent import PDDLStreamAgent


if __name__ == '__main__':
    run_agent(
        agent_class=PDDLStreamAgent, config='config_pigi.yaml',
    )

