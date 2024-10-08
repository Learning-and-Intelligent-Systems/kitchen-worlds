from config_custom import DATA_CONFIG_PATH

from cogarch_tools.cogarch_run import run_agent
from cogarch_tools.processes.pddlstream_agent import PDDLStreamAgent


if __name__ == '__main__':
    """ output will be in outputs/custom_piginet_data/{timestamped_run_dir} """
    scene_generation_only = False
    run_agent(
        agent_class=PDDLStreamAgent, config='config_generation_pigi.yaml',
        config_root=DATA_CONFIG_PATH, save_testcase=scene_generation_only
    )
