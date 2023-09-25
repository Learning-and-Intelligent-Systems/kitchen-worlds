from test_utils import parallel_processing, get_config


def process(number):
    print(number)
    return

inputs = []
for spec_seed in range(10):
    for env_seed in range(10):
        inputs.append("spec_seed_{}_env_seed_{}".format(spec_seed, env_seed))
parallel_processing(process, inputs, parallel=True)