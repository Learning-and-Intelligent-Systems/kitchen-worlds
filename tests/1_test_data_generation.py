import os
from os.path import basename, dirname


def test_parse_lisdf():
    assert os.system("python examples/test_parse_lisdf.py") == 0


def test_world_builder():
  assert os.system("python examples/test_world_builder.py -c kitchen_full_feg.yaml") == 0


def test_data_generation():
    assert os.system("python examples/test_data_generation.py --config_name kitchen_full_pr2.yaml --skip_prompt") == 0


def test_data_generation_pigi():
    assert os.system("python examples/test_data_generation_pigi.py") == 0


def test_image_generation(run_dir):
    ## render segmented images
    assert os.system(f"python examples/test_image_generation.py --path {run_dir}") == 0
    assert os.system(f"python examples/test_image_generation.py --task {dirname(run_dir)} --parallel") == 0


def test_video_generation(run_dir):
    ## render video of planned trajectory
    assert os.system(f"python examples/test_replay_pigi_data.py --given_path {run_dir}") == 0
    assert os.system(f"python examples/test_replay_pigi_data.py --given_dir {dirname(run_dir)}") == 0


def test_generation_pigi_custom():
    assert os.system("python your_project_folder/run_generation_pigi_custom.py") == 0


def test_generation_custom():
    assert os.system("python your_project_folder/run_generation_custom.py --config_name config_generation.yaml --skip_prompt") == 0


def test_render_images_custom(run_dir):
    assert os.system(f"python your_project_folder/render_images_custom.py --path {run_dir}") == 0
    assert os.system(f"python your_project_folder/render_images_custom.py --task {dirname(run_dir)} --parallel") == 0


def test_run_replay_custom(run_dir):
    assert os.system(f"python your_project_folder/run_replay_custom.py -p {run_dir}") == 0

## ------------------------------------------------------------------


def run_dependencies_tests():
    test_parse_lisdf()
    test_world_builder()


def run_data_generation_tests():
    test_data_generation()
    test_data_generation_pigi()


def get_most_recent_run_dir(test_dir):
    run_name = sorted([d for d in os.listdir(test_dir) if os.path.isdir(f"{test_dir}/{d}")], reverse=True)[0]
    run_dir = f"{basename(test_dir)}/{run_name}"
    print(f'\n\nrun_dir = {run_dir} \n')
    return run_dir


def run_data_processing_tests():
    ## find the most recent run inside outputs/test_pr2_kitchen_full folder, created by test_data_generation()
    run_dir = get_most_recent_run_dir("outputs/test_pr2_kitchen_full")

    test_image_generation(run_dir)
    test_video_generation(run_dir)


def run_custom_project_tests():
    test_generation_pigi_custom()
    test_generation_custom()
    run_dir = get_most_recent_run_dir("outputs/custom_pr2_kitchen_full")
    test_render_images_custom(run_dir)
    test_run_replay_custom(run_dir)

## ------------------------------------------------------------------


if __name__ == "__main__":
    run_dependencies_tests()
    run_data_generation_tests()
    run_data_processing_tests()
    run_custom_project_tests()
    print('\n\n Passed all tests')
