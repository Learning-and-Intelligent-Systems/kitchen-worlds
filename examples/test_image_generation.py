from config import OUTPUT_PATH

from pigi_tools.data_utils import  get_worlds_aabb

from data_generator.run_utils import parse_image_rendering_args, process_all_tasks
from data_generator.image_generation import generate_segmented_images


task_name = 'test_feg_kitchen_full'
given_path = None
parallel = False
use_viewer = True
generate_seg = False  ## generate RGB only
redo = True


args = parse_image_rendering_args(
    task_name=task_name, given_path=given_path, parallel=parallel, viewer=use_viewer,
    generate_seg=generate_seg, redo=True
)


def process_worlds_aabb():
    """ bounds ((-0.879, -2.56, -0.002), (1.15, 9.477, 2.841)) """
    run_dirs = process_all_tasks(None, args.task, OUTPUT_PATH, parallel=False, return_dirs=True, input_args=args)
    aabb = get_worlds_aabb(run_dirs)


if __name__ == "__main__":
    kwargs = dict(task_name=args.task, dataset_root=OUTPUT_PATH, parallel=args.parallel, path=args.path, input_args=args)
    process_all_tasks(generate_segmented_images, **kwargs)

    # process_worlds_aabb()
