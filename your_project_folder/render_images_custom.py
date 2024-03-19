from config_custom import OUTPUT_PATH

from data_generator.run_utils import process_all_tasks, parse_image_rendering_args
from data_generator.image_generation import generate_segmented_images


args = parse_image_rendering_args(task_name='custom_piginet_data')


if __name__ == "__main__":
    kwargs = dict(task_name=args.task, dataset_root=OUTPUT_PATH, path=args.path,
                  parallel=args.parallel, input_args=args)
    process_all_tasks(generate_segmented_images, **kwargs)

