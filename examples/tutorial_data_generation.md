# Tutorial: Generate Worlds, Problems, and Plans

Generating data involves creating data folders that include scene layout `scene.lisdf`, `problem.pddl`, `plan.json`, and trajectory `commands.pkl`. It can be run without gui (faster) and can be run in parallel. Note that planning is not guaranteed to be return a solution within timeout, depending on the domain.

There are two scripts for collecting data. 

**Note that both script may result in a failed output folder or early stop of simulation because the problem can't be solved, e.g. when the world generation script failed to find an initial world configuration for randomly sampled objects. It's normal. Just run the script again.**

1) One is simpler, cleaner, and more adaptable for your tasks.    
   * It supports parallel data collection (change to `parallel: true; n_data: 10` in config yaml file).
   * Example configuration files are provided in [kitchen-worlds/pybullet_planning/data_generator/configs](https://github.com/zt-yang/pybullet_planning/blob/master/data_generator/configs/kitchen_full_feg.yaml).
   * **If `parallel: false; n_data: 1`, after a plan is generated, the console will prompt you to press Enter to visualize execution and prompt you to press again to exit the program.**

```shell
python examples/test_data_generation.py --config_name kitchen_full_pr2.yaml  ## PR2 with extended torso range
python examples/test_data_generation.py --config_name kitchen_full_feg.yaml  ## floating franka gripper
python examples/test_data_generation.py --config_name kitchen_full_feg.yaml --simulate  ## simulation mode broken as of Oct 17, 2024
python examples/test_data_generation.py --config_path {path/to/your/custom_data_config.yaml}
```

2) The other uses a more general set of classes and processes. 
   * It supports replaning and continuously interacting with the environment. 
   * It was used to procedually sample scene layouts and generate trajectory data for PIGINet [Sequence-Based Plan Feasibility Prediction for Efficient Task and Motion Planning](https://piginet.github.io/). 
   * Example configuration files are provided in [kitchen-worlds/pybullet_planning/cogarch_tools/configs](https://github.com/zt-yang/pybullet_planning/blob/master/cogarch_tools/configs/config_pigi.yaml).

```shell
python examples/test_data_generation_pigi.py  ## PR2 with extended torso range
```

Once a plan is generated and console stopped generated more logs, press Enter to visualize execution.

The outputs from both scripts will be one or multiple data folders that you can use as input to scripts for rendering images and videos described in the next section.

For example, if a run is generated to `/home/yang/Documents/kitchen-worlds/outputs/test_feg_kitchen_mini/230214_205947`, use `path=test_feg_kitchen_mini/230214_205947`. 

You may also use `task_name=test_feg_kitchen_mini` as input to process all data folders for that task.

### Generate Images and Videos

Render images from camera poses given in `planning_config.json` of the data folders, which originates from the `camera_poses` field of data generation config files. The output images will be in their original data folders.

```shell
python examples/test_image_generation.py --path {path/to/your/task_name/data_dir}
python examples/test_image_generation.py --task {task_name} --parallel
```

Replay the generated trajectory in a given data path. Example configuration files are provided in [kitchen-worlds/pybullet_planning/pigi_tools/configs](https://github.com/zt-yang/pybullet_planning/blob/master/pigi_tools/configs/replay_rss.yaml). You can modify the options in config file to generate mp4, jpg, and gif.

```shell
python examples/test_replay_pigi_data.py --given_path {path/to/your/custom_replay_config.yaml}
```

### Customize Your Layout or Goals

Generate layout only:

```shell
python examples/test_world_builder.py -c kitchen_full_feg.yaml
```


---

## (Recommended) Quick Start Your Scene or Trajectory Generation Pipeline

We suggest putting your custom data generation code and config files inside a directory on the same level as `kitchen-worlds/pybullet_planning` in the project repo. For example, in `kitchen-worlds/your_project_folder`

### Step 1a) To generate PIGINet data

PIGINet data uses a specific procedure to generate scenes with randomized clutter. We recommend following 1b for your custom purposes.

The argument is name to your custom configuration file in [kitchen-worlds/your_project_folder/configs](https://github.com/Learning-and-Intelligent-Systems/kitchen-worlds/blob/master/your_project_folder/configs/config_generation_pigi.yaml):

Output will be in `outputs/{config.data.exp_subdir}/{timestamped_data_name}`.

```shell
## generates data folders with scene, problem, plan, trajectory
python your_project_folder/run_generation_pigi_custom.py

## render images, can run in parallel
python your_project_folder/render_images_custom.py --task custom_piginet_data --parallel
```

### Step 1b) To generate custom data (different world layout, goals, robots, etc.)

The argument is name to your custom configuration file in [kitchen-worlds/your_project_folder/configs](https://github.com/Learning-and-Intelligent-Systems/kitchen-worlds/blob/master/your_project_folder/configs/config_generation.yaml):

Data can be generated in parallel on CPU (set flag in config yaml file).

Output will be in `outputs/{config.data.out_dir}/{timestamped_data_names}`.

```shell
## generates data folders with scene, problem, plan, trajectory
python your_project_folder/run_generation_custom.py --config_name config_generation.yaml 
```


### Step 2) To render images for the generated scene

To train vision language models using generated data, we generate images for all runs in one output subdirectory associated for a task / data generation batch. 

For example, if data is generated using step 1b and default config is used, data will be generated in `outputs/custom_pr2_kitchen_full`, where `custom_pr2_kitchen_full` constitutes our task name here.

Some camera poses are defined in code, some in `outputs/{task_name}/{timestamped_data_name}/planning_config.json`. 

Output will be in each individual `outputs/{task_name}/{timestamped_data_name}/`

```shell 
python your_project_folder/render_images_custom.py --task {task_name}
```

### Step 3) To render video from successful planning runs

Replay the trajectory for review or rendering

Given path, for example, `timestamped_data_dir = 'custom_pr2_kitchen_full/241007_233942'` which contains the scene and plan files. 

The output `replay.mp4` will be saved in the same directory.

```shell
python your_project_folder/run_replay_custom.py -p {timestamped_data_dir}
```

Useful optional flags to this script:
* `--timestep 0.05` to set the simulation timestep, higher makes robot move slower and the video will have more frames.
* `--width 1440` sets the width of the video frame.
* `--height 1080` sets the height of the video frame.
