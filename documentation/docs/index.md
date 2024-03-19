# Kitchen Worlds

A library of long-horizon Task-and-Motion-Planning (TAMP) problems in kitchen and household scenes, as well as planners to solve them

- procedurally generate scenes with rigid and articulated objects
- visualize a scene in LISDF format (an extension to SDF that includes URDF)
- solve using PDDLStream a TAMP problem defined by a `scene.lisdf` and `problem.pddl` using existing `domain.pddl` and `stream.pddl`
- visualize the robot trajectory in pybullet

<img src="gifs/demo-cabbage.gif"></img>

<!--
<video autoplay loop muted playsinline width=100%>
  <source src="mp4/demo-cabbage.mp4" type="video/mp4">
</video>

<table class="multicol tightframes">
<tr>
<td width="33%">

<img src="imgs/demo-cabbage-1.png"></img>

</td>
<td width="33%">

<img src="imgs/demo-cabbage-2.png"></img>

</td>
<td width="33%">

<img src="imgs/demo-cabbage-4.png"></img>

</td>
</tr>
</table>
-->

## Setting Up

Clone the repo along with the submodules. It may take a while.

```shell
git clone git@github.com:Learning-and-Intelligent-Systems/kitchen-worlds.git --recursive
```

Install dependencies. Install [graphviz](https://pygraphviz.github.io/documentation/latest/install.html).

```shell
conda env create -f environment.yml
conda activate kitchen
## sudo apt-get install graphviz graphviz-dev  ## on Ubuntu
```

Build FastDownward, used by PDDLStream planner

```shell
## sudo apt install cmake g++ git make python3
(cd pddlstream; ./downward/build.py)
```

Build IK solvers (If using PR2; skip this if you're only using floating gripper).

1) IKFast solver for PR2 arm planning (the default IK):

```shell
## sudo apt-get install python-dev
(cd pybullet_planning/pybullet_tools/ikfast/pr2; python setup.py)
```

2) TracIK for PR2 whole-body IK that solves for base, torso, and arm together (this is better, but requires Ubuntu):

```shell
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev swig
pip install git+https://github.com/mjd3/tracikpy.git
```


## Quick Start

We suggest putting your custom data generation code and config files inside a directory on the same level as `kitchen-worlds/pybullet_planning` in the project repo. For example, in `kitchen-worlds/your_project_folder`

### To generate PIGINet data

The argument is name to your custom configuration file in [kitchen-worlds/your_project_folder/configs](https://github.com/Learning-and-Intelligent-Systems/kitchen-worlds/blob/master/your_project_folder/configs/config_generation_pigi.yaml):

```shell
## generates data folders with scene, problem, plan, trajectory
python your_project_folder/run_generation_pigi_custom.py

## render images, can run in parallel
python your_project_folder/render_images_custom.py --task custom_piginet_data --parallel
```

### To generate custom data (different world layout, goals, robots, etc.)

The argument is name to your custom configuration file in [kitchen-worlds/your_project_folder/configs](https://github.com/Learning-and-Intelligent-Systems/kitchen-worlds/blob/master/your_project_folder/configs/config_generation.yaml):

Data can be generated in parallel on CPU (set flag in config yaml file).

```shell
## generates data folders with scene, problem, plan, trajectory
python your_project_folder/run_generation_custom.py --config_name config_generation.yaml
```

---

## Examples

### Generate Worlds, Problems, and Plans

Collecting data involves generating data folders that include scene layout `scene.lisdf`, `problem.pddl`, `plan.json`, and trajectory `commands.pkl`. It can be run without gui (faster) and can be run in parallel. Note that planning is not guaranteed to be return a solution within timeout, depending on the domain.

There are two scripts for collecting data:

1) One is simpler, cleaner, and more adaptable for your tasks. It supports parallel data collection (change to `parallel: true; n_data: 10` in config yaml file). Example configuration files are provided in [kitchen-worlds/pybullet_planning/data_generator/configs](https://github.com/zt-yang/pybullet_planning/blob/master/data_generator/configs/kitchen_full_feg.yaml):

```shell
python examples/test_data_generation.py --config_name kitchen_full_pr2.yaml  ## PR2 with extended torso range
python examples/test_data_generation.py --config_name kitchen_full_feg.yaml  ## floating franka gripper
python examples/test_data_generation.py --config_path {path/to/your/custom_data_config.yaml}
```

2) The other uses a more general set of classes and processes. It supports replaning and continuously interacting with the environment. It was used to generate data for PIGINet [Sequence-Based Plan Feasibility Prediction for Efficient Task and Motion Planning](https://piginet.github.io/). Example configuration files are provided in [kitchen-worlds/pybullet_planning/cogarch_tools/configs](https://github.com/zt-yang/pybullet_planning/blob/master/cogarch_tools/configs/config_pigi.yaml):

```shell
python examples/test_data_generation_pigi.py  ## PR2 with extended torso range
```

The outputs from both scripts will be one or multiple data folders that you can use as input to the following post-processing scripts.

For example, a path may be `/home/yang/Documents/kitchen-worlds/outputs/test_feg_kitchen_mini/230214_205947`. You may also use the parent folder name `test_feg_kitchen_mini` as input to process all data folders for that task.

### Generate Images and Videos

Render images from camera poses given in `planning_config.json` of the data folders, which originates from the `camera_poses` field of data generation config files. The output images will be in their original data folders.

```shell
python examples/test_image_generation.py --task {path/to/your/task_name/data_dir}
python examples/test_image_generation.py --path {task_name}
```

Replay the generated trajectory in a given data path. Example configuration files are provided in [kitchen-worlds/pybullet_planning/pigi_tools/configs](https://github.com/zt-yang/pybullet_planning/blob/master/pigi_tools/configs/replay_rss.yaml). You can modify the options in config file to generate mp4, jpg, and gif.

```shell
python examples/test_replay_pigi_data.py --name replay_rss.yaml
python examples/test_replay_pigi_data.py --path {path/to/your/custom_replay_config.yaml}
```

### Customize Your Layout or Goals

Generate layout only:

```shell
python examples/test_world_builder.py -c kitchen_full_feg.yaml
```

## Acknowledgements

This repo works thanks for the tools provided by LIS lab members and alum:

* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is an awesome set of tools developed by Caelan Garret. A forked version is included with my own helper functions.
* the [pddlstream](https://github.com/caelan/pddlstream) is a planning framework developed by Caelan Garret.
* the [lisdf](https://github.com/Learning-and-Intelligent-Systems/lisdf) package is an input/output specification for TAMP problems developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao.

All the object models and urdf files are downloaded for free from the following sources:

* most articulated object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)
* most kitchen object models are downloaded from [Free3D](https://free3d.com/3d-models/food).

