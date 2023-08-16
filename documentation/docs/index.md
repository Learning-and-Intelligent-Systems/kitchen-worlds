# Kitchen Worlds

A library of long-horizon Task-and-Motion-Planning (TAMP) problems in kitchen and household scenes, as well as planners to solve them

- visualize a scene in LISDF format (an extension to SDF that includes URDF)
- solve a TAMP problem using PDDLStream defined with a scene.lisdf, problem.pddl, domain.pddl, stream.pddl
- procedurally generate scenes with rigid and articulated objects

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

## Notes for NS-Plan

1. install
```shell
pip install gymnasium
# compile ikfast
pybullet-planning$ (cd pybullet_tools/ikfast/franka_panda; python setup.py)
```
2. checkout assets
```shell
cd ./assets/models
git checkout ns_plan_acronym_object
```

3. generate semantic specs
```shell
# from repo root
cd ./pybullet_planning/nsplan_tools
python generate_semantic_specification.py
```

4. collect rollouts
First change yaml config files for data collection in `./configs/`, then run data collection code
```shell
# from repo root
cd ./tests
python prototype_clean_dish_gym.py
```


## Setup

1. Grab the submodules, may take a while

```shell
cd kitchen-worlds
git submodule update --init --recursive
```

2. Install dependencies
```shell
conda env create -f environment.yml
conda activate kitchen
sudo apt-get install graphviz graphviz-dev  ## on Ubuntu
```

3. Build FastDownward, used by PDDLStream planner

```shell
## sudo apt install cmake g++ git make python3
(cd pddlstream; ./downward/build.py)
```

4. Build IK solvers (If using PR2)

IKFast solver for PR2 arm planning:

```shell
## sudo apt-get install python-dev
(cd pybullet_planning/pybullet_tools/ikfast/pr2; python setup.py)
```

TracIK for PR2 base, torso, and arm planning:

```shell
cd ..
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev swig
git clone https://github.com/mjd3/tracikpy.git
pip install tracikpy/
```


[graphviz](https://pygraphviz.github.io/documentation/latest/install.html)

## Examples

collects data (including scene layout `scene.lisdf`, `problem.pddl`, plan, and trajectory) according to the configuration file in [pybullet_planning/pipelines](https://github.com/zt-yang/pybullet_planning/blob/master/pipelines/kitchen_full_feg.yaml) directory, and outputs to a subdirectory in `kitchen-worlds/outputs/test_feg_kitchen_mini/` named by datetime. 

```commandline
python test_data_generation.py -c kitchen_full_feg.yaml
```

replays the generated trajectory and generates a `replay.gif` in a given path to the data directory (containing `scene.lisdf`, `problem.pddl`, `commands.pkl`), for example:

```commandline
python test_replay.py --path /home/yang/Documents/kitchen-worlds/outputs/test_feg_kitchen_mini/230214_205947
```

----------

The test scripts below this line haven't been tested as of Feb 2023. Will be updated soon.

----------

## Examples (obsolete)

### Test planner for MALAO project

```
(cd tests; ./rerun.sh)
```


### LISDF parser testing

To run all tests before git push, do
```commandline
cd tests
sh run_tests.sh
```

To test basic lisdf functions
```commandline
## Test LISDF parser
python test_parse_lisdf.py

## Test load LISDF to Pybullet
python test_pybullet_lisdf.py

##  Test parse problem.pddl
python test_parse_pddl.py
```

### Planning

To solve some test problems wih PDDLStream, `-test` takes the name of subdirectory inside `test_cases` folder.

```commandline
cd pybullet_planning/tests
python test_pddlstream.py -test test_fridges_tables
python test_floating_gripper.py -test test_feg_pick
```

Some test cases (verified that they can run on Ubuntu with Python 3.7/3.8):

<table class="multicol">
<tr>
<td width="50%">

<img src="gifs/220613-clean-only.gif"></img>

</td>
<td width="50%">

<img src="gifs/220613-rearraneg-only.gif"></img>

</td>
</tr>
<tr>
<td width="50%">

`test_feg_clean_only`

</td>
<td width="50%">

`test_feg_cabinets_rearrange`

</td>
</tr>
</table>

Note: There may be weird bounding boxes drawn during planning and pybullet throwing errors. if that happens and planning doesn't return a solution. Stop the script and run again. It may be a pybullet problem (happens only in Ubutun with python 3.8 but not MacOS with python 3.7)

Other test cases (to be uploaded soon):

<table class="multicol">

<tr>
<td width="50%">

<img src="gifs/220602-serve-plate.gif"></img>

</td>
<td width="50%">

<img src="gifs/220531-cook-only.gif"></img>

</td>
</tr>
<tr>
<td width="50%">

`test_feg_dishwasher` (currently unavailable)

</td>
<td width="50%">

`test_feg_cook_only` (currently unavailable)

</td>
</tr>
</table>

### Scene generation

To build some scenes.

```commandline
python test_world_builder.py
```

The default script creates 10 scenes with variations on the following properties of movable objects: (1) mesh model instances, (2) poses on their assigned surface or space.

## Acknowledgements

This repo works thanks for the tools provided by LIS lab members and alum:

* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is an awesome set of tools developed by Caelan Garret. A forked version is included with my own helper functions.
* the [pddlstream](https://github.com/caelan/pddlstream) is a planning framework developed by Caelan Garret.
* the [lisdf](https://github.com/Learning-and-Intelligent-Systems/lisdf) package is an input/output specification for TAMP problems developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao.

All the object models and urdf files are downloaded for free from the following sources:

* most articulated object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)
* most kitchen object models are downloaded from [Free3D](https://free3d.com/3d-models/food).


## TODO

- [x] add `requirements.txt`
- [x] upload test scene files in the format of `.lisdf` -> need lisdf team to support a few more tags, including
  ``````xml
  <include><uri>...</uri></include>
  <state>...</state>
  ``````
- [x] upload test scene files with `<world><gui><camera><pose>`
- [x] update problem files in the format of `problem.pddl`
- [x] upload implicit domain files in the format of `domain.pddl`
- [x] add instructions to solve the example problems with TAMP planner [PDDLStream](https://github.com/caelan/pddlstream/tree/main)

- [ ] run PDDLStream on one problem in each category, with the same domain, different scenes and goals
- [ ] run PDDLStream with execution noise
- [ ] run PDDLStream+HPN with execution noise
- [ ] run TTM with execution noise
- [ ] add instructions on sampling scenes
- [ ] add instructions on sampling goals
