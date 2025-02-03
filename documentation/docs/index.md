# Kitchen Worlds

A library of long-horizon Task-and-Motion-Planning (TAMP) problems in simulated kitchen and household scenes, as well as planning algorithms to solve them

- Procedural scene, goal, and multi-step trajectory generation (follow sections A -> B2 -> C)
  - procedurally generate scenes with rigid and articulated objects
  - visualize a scene in LISDF format (an extension to SDF that includes URDF)
  - solve a TAMP problem defined by a `scene.lisdf` and `problem.pddl` using existing `domain.pddl` and `stream.pddl` using PDDLStream
  - visualize the robot trajectory in pybullet
- Use a pretrained VLM (e.g. GPT-4v) to guide the planning process (follow section A -> B1)

<!--
<video autoplay loop muted playsinline width=100%>
  <source src="mp4/demo-cabbage.mp4" type="video/mp4">
</video>
-->

<table class="multicol tightframes">
<tr>
<td width="52%">

<img src="gifs/rss23kitchens.gif"></img>

</td>
<td width="45%">

<img src="gifs/icra25vlmtamp.gif"></img>

</td>
</tr>
</table>

## Installation

1. Clone the repo along with the submodules. It may take a few minutes. 

```shell
git clone git@github.com:Learning-and-Intelligent-Systems/kitchen-worlds.git --recursive
```

<details close>
<summary>In case the submodules are not up-to-date, checkout the most recent changes:
</summary>

<pre>
(cd pybullet_planning; git checkout master; git pull); \
  (cd pddlstream; git checkout caelan/diverse); \
  (cd assets/models; git checkout main); 
</pre>

</details>

2. Install dependencies. It may take a dozen minutes.
<!-- 
Install [graphviz](https://pygraphviz.github.io/documentation/latest/install.html).
-->
```shell
conda env create -f environment.yml
conda activate kitchen
## sudo apt-get install graphviz graphviz-dev  ## on Ubuntu
```

3. Build FastDownward, the task planner used by PDDLStream planner.
<!--
It has pre-requisites of cmake, first download the source tar from [site](https://cmake.org/download/).

<details close>
<summary>Expand Instructions Code</summary>

<pre>
sudo apt-get install build-essential libssl-dev
cd ~/Downloads
tar -zxvf cmake-3.31.0-rc1.tar.gz
cd cmake-3.31.0-rc1
./bootstrap
make
sudo make install
cmake --version
</pre>

</details>
-->
    

```shell
## sudo apt install cmake g++ git make python3  ## if not already installed
(cd pddlstream; ./downward/build.py)
```

4. Build IK solvers (If using mobile manipulators; skip this if you're only using the floating gripper).

* (If on Ubuntu, this one is better) TracIK for whole-body IK that solves for base, torso, and arm together

    ```shell
    sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev swig
    pip install git+https://github.com/mjd3/tracikpy.git
    ```

* IKFast solver for arm planning (the default IK), which needs to be compiled for each robot type. Here's example for PR2:

    ```shell
    ## sudo apt-get install python-dev
    (cd pybullet_planning/pybullet_tools/ikfast/pr2; python setup.py)
    ```


### Test Installation

See [trouble-shooting.md](pybullet_planning/trouble-shooting.md) for common issues.

```shell 
conda activate kitchen
python examples/test_parse_lisdf.py
python examples/test_data_generation.py
```

---

## Tutorials

Here are some example scripts to help you understand the scene generation and task and motion planning tools. Once they are all working for you, we recommend you follow the next section to set up your own data generation pipeline with custom config files.

### 1. VLM-TAMP

See [vlm_tools/README.md](https://github.com/zt-yang/pybullet_planning/blob/master/vlm_tools/README.md) for more details.

### 2. Generate Worlds, Problems, and Plans

See [examples/tutorial_data_generation.md](examples/tutorial_data_generation.md) for more details.

---

## Tests

For developers, run all tests before merging to master:

```shell
python tests/1_test_data_generation.py
```
---

## References

Please cite one of the following papers if you use this code in your research:

```text 
@misc{yang2024guidinglonghorizontaskmotion,
      title={Guiding Long-Horizon Task and Motion Planning with Vision Language Models}, 
      author={Zhutian Yang and Caelan Garrett and Dieter Fox and Tomás Lozano-Pérez and Leslie Pack Kaelbling},
      year={2024},
      eprint={2410.02193},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2410.02193}, 
} 

@INPROCEEDINGS{yang2023piginet, 
    AUTHOR    = {Zhutian  Yang AND Caelan R Garrett AND Tomas Lozano-Perez AND Leslie Kaelbling AND Dieter Fox}, 
    TITLE     = {{Sequence-Based Plan Feasibility Prediction for Efficient Task and Motion Planning}}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2023}, 
    ADDRESS   = {Daegu, Republic of Korea}, 
    MONTH     = {July}, 
    DOI       = {10.15607/RSS.2023.XIX.061} 
} 
```

---

## Acknowledgements

The development is partially performed during internship at NVIDIA Research, Seattle Robotics Lab.

This repo works thanks for the tools provided by LIS lab members and alum:

* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is an awesome set of tools developed by Caelan Garret. A forked version is included with my own helper functions.
* the [pddlstream](https://github.com/caelan/pddlstream) is a planning framework developed by Caelan Garret.
* the [lisdf](https://github.com/Learning-and-Intelligent-Systems/lisdf) package is an input/output specification for TAMP problems developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao.

All the object models and urdf files are downloaded for free from the following sources:

* most articulated object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)
* most kitchen object models are downloaded from [Free3D](https://free3d.com/3d-models/food).

