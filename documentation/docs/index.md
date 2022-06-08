# Kitchen Worlds

A library of long-horizon Task-and-Motion-Planning (TAMP) problems in kitchen and household scenes, as well as planners to solve them

- visualize a scene in LISDF format (an extension to SDF that includes URDF)
- solve a TAMP problem using PDDLStream defined with a scene.lisdf, problem.pddl, domain.pddl, stream.pddl
- procedurally generate scenes with rigid and articulated objects

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

## Setup

Grab the submodules, may take a while

```shell
cd kitchen-worlds
git submodule update --init --recursive
```

Install the dependencies, in a virtual environment if you'd like

```shell
## pip install virtualenv  ## if you haven't install
python3 -m virtualenv venv/kitchen
source venv/kitchen/bin/activate

pip install -r requirements.txt
pip install scipy untangle
```


## Examples

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

To solve some test problems wih PDDLStream, `-test` takes the name of subdirectory inside `test_cases` folder, e.g. `blocks_pick`, `blocks_kitchen`, `kitchen`:

```commandline
python test_pddlstream.py -test test_pr2_kitchen
```

To build some scenes
```commandline
python test_world_builder.py
```

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
