# kitchen-worlds

![kitchen_basics.lisdf in PyBullet](media/kitchen_basics.png)

A collection of kitchen scenes in LISDF format (an extension to SDF that includes URDF), kitchen object models, and TAMP problems. A product of LIS TAMP I/O Hackathon 03/25/2022.

## Setup

Grab the lisdf package, developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao

```shell
git submodule init
```

## Examples

The kitchen scenes are in path `asset/scenes/`.

To test parsing of LISDF files, run

```shell
python parser_test.py
```

To test loading LISDF worlds into PyBullet, run (expecting failure)

```shell
python pybullet_test.py
```

## LISDF Spec

Current kitchen scenes in the format of  `.lisdf` are in path `asset/scenes/`, e.g. `kitchen_counter.lisdf` as simulated below. The LISDF format has a few additions compared to [SDF](http://sdformat.org/spec?ver=1.9&elem=sdf):

* add actor and models with `<uri>URDF_file</uri>`

![kitchen_counter.lisdf in PyBullet](media/kitchen_counter.png)

## TODO

- [ ] upload test scene files in the format of `.lisdf` -> need lisdf team to support a few more tags, including

  ``````xml
  <world><actor>...</actor></world>
  <model><uri>...</uri></model>
  <state>...</state>
  ``````

- [ ] upload problem files in the format of `problem.pddl`

- [ ] add `requirements.txt`

- [ ] upload implicit domain files in the format of `domain.pddl`

- [ ] add instructions to solve the example problems with TAMP planner [PDDLStream](https://github.com/caelan/pddlstream/tree/main)

- [ ] add instructions on generating kitchen scenes with scripts in [cognitive-architectures](https://github.mit.edu/ztyang/cognitive-architectures/tree/master/bullet)

## Acknowledgements

In order to be lightweight, some functions in this repo is copied from the work of other LIS members instead of added as part of their original submodules:

* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is a subset of tools developed by Caelan Garret
* most object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)
