# kitchen-worlds

![example kitchen scene](media/move_egg.png)

A collection of kitchen scenes in LISDF format (an extension to SDF that includes URDF), kitchen object models, and TAMP problems. A product of LIS TAMP I/O Hackathon 03/25/2022.

## Setup

Initiate lisdf submodule:

```shell
git submodule init 
```

## Examples

To test parsing of LISDF files, run

```shell
python parser_test.py
```

To test loading LISDF worlds into PyBullet, run

```shell
python pybullet_test.py
```

## TODO

- [ ] upload test scene files in the format of `.lisdf` -> need lisdf team to resolve a few issues with parsing
- [ ] upload problem files in the format of `problem.pddl`
- [ ] add `requirements.txt`
- [ ] upload implicit domain files in the format of `domain.pddl`
- [ ] add instructions to solve the example problems with TAMP planner [PDDLStream](https://github.com/caelan/pddlstream/tree/main)
- [ ] add instructions on generating kitchen scenes with scripts in [cognitive-architectures](https://github.mit.edu/ztyang/cognitive-architectures/tree/master/bullet)

## Acknowledgements

In order to be lightweight, some functions in this repo is copied from the work of other LIS members instead of added as part of their original submodules:

* the [lisdf](https://github.com/Learning-and-Intelligent-Systems/lisdf) package is developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao
* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is a subset of tools developed by Caelan Garret
* most object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)