# kitchen-worlds

![kitchen_basics.lisdf in PyBullet](media/kitchen_basics.png)

A collection of kitchen scenes in LISDF format (an extension to SDF that includes URDF), kitchen object models, and TAMP problems. A product of LIS TAMP I/O Hackathon 03/25/2022.

## Setup

Grab the [lisdf](https://github.com/Learning-and-Intelligent-Systems/lisdf) package, developed by William Shen, Nishanth Kumar, Aidan Curtis, and Jiayuan Mao. Follow these steps:

```shell
cd kitchen-worlds
git submodule init
cd lisdf
git pull
pip install -r requirements.txt
```

## Examples

The kitchen scenes are in path `asset/scenes/`.

To test parsing of LISDF files, run

```shell
python parse_lisdf.py
```

To test loading LISDF worlds into PyBullet, run

```shell
python pybullet_lisdf.py
```

## LISDF Spec

Current kitchen scenes in the format of  `.lisdf` are in path `asset/scenes/`, e.g. `kitchen_counter.lisdf` as simulated below. The LISDF format has a few additions compared to [SDF](http://sdformat.org/spec?ver=1.9&elem=sdf):

* add object and robot models with 
  ```xml
  <include>
      <uri>URDF_file</uri>
      <static>true/false</static>
      <pose>x y z r p y</pose>
      <scale> an integer to scale model .obj with original ratio</scale>
  </include>
  ```

![kitchen_counter.lisdf in PyBullet](media/kitchen_counter.png)

## TODO

- [x] add `requirements.txt`
- [x] upload test scene files in the format of `.lisdf` -> need lisdf team to support a few more tags, including
  ``````xml
  <include><uri>...</uri></include>
  <state>...</state>
  ``````
- [ ] upload test scene files with `<world><gui><camera><pose>`

- [ ] update problem files in the format of `problem.pddl`

- [ ] upload implicit domain files in the format of `domain.pddl`

- [ ] add instructions to solve the example problems with TAMP planner [PDDLStream](https://github.com/caelan/pddlstream/tree/main)

- [ ] add instructions on generating kitchen scenes with scripts in [cognitive-architectures](https://github.mit.edu/ztyang/cognitive-architectures/tree/master/bullet)

## Acknowledgements

In order to be lightweight, some functions in this repo is copied from the work of other LIS members instead of added as part of their original submodules:

* the [pybullet_tools](https://github.com/caelan/pybullet-planning/tree/master/pybullet_tools) package is a subset of tools developed by Caelan Garret
* most object models are downloaded from [PartNet Mobility dataset](https://sapien.ucsd.edu/browse) (Mo, Kaichun, et al. "Partnet: A large-scale benchmark for fine-grained and hierarchical part-level 3d object understanding." *Proceedings of the IEEE/CVF conference on computer vision and pattern recognition*. 2019.)
