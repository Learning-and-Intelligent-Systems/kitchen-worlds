# LISPDDL = LIS extended version to PDDL for describing TAMP problems

To enable the specification and parsing of continuous objects in PDDL, the LISPDDL format has a few extensions compared to [PDDL 1.2](https://planning.wiki/ref/pddl/problem):


## Test Parsing LISPDDL

You may test parse your problem pddl file with the following:

```
cd tests
python test_pybullet_lisdf -s path/to/lisdf -d path/to/domain -d path/to/problem

## for example
python test_parse_pddl -s path/to/lisdf -d path/to/domain -d path/to/problem
```

Note that it requires three input:

* A scene described in LISDF, so that the bodies and links referred as strings in the pddl files can be grounded in the simulator.
* A partial domain described in PDDL, where the operators are taken out.
