# Test LISDF + PDDL + Streams with PDDLStream

### Set up PDDLStream

```commandline
# git submodule add https://github.com/caelan/pddlstream.git
cd pddlstream
git submodule update --init --recursive  ## may take a few minutes
./downward/build.py ## may take a few minutes

cd examples/pybullet/utils
git pull origin master
cd motion
git pull origin master
```
