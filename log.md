---
mobility.urdf
make sure object name has no space

---
add grasps to db
use test_skills to check

---
change top grasp tolerance

---
world.py self.constants add bowl
feg_kitchen_clean.pddl constants add bowl
NoDirtyPlateInCabinet (OfType ?o @bowl)

---
also clean in dishwasher
load_storage_mechanism(world, world.name_to_object('dishwasherbox'), epsilon=epsilon) does not work for dishwasher

---
counter is one piece

---
lisdf_loader.py make_furniture_transparent make cabinettop transparent

---
loaders.py  load_counter_moveables  force bottles to be in sink
counters["bottle"] = [world.name_to_object(n) for n in ["sink#1::sink_bottom"]]

---
We can add new acronym objects and grasps using [create_acronym_objects_and_grasps.py](create_acronym_objects_and_grasps.py)

---
use_doors usually needs to use diverse=True to find plans

---
instead of using counters returned, now I am using name_to_obj to retrieve counters. Not sure whether this will create
problems elsewhere
```python
    def place_on_counter(obj_name, counter_name, category=None, ins=True):
        # retrieve counter name
        counter = world.name_to_object(counter_name)
        obj = counter.place_new_obj(obj_name, category=category, RANDOM_INSTANCE=ins, world=world)
        if verbose:
            print(f'placed {obj} on {counter.name}')
        if 'bottom' not in counter.name:
            adjust_for_reachability(obj, counter, d_x_min, world=world)
        return obj
```

---
changed /Sink/00005/mobility.urdf scale in `world_builder/partnet_scales.py`

---
do you need to use all three in sampled goals? NoDirtyBowlInCabinet, NoDirtyPanInCabinet, NoDirtyMugInCabinet?
any axiom with (not (exists)) will make FastDownward slows down drastically, it's ok to have one for different types, but you may not need to have all three of them in the domain for each problem if the problem doesn't require it
the more specific the definition, the less it burdens the planner because it has t enumerate all facts that satisfy it in future possible states
Then I will use separate domain files
---
TODO
-[ ] also clean in dishwasher
-[x] more objects
-[ ] specify initial state
-[ ] debug acronym grasps, using [test_skills.py](test_skills.py)