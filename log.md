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
TODO
-[ ] also clean in dishwasher
-[x] more objects
-[ ] specify initial state
-[ ] debug acronym grasps, using [test_skills.py](test_skills.py)