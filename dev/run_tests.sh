echo "\n... Test LISDF parser"
python test_parse_lisdf.py

echo "\n... Test load LISDF to Pybullet"
python test_pybullet_lisdf.py

echo "\n... Test parse problem.pddl"
python test_parse_pddl.py

echo "\n... Test solve PDDLStream problem"
python test_pddlstream.py

echo "\n... Test build scene"
python test_world_builder.py

