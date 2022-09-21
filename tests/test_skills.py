#!/usr/bin/env python

from __future__ import print_function
import os
import sys
import random
import json
import shutil
import time

import numpy as np
from os.path import join, abspath, dirname, isdir, isfile
from os import listdir
from config import EXP_PATH

from pybullet_planning.pybullet_tools.pr2_utils import get_group_conf
from pybullet_planning.pybullet_tools.pr2_primitives import get_base_custom_limits, control_commands, apply_commands
from pybullet_planning.pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, get_pose, approximate_as_prism, draw_aabb, multiply, unit_quat, remove_body, invert, \
    Pose, get_link_pose, get_joint_limits, WHITE, RGBA, set_all_color, RED, GREEN, set_renderer, clone_body, \
    add_text
from pybullet_planning.pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, set_camera_target_body, \
    draw_bounding_lines, fit_dimensions, draw_fitted_box, get_hand_grasps, get_partnet_doors, get_partnet_spaces, \
    open_joint, get_instance_name
from pybullet_planning.pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    visualize_grasps_by_quat, visualize_grasps
from pybullet_planning.pybullet_tools.logging import TXT_FILE

## custom stream_map
from pybullet_planning.pybullet_tools.general_streams import get_grasp_list_gen, get_contain_list_gen
from pybullet_planning.pybullet_tools.pr2_streams import get_stable_gen, \
    Position, get_handle_grasp_gen
from pybullet_planning.pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands
from pybullet_planning.pybullet_tools.flying_gripper_utils import se3_from_pose, \
    pose_from_se3, se3_ik, set_cloned_se3_conf

from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test
from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.world import State
from world_builder.loaders import create_gripper_robot, create_pr2_robot
from world_builder.utils import load_asset
from world_builder.utils import get_instances as get_instances_helper
# from pybullet_planning.world_builder.colors import *
from world_builder.partnet_scales import MODEL_SCALES as TEST_MODELS
from world_builder.partnet_scales import MODEL_HEIGHTS, OBJ_SCALES

from test_pddlstream import get_args

from pybullet_tools.pr2_problems import create_floor
from pybullet_tools.utils import connect, draw_pose, unit_pose, link_from_name, load_pybullet, load_model, \
    sample_aabb, AABB, set_pose, get_aabb, get_aabb_center, quat_from_euler, Euler, HideOutput, get_aabb_extent, \
    set_camera_pose
from pybullet_tools.flying_gripper_utils import create_fe_gripper, set_se3_conf
import math


DEFAULT_TEST = 'kitchen' ## 'blocks_pick'
ASSET_PATH = join('..', 'assets')

# ####################################

def get_instances(category):
    instances = get_instances_helper(category)
    keys = list(instances.keys())
    if not keys[0].isdigit():
        keys = list(set([k.lower() for k in keys]))
        instances = {k: instances[k] for k in keys}
    return instances


def get_test_world(robot='feg', semantic_world=False, DRAW_BASE_LIMITS=False):
    args = get_args() ## exp_name
    connect(use_gui=True, shadows=False, width=1980, height=1238)  ##  , width=360, height=270
    draw_pose(unit_pose(), length=.5)
    # create_floor()
    if semantic_world:
        from world_builder.world import World
        world = World(args)
    else:
        from lisdf_tools.lisdf_loader import World
        world = World(args)
    add_robot(world, robot, DRAW_BASE_LIMITS=DRAW_BASE_LIMITS)
    return world


def add_robot(world, robot, **kwargs):
    if robot == 'pr2':
        from world_builder.loaders import BASE_LIMITS as custom_limits
        base_q = [0, -0.5, 0]
        robot = create_pr2_robot(world, custom_limits=custom_limits,
                                 base_q=base_q, **kwargs)

    elif robot == 'feg':
        custom_limits = {0: (-5, 5), 1: (-5, 5), 2: (0, 3)}
        # init_q = [3, 1, 1, 0, 0, 0]
        init_q = [0, 0, 0, 0, 0, 0]
        # robot = create_fe_gripper(init_q=init_q)
        # world.add_robot(robot, 'feg')
        robot = create_gripper_robot(world, custom_limits=custom_limits,
                                     initial_q=init_q, **kwargs)

    return robot


def get_z_on_floor(body):
    return get_aabb_extent(get_aabb(body))[-1]/2


def get_floor_aabb(custom_limits):
    x_min, x_max = custom_limits[0]
    y_min, y_max = custom_limits[1]
    return AABB(lower=(x_min, y_min), upper=(x_max, y_max))


def sample_pose_on_floor(body, custom_limits):
    x, y = sample_aabb(get_floor_aabb(custom_limits))
    z = get_z_on_floor(body)
    return ((x, y, z), quat_from_euler((0, 0, math.pi)))


def pose_from_2d(body, xy, random_yaw=False):
    z = get_z_on_floor(body)
    yaw = math.pi ## facing +x axis
    if random_yaw:
        yaw = random.uniform(-math.pi, math.pi)
    return ((xy[0], xy[1], z), quat_from_euler((0, 0, yaw)))

# ####################################


def test_robot_rotation(body, robot):
    pose = ((0.2,0.3,0), quat_from_euler((math.pi/4, math.pi/2, 1.2)))
    set_pose(body, pose)
    conf = se3_ik(robot, pose)
    set_se3_conf(robot, conf)
    set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)


def test_spatial_algebra(body, robot):

    ## transformations
    O_T_G = ((0.5, 0, 0), unit_quat())
    O_R_G = ((0, 0, 0), quat_from_euler((0, -math.pi / 2, 0)))
    G = multiply(O_T_G, O_R_G)
    gripper = robot.create_gripper(color=RED)

    ## original object pose
    set_pose(body, unit_pose())
    set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)
    W_X_G = multiply(get_pose(body), G)
    set_pose(gripper, W_X_G)
    set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)

    ## new object pose given rotation
    # object_pose = ((0.4, 0.3, 0), quat_from_euler((-1.2, 0.3, 0)))
    object_pose = ((0, 0, 0), quat_from_euler((-1.2, 0, 0)))
    set_pose(body, object_pose)
    W_X_G = multiply(get_pose(body), G)
    draw_pose(W_X_G, length=0.3)
    set_pose(gripper, W_X_G)
    set_camera_target_body(gripper, dx=0.5, dy=0, dz=0.5)


def get_gap(category):
    gap = 1
    if category == 'MiniFridge':
        gap = 2
    if category == 'Food':
        gap = 0.5
    return gap


def test_grasps(categories=[], robot='feg'):
    world = get_test_world(robot)
    robot = world.robot

    problem = State(world, grasp_types=robot.grasp_types)  ## , 'side' , 'top'
    funk = get_grasp_list_gen(problem, collisions=True, visualize=False, RETAIN_ALL=False)

    i = -1
    for cat in categories:
        i += 1
        instances = get_instances(cat)
        n = len(instances)
        locations = [(i, get_gap(cat) * n) for n in range(1, n+1)]
        j = -1
        for id, scale in instances.items():
            j += 1
            path, body, _ = load_model_instance(cat, id, scale=scale, location=locations[j])
        # for id, scale in TEST_MODELS[cat].items():
        #     j += 1
        #     path = join(ASSET_PATH, 'models', cat, id)
        #     body = load_body(path, scale, locations[j], random_yaw=True)
            instance_name = get_instance_name(abspath(path))
            world.add_body(body, f'{cat.lower()}#{id}', instance_name)
            set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)
            text = id.replace('veggie', '').replace('meat', '')
            draw_text_label(body, text, offset=(0, -0.2, 0.1))

            """ test others """
            # test_robot_rotation(body, world.robot)
            # test_spatial_algebra(body, world.robot)
            # draw_fitted_box(body, draw_centroid=True)
            # grasps = get_hand_grasps(problem, body)

            """ test grasps """
            # set_renderer(True)
            # body_pose = get_pose(body)  ## multiply(get_pose(body), Pose(euler=Euler(math.pi/2, 0, -math.pi/2)))
            # outputs = funk(body)
            # print(f'grasps on body {body}:', outputs)
            # # set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.8)
            # visualize_grasps(problem, outputs, body_pose, RETAIN_ALL=True)
            # set_renderer(True)

        if len(categories) > 1:
            wait_if_gui(f'------------- Next object category? finished ({i+1}/{len(categories)})')

        if cat == 'MiniFridge':
            set_camera_pose((3, 7, 2), (0, 7, 1))
        elif cat == 'Food':
            set_camera_pose((3, 3, 2), (0, 3, 1))

    # set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)
    set_renderer(True)
    wait_if_gui('Finish?')
    disconnect()


def load_body(path, scale, pose_2d=(0,0), random_yaw=False):
    file = join(path, 'mobility.urdf')
    print('loading', file)
    with HideOutput(True):
        body = load_model(file, scale=scale)
        if isinstance(body, tuple): body = body[0]
    pose = pose_from_2d(body, pose_2d, random_yaw=random_yaw)
    set_pose(body, pose)
    return body, file


def load_model_instance(category, id, scale=1, location = (0, 0)):
    from world_builder.utils import get_model_scale

    models_path = join(ASSET_PATH, 'models')
    category = [c for c in listdir(models_path) if c.lower() == category.lower()][0]
    if not id.isdigit():
        id = [i for i in listdir(join(models_path, category)) if i.lower() == id.lower()][0]
    path = join(models_path, category, id)

    if category in MODEL_HEIGHTS:
        height = MODEL_HEIGHTS[category]['height']
        scale = get_model_scale(path, h=height)
    elif category in TEST_MODELS:
        scale = TEST_MODELS[category][id]

    body, file = load_body(path, scale, location)
    return file, body, scale


def draw_text_label(body, text, offset=(0, -0.05, .5)):
    lower, upper = get_aabb(body)
    position = ((lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2, upper[2])
    position = [position[i] + offset[i] for i in range(len(position))]
    add_text(text, position=position, color=(1, 0, 0), lifetime=0)


def test_handle_grasps(robot, category):
    from pybullet_tools.pr2_streams import get_handle_pose

    world = get_test_world(robot, DRAW_BASE_LIMITS=False)
    problem = State(world)
    funk = get_handle_grasp_gen(problem, visualize=False)

    ## load fridge
    instances = get_instances(category)
    n = len(instances)
    i = 0
    locations = [(0, 2*n) for n in range(1, n+1)]
    set_camera_pose((4, 3, 2), (0, 3, 0.5))
    for id in instances:
        path, body, _ = load_model_instance(category, id, location=locations[i])
        instance_name = get_instance_name(path)
        world.add_body(body, f'{category.lower()}#{id}', instance_name)
        set_camera_target_body(body, dx=1, dy=1, dz=1)
        draw_text_label(body, id)

        # ## color links corresponding to semantic labels
        # body_joints = get_partnet_doors(path, body)
        # world.add_joints(body, body_joints)
        #
        # for body_joint in body_joints:
        #     outputs = funk(body_joint)
        #     body_pose = get_handle_pose(body_joint)
        #
        #     set_renderer(True)
        #     set_camera_target_body(body, dx=2, dy=1, dz=1)
        #     visualize_grasps(problem, outputs, body_pose, RETAIN_ALL=True)
        #     set_camera_target_body(body, dx=2, dy=1, dz=1)
        i += 1

    set_camera_pose((8, 8, 2), (0, 8, 1))
    wait_if_gui('Finish?')
    disconnect()


def reload_after_vhacd(path, body, scale, id=None):
    from pybullet_tools.srl_stream_utils import process_urdf, TEMP_URDF_DIR

    pose = get_pose(body)
    remove_body(body)
    new_urdf_path = process_urdf(path)
    id_urdf_path = join(TEMP_URDF_DIR, f"{id}.urdf")
    os.rename(new_urdf_path, id_urdf_path)
    body = load_pybullet(id_urdf_path, scale=scale)
    set_pose(body, pose)
    return id_urdf_path, body


def get_contain_list_gen(problem, num_samples=10, **kwargs):
    from pybullet_tools.general_streams import get_contain_list_gen as gen_helper
    funk = gen_helper(problem, num_samples=num_samples, **kwargs)

    def contain_list_gen(body, container):
        g = funk(body, container)
        poses = []
        while len(poses) < num_samples:
            try:
                pose = next(g)
                poses.append(pose)
            except StopIteration:
                break
        return poses
    return contain_list_gen


def test_placement_in(robot, category):
    from pybullet_tools.pr2_streams import get_handle_pose

    world = get_test_world(robot)
    problem = State(world)
    funk = get_contain_list_gen(problem, collisions=True, verbose=False,
                                force_storage=True, num_samples=60)

    ## load fridge
    instances = get_instances(category)
    n = len(instances)
    i = 0
    locations = [(0, 2 * n) for n in range(1, n + 1)]
    set_camera_pose((4, 3, 2), (0, 3, 0.5))
    for id in instances:
        if id not in ['11709']:
            continue
        (x, y) = locations[i]
        path, body, scale = load_model_instance(category, id, location=(x, y))
        # new_urdf_path, body = reload_after_vhacd(path, body, scale, id=id)
        instance_name = get_instance_name(path)
        name = f'{category.lower()}#{id}'
        if category in ['MiniFridge', 'Fridge', 'Cabinet', 'Microwave']:
            name += '_storage'
        world.add_body(body, name, instance_name)
        set_camera_target_body(body, dx=1, dy=0, dz=1)

        ## color links corresponding to semantic labels
        spaces = get_partnet_spaces(path, body)
        world.add_spaces(body, spaces)

        for door in get_partnet_doors(path, body):
            open_joint(door[0], door[1])

        for body_link in spaces:
            x += 1
            # space = clone_body(body, links=body_link[-1:], visual=True, collision=True)
            # world.add_body(space, f'{category.lower()}#{id}-{body_link}')

            cabbage = load_asset('VeggieCabbage', x=x, y=y, z=0, yaw=0)[0]
            cabbage_name = f'cabbage#{i}-{body_link}'
            world.add_body(cabbage, cabbage_name)

            outputs = funk(cabbage, body_link)
            set_pose(cabbage, outputs[0][0].value)
            for i in range(1, len(outputs)):
                marker = load_asset('VeggieCabbage', x=x, y=y, z=0, yaw=0)[0]
                world.add_body(cabbage, cabbage_name+f"_({i})")
                set_pose(marker, outputs[i][0].value)

            set_renderer(True)
            set_camera_target_body(cabbage, dx=1, dy=0, dz=1)
        i += 1

    # set_camera_pose((4, 3, 2), (0, 3, 0.5))
    wait_if_gui('Finish?')
    disconnect()


def test_gripper_joints():
    """ visualize ee link pose as conf changes """
    world = get_feg_world()
    robot = world.robot

    set_se3_conf(robot, (0, 0, 0, 0, 0, 0))
    set_camera_target_body(robot, dx=0.5, dy=0.5, dz=0.5)
    for j in range(3, 6):
        limits = get_joint_limits(robot, j)
        values = np.linspace(limits[0], limits[1], num=36)
        for v in values:
            conf = [0, 0, 0, 0, math.pi/2, 0]
            conf[j] = v
            set_se3_conf(robot, conf)
            set_camera_target_body(robot, dx=0.5, dy=0.5, dz=0.5)
            time.sleep(0.1)

    wait_if_gui('Finish?')
    disconnect()


def test_gripper_range(IK=False):
    """ visualize all possible gripper orientation """
    world = get_feg_world()
    robot = world.robot

    set_se3_conf(robot, (0, 0, 0, 0, 0, 0))
    set_camera_target_body(robot, dx=0.5, dy=0.5, dz=0.5)
    choices = np.linspace(-math.pi, math.pi, num=9)[:-1]
    bad = [choices[1], choices[3]]
    mi, ma = min(choices), max(choices)
    ra = ma - mi
    def get_color(i, j, k):
        color = RGBA((i-mi)/ra, (j-mi)/ra, (k-mi)/ra, 1)
        return color
    def mynice(tup):
        tup = nice(tup)
        if len(tup) == 2:
            return tup[-1]
        return tuple(list(tup)[-3:])
    for i in choices:
        for j in choices:
            for k in choices:
                if IK:
                    gripper = create_fe_gripper(init_q=[0, 0, 0, 0, 0, 0], POINTER=True)
                    pose = ((0,0,0), quat_from_euler((i,j,k)))
                    conf = se3_ik(robot, pose)
                    if conf == None:
                        remove_body(gripper)
                        print('failed IK at', nice(pose))
                        continue
                    else:
                        print('pose =', mynice(pose), '-->\t conf =', mynice(conf))
                        set_se3_conf(gripper, conf)
                        set_all_color(gripper, WHITE)
                        # if j in bad:
                        #     set_all_color(gripper, RED)
                        # else:
                        #     set_all_color(gripper, GREEN)
                else:
                    conf = [0, 0, 0, i, j, k]
                    gripper = create_fe_gripper(init_q=conf, POINTER=True)
                    set_all_color(gripper, WHITE)
                    pose = get_link_pose(gripper, link_from_name(gripper, 'panda_hand'))
                    print('conf =', mynice(conf), '-->\t pose =', mynice(pose))

                    # set_all_color(gripper, get_color(i,j,k))
            set_camera_target_body(robot, dx=0.5, dy=0.5, dz=0.5)
    set_camera_target_body(robot, dx=0.5, dy=0.5, dz=0.5)

    wait_if_gui('Finish?')
    disconnect()


def test_handle_grasps_counter():
    from world_builder.loaders import load_floor_plan
    from world_builder.world import World

    args = get_args()
    connect(use_gui=True, shadows=False, width=1980, height=1238)
    draw_pose(unit_pose(), length=2.)

    # lisdf_path = join(ASSET_PATH, 'scenes', f'kitchen_lunch.lisdf')
    # world = load_lisdf_pybullet(lisdf_path, verbose=True)

    world = World(args)
    floor = load_floor_plan(world, plan_name='counter.svg')
    robot = add_robot(world, 'feg', DRAW_BASE_LIMITS=False)

    world.summarize_all_objects()
    state = State(world, grasp_types=robot.grasp_types)
    joints = world.cat_to_bodies('door')
    joints = [(6, 1)]

    for body_joint in joints:
        obj = world.BODY_TO_OBJECT[body_joint]
        link = obj.handle_link
        body, joint = body_joint
        set_camera_target_body(body, link=link, dx=0.5, dy=0.5, dz=0.5)
        draw_fitted_box(body, link=link, draw_centroid=True)
        grasps = get_hand_grasps(state, body, link=link, visualize=False,
                                 RETAIN_ALL=True, HANDLE_FILTER=True, LENGTH_VARIANTS=True)
        set_camera_target_body(body, link=link, dx=0.5, dy=0.5, dz=0.5)

    wait_if_gui('Finish?')
    disconnect()


def test_placement_counter():
    from world_builder.loaders import load_floor_plan
    from world_builder.world import World

    args = get_args()
    connect(use_gui=True, shadows=False, width=1980, height=1238)
    draw_pose(unit_pose(), length=2.)

    surfaces = {
        'counter': {
            'back_left_stove': [],
            'back_right_stove': [],
            'front_left_stove': [],
            'front_right_stove': [],
            'hitman_tmp': [],
            'indigo_tmp': [],
        }
    }
    spaces = {
        'counter': {
            'sektion': [],
            'dagger': [],
            'hitman_drawer_top': [],
            # 'hitman_drawer_bottom': [],
            'indigo_drawer_top': [],
            # 'indigo_drawer_bottom': [],
            'indigo_tmp': []
        },
    }

    world = World(args)
    floor = load_floor_plan(world, plan_name='counter.svg', surfaces=surfaces, spaces=spaces)
    robot = add_robot(world, 'feg')

    world.open_all_doors_drawers()
    world.summarize_all_objects()
    state = State(world, grasp_types=robot.grasp_types)
    funk = get_grasp_list_gen(state, collisions=True, visualize=False, RETAIN_ALL=False)

    surfaces = world.cat_to_bodies('surface')
    spaces = world.cat_to_bodies('space')
    regions = spaces
    opened_poses = {}

    for rg in regions:
        r = world.BODY_TO_OBJECT[rg]
        draw_aabb(get_aabb(r.body, link=r.link))
        opened_poses[rg] = get_link_pose(r.body, r.link)

        # if rg in surfaces:
        #     body = r.place_new_obj('OilBottle')
        #     draw_fitted_box(body, draw_centroid=False)
        #     set_camera_target_body(body, dx=0.5, dy=0, dz=0)
        #     set_camera_target_body(body, dx=0.5, dy=0, dz=0)
        #     grasps = get_hand_grasps(state, body, visualize=True, RETAIN_ALL=True)

        if rg in spaces:
            body = r.place_new_obj('MeatTurkeyLeg')
            draw_fitted_box(body, draw_centroid=False)
            set_camera_target_body(body, dx=0.1, dy=0, dz=0.5)
            set_camera_target_body(body, dx=0.1, dy=0, dz=0.5)
            # grasps = get_hand_grasps(state, body, visualize=False, RETAIN_ALL=False)

            outputs = funk(body)
            print(f'grasps on body {body}:', outputs)
            visualize_grasps(state, outputs, get_pose(body), RETAIN_ALL=True)

        set_renderer(True)
        print(f'test_placement_counter | placed {body} on {r}')
    wait_if_gui('Finish?')
    disconnect()


def get_data(category):
    from world_builder.paths import PARTNET_PATH

    models = get_instances(category)

    target_model_path = join(ASSET_PATH, 'models', category)
    if not isdir(target_model_path):
        os.mkdir(target_model_path)

    if isdir(PARTNET_PATH):
        for idx in models:
            old_path = join(PARTNET_PATH, idx)
            new_path = join(target_model_path, idx)
            if isdir(old_path) and not isdir(new_path):
                shutil.copytree(old_path, new_path)
                print(f'copying {old_path} to {new_path}')


def test_texture(category, id):
    import untangle
    connect(use_gui=True, shadows=False, width=1980, height=1238)
    path = join(ASSET_PATH, 'models', category, id) ## , 'mobility.urdf'

    body = load_body(path, 0.2)
    set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)
    set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.5)

    # content = untangle.parse(path).robot
    #
    # import xml.etree.ElementTree as gfg
    # root = gfg.Element("robot")
    # tree = gfg.ElementTree(content)
    # with open(path.replace('mobility', 'mobility_2'), "wb") as files:
    #     tree.write(files)


def test_pick_place_counter(robot):
    from world_builder.loaders import load_random_mini_kitchen_counter
    world = get_test_world(robot, semantic_world=True)
    load_random_mini_kitchen_counter(world)


def test_vhacd():

    urdf_path = '../assets/models/MiniFridge/11709/mobility.urdf'

    world = get_test_world()
    body = load_pybullet(urdf_path)
    set_pose(body, ((0, 0, 0), unit_quat()))

    new_urdf_path = process_urdf(urdf_path)
    body = load_pybullet(new_urdf_path)
    set_pose(body, ((0, 2, 0), unit_quat()))

    set_camera_target_body(body, dx=1, dy=1, dz=1)
    print()


if __name__ == '__main__':

    ## --- MODELS  ---
    # get_data(category='MiniFridge')
    # test_texture(category='CoffeeMachine', id='103127')
    # test_vhacd()

    ## --- robot (FEGripper) related  ---
    # test_gripper_joints()
    # test_gripper_range()


    ## --- grasps related ---
    robot = 'pr2' ## 'feg' ##
    # test_grasps(['MiniFridge'], robot)  ## 'Bottle', 'Stapler', 'Camera', 'Glasses', 'Food'
    # test_handle_grasps_counter()
    # test_handle_grasps(robot, category='MiniFridge')
    # test_pick_place_counter(robot)


    ## --- placement related  ---
    # test_placement_counter()
    test_placement_in(robot, category='MiniFridge')
