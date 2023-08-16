#!/usr/bin/env python

from __future__ import print_function
import shutil
import pickle
import os
import time
import random
import copy
import json
from os.path import join, abspath, dirname, isdir, isfile, basename
from config import EXP_PATH, OUTPUT_PATH

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, has_gui, reset_simulation, set_random_seed, \
    set_numpy_seed, set_renderer
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime
from pybullet_tools.pr2_agent import solve_multiple, post_process, pddlstream_from_state_goal, \
    create_cwd_saver, solve_one
from pybullet_tools.pr2_primitives import control_commands, apply_commands
from pybullet_tools.logging import parallel_print, myprint

from lisdf_tools.lisdf_loader import pddl_files_from_dir

from world_builder.world import State
from world_builder.actions import apply_actions
from world_builder.world_generator import save_to_outputs_folder

from test_world_builder import create_pybullet_world

# additional imports
import math
import argparse
from world_builder.builders import initialize_pybullet
from world_builder.utils import get_domain_constants, parse_yaml, load_asset
from world_builder.world import World
from world_builder.robot_builders import get_robot_builder, create_gripper_robot, create_pr2_robot
from pybullet_tools.utils import set_all_color, GREEN, RED, HideOutput, get_aabb_extent, set_pose, quat_from_euler, get_pose, remove_body, dump_world, Pose, Euler, invert, multiply, draw_pose
from pybullet_tools.bullet_utils import set_camera_target_body
from pybullet_tools.flying_gripper_utils import set_se3_conf, get_se3_conf
from pybullet_tools.general_streams import get_grasp_list_gen, get_grasp_gen
from world_builder.entities import Moveable

from config import ASSET_PATH
from world_builder.utils import get_instances as get_instances_helper, load_model, get_instance_name, add_text
from os import listdir
from world_builder.partnet_scales import MODEL_HEIGHTS, OBJ_SCALES, MODEL_SCALES

#####################################


def get_config(config_name):
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default=config_name)
    args = parser.parse_args()
    config = parse_yaml(args.config)
    return config


def get_instances(category, **kwargs):
    cat_dir = join(ASSET_PATH, 'models', category)
    if not isdir(cat_dir):
        os.mkdir(cat_dir)
        get_data(categories=[category])
    instances = get_instances_helper(category, **kwargs)
    return instances


def get_data(categories):
    from world_builder.paths import PARTNET_PATH

    for category in categories:
        models = get_instances_helper(category)

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


def get_model_path(category, id):
    models_path = join(ASSET_PATH, 'models')
    category = [c for c in listdir(models_path) if c.lower() == category.lower()][0]
    if not id.isdigit():
        id = [i for i in listdir(join(models_path, category)) if i.lower() == id.lower()][0]
    path = join(models_path, category, id)
    return path


def load_model_instance(category, id, scale=1, location = (0, 0)):
    from world_builder.utils import get_model_scale

    path = get_model_path(category, id)
    if category in MODEL_HEIGHTS:
        height = MODEL_HEIGHTS[category]['height']
        scale = get_model_scale(path, h=height)
    elif category in MODEL_SCALES:
        scale = MODEL_SCALES[category][id]

    body, file = load_body(path, scale, location)
    return file, body, scale


def load_body(path, scale, pose_2d=(0, 0), random_yaw=False):
    file = join(path, 'mobility.urdf')
    # if 'MiniFridge' in file:
    #     file = file[file.index('../')+2:]
    #     file = '/home/yang/Documents/cognitive-architectures/bullet' + file
    print('loading', file)
    with HideOutput(True):
        body = load_model(file, scale=scale)
        if isinstance(body, tuple): body = body[0]
    pose = pose_from_2d(body, pose_2d, random_yaw=random_yaw)
    # pose = (pose[0], unit_quat())
    set_pose(body, pose)
    return body, file

def pose_from_2d(body, xy, random_yaw=False):
    z = get_z_on_floor(body)
    yaw = math.pi ## facing +x axis
    if random_yaw:
        yaw = random.uniform(-math.pi, math.pi)
    return ((xy[0], xy[1], z), quat_from_euler((0, 0, yaw)))

def get_z_on_floor(body):
    return get_aabb_extent(get_aabb(body))[-1]/2


def draw_text_label(body, text, offset=(0, -0.05, .5)):
    lower, upper = get_aabb(body)
    position = ((lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2, upper[2])
    position = [position[i] + offset[i] for i in range(len(position))]
    add_text(text, position=position, color=(1, 0, 0), lifetime=0)

def main():

    DEFAULT_YAML = "/home/weiyu/Research/nsplan/private_copy/kitchen-worlds/configs/test/feg_grasp.yaml"
    # DEFAULT_YAML = "/home/weiyu/Research/nsplan/original/kitchen-worlds/configs/test/pr2.yaml"
    config = get_config(DEFAULT_YAML)

    """ ============== initiate simulator ==================== """
    initialize_pybullet(config)
    constants = get_domain_constants(config.planner.domain_pddl)
    world = World(time_step=config.time_step, camera=config.camera, segment=config.segment,
                  constants=constants)

    robot_builder = get_robot_builder(config.robot.builder_name)
    robot = robot_builder(world, config.robot.robot_name, **config.robot.builder_kwargs)

    if config.robot.robot_name == "feg":
        print(robot.get_gripper_joints())
        print(robot.get_positions())
        set_all_color(robot, RED)
        print(get_se3_conf(robot))
        # first three numbers are xyz, last three are rotations
        set_se3_conf(robot, (1.0, 1.0, 1.0, 1.57, -1.57, 0.0))

    # option 1
    # cat = "Bottle"
    # instances = get_instances(cat)
    # instance_id, instance_scale = list(instances.items())[0]
    # path, body, _ = load_model_instance(cat, instance_id, scale=instance_scale)
    # instance_name = get_instance_name(abspath(path))
    # obj_name = f'{cat.lower()}#{id}'
    # # world.add_body(body, obj_name, instance_name)
    # set_camera_target_body(body)
    # text = instance_id.replace('veggie', '').replace('meat', '')
    # draw_text_label(body, text, offset=(0, -0.2, 0.1))

    # option 2
    obj_name = "Mug"
    category = obj_name
    obj = world.add_object(Moveable(load_asset(obj_name.lower(), RANDOM_INSTANCE=False, scale=1.0), category=category))
    body = obj.body

    dump_world()

    body_pose = get_pose(body)

    # #---------------------------------------------------------------
    # # g = (t, quat)
    # # g = [-0.002, 0.1678, 0.0829, 2.4213, -0.2838, 0.2167]  # for pan
    # # g = [0.1019, -0.0606, 0.1084, -2.7531, 0.418, 0.6877]  # for bowl
    # # ((-0.002, 0.1678, 0.0829), (0.9263959560351546, 0.05063616383573751, 0.16930116411850543, 0.33251109362219955))
    #
    # # debug: why do we need to do this?
    # r = Pose(euler=Euler(math.pi / 2, 0, -math.pi / 2))
    # body_pose = multiply(body_pose, invert(r))
    #
    # instance_name = world.get_instance_name(obj.name)
    # print(instance_name)
    # if instance_name == "Bowl_8eab5598b81afd7bab5b523beb03efcd_S":
    #     gs = [[0.1019, -0.0606, 0.1084, -2.7531, 0.418, 0.6877],
    #           [-0.0249, 0.0539, 0.1179, 2.9819, -0.0682, -2.3769],
    #           [0.079, 0.0842, 0.1005, -2.684, -0.2772, 2.4804],
    #           [0.0208, -0.113, 0.1134, -2.7269, 0.1193, 0.0059],
    #           [0.0487, 0.0695, 0.109, 3.0231, 0.09, -0.2725],
    #           [-0.0057, -0.0715, 0.117, -3.006, -0.2409, 2.4122],
    #           [-0.1177, 0.0256, 0.1121, -2.6601, 0.0645, -1.8323],
    #           [0.0918, 0.0165, 0.0988, 2.8828, 0.0647, -1.0832],
    #           [0.0433, -0.1069, 0.1045, -2.7995, -0.3778, 0.8176],
    #           [-0.0452, -0.0207, 0.1129, 2.7637, -0.2926, -0.4089],
    #           [-0.0631, -0.1207, 0.0789, 2.4132, -0.4492, 2.554],
    #           [-0.0488, -0.1413, 0.0926, 2.3212, 0.2521, 3.0302],
    #           [0.0992, -0.0308, 0.1106, 2.8168, -0.0459, -1.8491],
    #           [-0.1085, -0.0687, 0.1053, -2.6121, 0.4471, -1.2544],
    #           [-0.1424, -0.0065, 0.0671, -2.3578, 0.5776, -1.7239],
    #           [0.0533, 0.1419, 0.0914, 2.3109, 0.3193, -0.0698],
    #           [-0.0322, -0.1015, 0.1106, 2.9449, -0.4867, 2.3275],
    #           [0.0725, -0.0233, 0.1167, 3.1079, -0.0112, -2.2106],
    #           [0.0739, -0.0129, 0.1285, 3.131, -0.0007, -1.7106],
    #           [0.0424, -0.0673, 0.1231, 3.0939, -0.039, -2.9405],
    #           [-0.1022, -0.0447, 0.0981, -2.6525, 0.4392, -1.1938],
    #           [0.0084, -0.1168, 0.0946, -2.6807, 0.1917, -0.2817],
    #           [-0.0896, 0.0373, 0.1175, 3.0358, -0.3394, 0.6069],
    #           [-0.1, -0.0327, 0.1036, 2.8679, 0.4371, 2.2226],
    #           [-0.1049, -0.0408, 0.1206, 2.7459, -0.0767, 1.9665],
    #           [0.0447, 0.0912, 0.1009, -2.9748, -0.392, -3.0103],
    #           [-0.0422, 0.1084, 0.1113, 2.7705, 0.3279, 0.7212],
    #           [0.0412, -0.1014, 0.0985, 2.8096, -0.3246, -3.1185],
    #           [0.0831, -0.0007, 0.1089, -3.0247, 0.4841, -0.835],
    #           [0.0713, 0.0337, 0.1069, -3.0534, 0.6755, 1.659]]
    # elif instance_name == "Mug_159e56c18906830278d8f8c02c47cde0_M":
    #     gs = [[-0.0189, 0.0561, 0.1459, 3.0965, -0.568, -0.1418],
    #           [-0.0602, 0.011, 0.1532, -2.96, -0.396, 0.2446],
    #           [0.0764, -0.0295, 0.1529, 2.9541, 0.3094, -1.6001],
    #           [0.0674, 0.0689, 0.1486, 2.7429, -0.1111, -0.9344],
    #           [-0.0008, -0.0474, 0.1382, -3.1077, -0.0279, 2.9482],
    #           [0.0242, -0.0368, 0.1558, 3.0074, -0.0423, 0.9607],
    #           [-0.0169, 0.0608, 0.1338, 2.9594, -0.0972, 0.5928],
    #           [0.0615, 0.0498, 0.1372, 3.0229, 0.3808, -0.2851],
    #           [0.0179, 0.0446, 0.1501, 2.9819, -0.2886, -2.6907],
    #           [-0.0398, -0.0917, 0.1216, -2.6866, -0.7536, -0.0166],
    #           [0.0601, -0.0085, 0.139, 3.1062, 0.2869, -1.4959],
    #           [0.0702, -0.009, 0.1613, 3.0488, -0.1552, -2.04],
    #           [0.0831, 0.0286, 0.1302, -2.8137, -0.0166, 1.6709],
    #           [0.0176, 0.0614, 0.1574, 3.0704, -0.1336, -0.5924],
    #           [-0.0361, -0.0661, 0.1361, 2.9468, 0.6832, 3.1322],
    #           [-0.1409, -0.0003, 0.0944, -2.9164, 0.8702, -2.8383],
    #           [0.0831, 0.018, 0.1391, 2.868, 0.0794, -1.2972],
    #           [-0.0365, -0.0745, 0.1511, -2.8471, 0.3713, -0.9122],
    #           [0.0244, 0.0852, 0.1325, -2.8934, 0.4222, 2.4269],
    #           [-0.0065, -0.0926, 0.149, 2.693, -0.1705, 2.9429],
    #           [-0.0709, 0.0026, 0.1478, -2.9779, -0.0742, -1.6203],
    #           [-0.0145, 0.0855, 0.1497, 2.767, 0.3016, 0.4351],
    #           [-0.0732, -0.0171, 0.1414, -3.1382, -0.5225, 0.8298],
    #           [-0.0001, 0.0877, 0.1433, 2.7535, 0.2113, 0.164],
    #           [-0.0684, 0.0121, 0.1551, 3.0032, -0.0088, 1.4444],
    #           [-0.0203, -0.045, 0.1426, 3.1233, 0.0058, 2.1886],
    #           [0.0837, -0.03, 0.1477, 2.8491, -0.452, -2.2755],
    #           [0.013, -0.0731, 0.1391, 2.8509, 0.3463, -2.9644],
    #           [0.0765, -0.0324, 0.1535, 2.874, -0.1975, -2.0895],
    #           [-0.0543, 0.0315, 0.1389, -2.9854, 0.1463, -2.3217]]
    #     # gs = [[-0.05452692849213657, 0.05580193148181692, 0.13194711046198976, -2.9326859394462597, -0.42522750703534445, -1.77970304093843]]
    # elif instance_name == "Pan_a5e0efaaf0ded23e88d29413a3dd87fd_M":
    #     gs = [[0.12154350481326542, -0.011825432322415745, 0.07342278524089237, 2.440350283025683, -0.07771499636406706, -1.7929916686401015]]
    # elif instance_name == "Pan_e8cd2c495f1a33cdd068168ae86ef29a_S":
    #     gs = [[-0.10309832510901273, 0.045809102024986004, 0.1107050805156527, -2.88744678356661, 0.4384439154850482, 1.688992263838868]]
    #
    # for g in gs:
    #     print(f"grasp {g} on body pose {body_pose}")
    #     g = (g[:3], quat_from_euler(g[3:]))
    #
    #     grasp_pose = multiply(body_pose, g)
    #     handles = draw_pose(g, length=0.05)
    #
    #     robot.visualize_grasp(body_pose, g, verbose=True)
    #     input("stop here")
    #
    # # ---------------------------------------------------------------

    problem = State(world)
    # funk = get_grasp_list_gen(problem, collisions=True, visualize=True,
    #                           RETAIN_ALL=True, top_grasp_tolerance=None, verbose=True)
    funk = get_grasp_gen(problem, collisions=True, visualize=True,
                         RETAIN_ALL=True, top_grasp_tolerance=None, verbose=True)

    outputs = funk(body)
    if isinstance(outputs, list):
        print(f'grasps on body {body}:', outputs)

    # gripper_body = robot.grippers["hand"]
    # print(robot.grippers)

    for grasp in outputs:

        if "hand" in robot.grippers:
            gripper_body = robot.grippers["hand"]
            remove_body(gripper_body)
            robot.remove_gripper()

        grasp = grasp[0]
        gripper_grasp = robot.visualize_grasp(body_pose, grasp.value, body=grasp.body, width=grasp.grasp_width)
        # gripper_approach = robot.visualize_grasp(body_pose, grasp.approach, color=RED)
        input("next grasp?")
        remove_body(gripper_grasp)
        # remove_body(gripper_approach)
        robot.remove_gripper()


    # body_pose = get_pose(body)
    # visualize_grasps(problem, outputs, body_pose, RETAIN_ALL=not test_attachment or True,
    #                  TEST_ATTACHMENT=test_attachment)

    input("next")


if __name__ == '__main__':
    main()



