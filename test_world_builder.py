import os
from os.path import join, isfile
import sys
sys.path.extend(['pddlstream', 'pybullet_planning'])

import pybullet as p
from pybullet_tools.utils import set_random_seed, connect, enable_preview, \
    disconnect, draw_pose, set_all_static, wait_if_gui, remove_handles, unit_pose, get_sample_fn, pairwise_collision, \
    set_camera_pose, add_line, get_point, BLACK, get_name, CLIENTS, get_client, link_from_name, \
    get_link_subtree, clone_body, set_all_color, GREEN, BROWN, invert, multiply, set_pose, VideoSaver

from pybullet_tools.utils import apply_alpha, get_camera_matrix, LockRenderer, HideOutput, load_model, TURTLEBOT_URDF, \
    set_all_color, dump_body, draw_base_limits, multiply, Pose, Euler, PI, draw_pose, unit_pose, create_box, TAN, Point, \
    GREEN, create_cylinder, INF, BLACK, WHITE, RGBA, GREY, YELLOW, BLUE, BROWN, RED, stable_z, set_point, set_camera_pose, \
    set_all_static, get_model_info, load_pybullet, remove_body, get_aabb, set_pose, wait_if_gui, get_joint_names, \
    get_min_limit, get_max_limit, set_joint_position, set_joint_position, get_joints, get_joint_info, get_moving_links, \
    get_pose, get_joint_position, enable_gravity, enable_real_time, get_links, set_color, dump_link, draw_link_name, \
    get_link_pose, get_aabb, get_link_name, sample_aabb, aabb_contains_aabb, aabb2d_from_aabb, sample_placement, \
    aabb_overlap, get_links, get_collision_data, get_visual_data, link_from_name, body_collision, get_closest_points, \
    load_pybullet, FLOOR_URDF, pairwise_collision, is_movable, get_bodies, get_aabb_center, draw_aabb
from pybullet_tools.pr2_primitives import get_group_joints, Conf

from world_builder.world import World, State
from world_builder.entities import Object, Region, Environment, Robot, Camera, Floor, Stove,\
    Surface, Moveable, Supporter, Steerable, Door, create_robot
from world_builder.world_generator import to_lisdf

import argparse

def get_parser():
    parser = argparse.ArgumentParser()
    ## -------- simulation related
    parser.add_argument('-v', '--viewer', action='store_true', help='')
    parser.add_argument('-d', '--drive', action='store_true', help='')
    parser.add_argument('-t', '--time_step', type=float, default=4e-0)
    parser.add_argument('--teleport', action='store_true', help='')
    parser.add_argument('-s', '--seed', type=int, default=None, help='')
    parser.add_argument('-cam', '--camera', action='store_true', default=True, help='')
    parser.add_argument('-seg', '--segment', action='store_true', default=False, help='')
    parser.add_argument('-mon', '--monitoring', action='store_true', default=False)

    args = parser.parse_args()  # TODO: flag to save a video
    set_random_seed(args.seed)
    return args

def create_pybullet_world(builder, floorplan=None, SAVE_LISDF=False, world_name='test_scene'):
    args = get_parser()

    connect(use_gui=True, shadows=False, width=1980, height=1238)

    # set_camera_pose(camera_point=[2.5, 0., 3.5], target_point=[1., 0, 1.])
    if args.camera:
        enable_preview()
    if not args.segment:
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False)
    # p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, True)
    # parameter = add_parameter(name='facts')
    # button = add_button(name='TBD')
    draw_pose(unit_pose(), length=1.)  # TODO: draw grid

    world = World(args, time_step=args.time_step)
    robot = create_robot(world)

    builder(world, floorplan)

    ## no gravity once simulation starts
    set_all_static()
    world.summarize_all_objects()

    state = State(world)
    file = None
    if SAVE_LISDF:
        file = to_lisdf(state.world, state.get_facts(), floorplan=floorplan, world_name=world_name)

    wait_if_gui('exit?')
    disconnect()
    return file

def build_world_from_shapes(world, floorplan=None):
    w = .5
    h = .9
    mass = 1

    fridge = world.add_box(
        Supporter(create_box(w, w, h, color=(.75, .75, .75, 1)), name='fridge'),
        Pose(point=Point(2, 0, h / 2)))

    egg = world.add_box(
        Moveable(create_box(.07, .07, .1, mass=mass, color=(1, 1, 0, 1)), category='egg', name='egg'),
        Pose(point=Point(2, -0.18, h + .1 / 2)))

    cabbage = world.add_box(
        Moveable(create_box(.07, .07, .1, mass=mass, color=(0, 1, 0, 1)), category='veggie', name='cabbage'),
        Pose(point=Point(2, 0, h + .1 / 2)))

    salter = world.add_box(
        Moveable(create_box(.07, .07, .1, mass=mass, color=(0, 0, 0, 1)), category='salter', name='salter'),
        Pose(point=Point(2, 0.18, h + .1 / 2)))

    plate = world.add_box(
        Moveable(create_box(.07, .07, .1, mass=mass, color=(0.4, 0.4, 0.4, 1)), category='plate', name='plate'),
        Pose(point=Point(2 + 0.18, 0, h + .1 / 2)))

    sink = world.add_box(
        Supporter(create_box(w, w, h, color=(.25, .25, .75, 1)), category='sink', name='sink'),
        Pose(point=Point(0, 2, h / 2)))

    stove = world.add_box(
        Supporter(create_box(w, w, h, color=(.75, .25, .25, 1)), category='stove', name='stove'),
        Pose(point=Point(0, -2, h / 2)))

    counter = world.add_box(
        Supporter(create_box(w, w, h, color=(.25, .75, .25, 1)), category='counter', name='counter'),
        Pose(point=Point(-2, 2, h / 2)))

    table = world.add_box(
        Supporter(create_box(w, w, h, color=(.75, .75, .25, 1)), category='table', name='table'),
        Pose(point=Point(-2, -2, h / 2)))


def build_world_from_floorplan(world, floorplan=None):
    pass

if __name__ == '__main__':
    builder = build_world_from_shapes
    create_pybullet_world(builder, SAVE_LISDF=False)
