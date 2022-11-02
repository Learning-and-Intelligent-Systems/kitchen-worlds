import os
from os.path import join, isfile
import sys
from config import ASSET_PATH, EXP_PATH
import time
import tqdm
import pybullet as p
import random
import numpy as np
import argparse

from pybullet_tools.utils import set_random_seed, set_numpy_seed, connect, enable_preview, \
    disconnect, draw_pose, set_all_static, wait_if_gui, remove_handles, unit_pose, get_sample_fn, pairwise_collision, \
    set_camera_pose, add_line, get_point, BLACK, get_name, CLIENTS, get_client, link_from_name, \
    get_link_subtree, clone_body, set_all_color, GREEN, BROWN, invert, multiply, set_pose, VideoSaver, reset_simulation, \
    stable_z, Euler

from world_builder.world import World, State
from world_builder.builders import test_pick, test_exist_omelette, test_kitchen_oven, test_feg_pick, test_one_fridge

from pybullet_planning.world_builder.entities import Object, Floor, Moveable, Surface, Supporter
from pybullet_tools.utils import create_box, TAN, WHITE, BLACK, GREY, Pose, Point, PI, load_model, \
get_aabb, get_aabb_extent

from pybullet_planning.world_builder.utils import read_xml, load_asset, get_model_scale


def test_feg_kitchen(world, **kwargs):
    sample_kitchen_counter_scene(world, **kwargs)
    goal = sample_kitchen_counter_goal(world)
    return goal


def sample_kitchen_counter_scene(world, **kwargs):

    # load floor
    FLOOR_HEIGHT = 1e-3
    FLOOR_WIDTH = 4
    FLOOR_LENGTH = 8
    floor = world.add_object(
        Floor(create_box(w=round(FLOOR_WIDTH, 1), l=round(FLOOR_LENGTH, 1), h=FLOOR_HEIGHT, color=TAN, collision=True)),
        Pose(point=Point(x=round(0.5*FLOOR_WIDTH, 1), y=round(0, 1), z=-0.5*FLOOR_HEIGHT)))

    # load wall
    WALL_HEIGHT = 5
    WALL_WIDTH = FLOOR_HEIGHT
    WALL_LENGTH = FLOOR_LENGTH
    wall = world.add_object(
        Supporter(create_box(w=round(WALL_WIDTH, 1), l=round(WALL_LENGTH, 1), h=WALL_HEIGHT, color=WHITE, collision=True)),
        Pose(point=Point(x=round(-0.5*WALL_WIDTH, 1), y=round(0, 1), z=0.5*WALL_HEIGHT)))


    # sample ordering of fridge, oven, dishwasher
    if_fridge = random.randint(0, 4) # 0: no fridge, odd: fridge, even: minifridge
    if_minifridge = random.randint(0, 4)
    if_oven = random.randint(0, 4) # 80% chance of getting oven
    if_dishwasher = random.randint(0, 4)

    fixtures = [None, None]

    if if_dishwasher:
        fixtures[0] = 'dishwasher'
    if if_minifridge:
        fixtures[1] = 'minifridge'
    random.shuffle(fixtures)

    if random.randint(0, 1): # oven left
        if if_oven:
            fixtures.insert(0, 'oven')
        else:
            fixtures.insert(0, None)
        if if_fridge:
            fixtures.append('fridge')
        else:
            fixtures.append(None)
    else:
        if if_fridge:
            fixtures.insert(0, 'fridge')
        else:
            fixtures.insert(0, None)
        if if_oven:
            fixtures.append('oven')
        else:
            fixtures.append(None)

    # if if_fridge == 0:
    #     fixtures.append(None)
    #     random.shuffle(fixtures)
    # elif (if_fridge%2) == 1:
    #     if random.randint(0, 1):
    #         fixtures.insert(0, 'fridge')
    #     else:
    #         fixtures.append('fridge')
    # elif (if_fridge%2) == 0:
    #     if random.randint(0, 1):
    #         fixtures.insert(0, 'minifridge')
    #     else:
    #         fixtures.append('minifridge')

    # sample placements of fixtures
    yaw = {0: 0, 90: PI / 2, 180: PI, 270: -PI / 2}[180]
    MIN_COUNTER_Z = 0.9
    fixtures_cfg = {}


    for idx, cat in enumerate(fixtures):
        if cat:
            center_x = 0.6
            center_y = -1.5 + idx*1
            center_x += random.random()*0.1 - 0.1
            center_y += random.random()*0.1 - 0.1
            center_z = MIN_COUNTER_Z - random.random()*0.1 - 0.05
            w = 2*min(abs(center_x - 0), abs(1-center_x))
            l = 2*min(abs(center_y - 2 + idx*1), abs(-1 + idx*1 -center_y))
            fixture = {}
            if idx in [1, 2]: # control height for center furniture
                fixture['id'] = world.add_object(Object(load_asset(cat, x=center_x, y=center_y, yaw=yaw, \
                                floor=floor, w=w, l=l, h=center_z, RANDOM_INSTANCE=True)))
            else:
                fixture['id'] = world.add_object(Object(load_asset(cat, x=center_x, y=center_y, yaw=yaw, \
                                floor=floor, w=w, l=l, h=2.5*MIN_COUNTER_Z, RANDOM_INSTANCE=True)))
            center_z = stable_z(fixture['id'].body, floor)
            # center_x = 1-get_aabb_extent(get_aabb(fixture['id'].body))[0]/2
            center_x += 1 - get_aabb(fixture['id'].body)[1][0]
            fixture['pose'] = Pose(point=Point(x=center_x, y=center_y, z=center_z), euler=Euler(yaw=yaw))
            set_pose(fixture['id'].body, fixture['pose'])
            fixtures_cfg[cat] = fixture

    # oven_aabb = get_aabb(fixtures_cfg['oven']['id'])
    # fridge_aabb = get_aabb(fixtures_cfg['fridge']['id'])
    if fixtures[0] is not None:
        min_counter_y = get_aabb(fixtures_cfg[fixtures[0]]['id'])[1][1]
    else:
        min_counter_y = -2
    if fixtures[3] is not None:
        max_counter_y = get_aabb(fixtures_cfg[fixtures[3]]['id'])[0][1]
    else:
        max_counter_y = 2
    min_counter_z = MIN_COUNTER_Z
    if fixtures[1] is not None:
        tmp_counter_z = get_aabb(fixtures_cfg[fixtures[1]]['id'])[1][2]
        if tmp_counter_z > min_counter_z:
            min_counter_z = tmp_counter_z
    if fixtures[2] is not None:
        tmp_counter_z = get_aabb(fixtures_cfg[fixtures[2]]['id'])[1][2]
        if tmp_counter_z > min_counter_z:
            min_counter_z = tmp_counter_z

    # add counter
    COUNTER_HEIGHT = 0.05
    COUNTER_WIDTH = 1
    COUNTER_LENGTH = max_counter_y - min_counter_y

    counter = world.add_object(
        Supporter(create_box(w=COUNTER_WIDTH, l=COUNTER_LENGTH, h=COUNTER_HEIGHT, color=GREY, collision=True)),
        Pose(point=Point(x=0.5*COUNTER_WIDTH, y=(max_counter_y+min_counter_y)/2, z=min_counter_z)))


    # microwave = world.name_to_body('microwave')
    # world.put_on_surface(microwave, counter)
    microwave = counter.place_new_obj('microwave', scale=0.4 + 0.1*random.random())
    set_pose(microwave, Pose(point=microwave.get_pose()[0], euler=Euler(yaw=yaw)))


    # add pot
    pot = counter.place_new_obj('kitchenpot', scale=0.2)
    set_pose(pot, Pose(point=pot.get_pose()[0], euler=Euler(yaw=yaw)))

    # add shelf
    SHELF_HEIGHT = 0.05
    SHELF_WIDTH = 0.5
    MIN_SHELF_LENGTH = 1
    min_shelf_y = min_counter_y + random.random()*(max_counter_y - min_counter_y - MIN_SHELF_LENGTH)
    max_shelf_y = max_counter_y - random.random()*(max_counter_y - min_shelf_y - MIN_SHELF_LENGTH)
    SHELF_LENGTH = max_shelf_y - min_shelf_y

    shelf = world.add_object(
        Supporter(create_box(w=SHELF_WIDTH, l=SHELF_LENGTH, h=SHELF_HEIGHT, color=GREY, collision=True)),
        Pose(point=Point(x=0.5*SHELF_WIDTH, y=(max_shelf_y+min_shelf_y)/2, z=1.5)))


    # add food
    food_ids = []
    for i in range(5):
        food_ids.append(counter.place_new_obj('food'))
        set_pose(food_ids[i], Pose(point=food_ids[i].get_pose()[0]))

    # add bottle
    bottle_ids = []
    for i in range(3):
        bottle_ids.append(shelf.place_new_obj('bottle'))
        set_pose(bottle_ids[i], Pose(point=bottle_ids[i].get_pose()[0]))


    # add camera
    camera_pose = Pose(point=Point(x=4.2, y=0, z=2.5), euler=Euler(roll=PI/2+PI/8, pitch=0, yaw=-PI/2))
    world.add_camera(camera_pose)
    rgb, depth, segmented, view_pose, camera_matrix = world.camera.get_image()


def sample_kitchen_counter_goal(world):
    bottle = random.choice(world.cat_to_bodies('bottle'))

    hand = world.robot.arms[0]
    goal_candidates = [
        [('Holding', hand, bottle)],
        # [('On', cabbage, counter)],
        # [('In', cabbage, fridge)],
    ]
    return random.choice(goal_candidates)


def main():


    DEFAULT_TEST = test_kitchen_oven  ## test_one_fridge | test_feg_pick | test_kitchen_oven | test_exist_omelette
    USE_GUI = True


    """ ============== initiate simulator ==================== """

    ## for viewing, not the size of depth image
    connect(use_gui=USE_GUI, shadows=False, width=1200, height=800)

    # set_camera_pose(camera_point=[2.5, 0., 3.5], target_point=[1., 0, 1.])
    set_camera_pose(camera_point=[4, 0., 2.5], target_point=[0, 0, 0])
    if True: #args.camera:
        enable_preview()
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)
    draw_pose(unit_pose(), length=1.)

    """ ============== sample world configuration ==================== """
    world = World(time_step=4e-0)
    # _ = test_feg_pick(world, floorplan='kitchen_v3.svg', verbose=False)


    # seed = 0
    # set_random_seed(seed)
    # set_numpy_seed(seed)

    sample_kitchen_counter_scene(world)
    # time.sleep(2)
    # from IPython import embed
    # embed()
    # input("Press the <Enter> key on the keyboard to exit.")


if __name__ == '__main__':
    main()
