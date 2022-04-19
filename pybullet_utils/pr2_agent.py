from __future__ import print_function

import os
import time

from bullet.utils import summarize_facts, print_plan, print_goal, save_pickle, set_camera_target_body, \
    set_camera_target_robot, nice
from bullet.processes.pddlstream_agent.pr2_streams import get_stable_gen, get_contain_gen, get_position_gen, \
    Position, get_handle_grasp_gen, LinkPose, get_ik_ir_grasp_handle_gen, get_pull_drawer_handle_motion_gen, \
    get_joint_position_test, get_marker_grasp_gen, get_bconf_in_region_test, get_pull_door_handle_motion_gen, \
    get_bconf_in_region_gen, get_pose_in_region_gen, visualize_grasp, get_motion_wconf_gen, get_update_wconf_p_two_gen, \
    get_marker_pose_gen, get_pull_marker_to_pose_motion_gen, get_pull_marker_to_bconf_motion_gen,  \
    get_pull_marker_random_motion_gen, get_ik_ungrasp_handle_gen, get_pose_in_region_test, \
    get_cfree_btraj_pose_test, get_joint_position_open_gen, get_ik_ungrasp_mark_gen, \
    sample_joint_position_open_list_gen, get_update_wconf_pst_gen, get_ik_ir_wconf_gen, \
    get_update_wconf_p_gen, get_ik_ir_wconf_gen, get_pose_in_space_test, get_turn_knob_handle_motion_gen

from bullet.worlds.kitchen_worlds import BASE_LIMITS
from bullet.entities import Object

from pybullet_tools.pr2_primitives import get_group_joints, Conf
from pybullet_tools.pr2_problems import create_pr2
from pybullet_tools.pr2_utils import PR2_TOOL_FRAMES, create_gripper, set_group_conf
from pybullet_tools.utils import connect, disconnect, wait_if_gui, LockRenderer, HideOutput, get_client, \
    joint_from_name, WorldSaver, sample_placement, PI, add_parameter, add_button, Pose, Point, Euler, \
    euler_from_quat, get_joint, get_joints, PoseSaver, get_pose, get_link_pose, get_aabb, \
    get_joint_position, aabb_overlap, add_text, remove_handles, get_com_pose, get_closest_points,\
    set_color, RED, YELLOW, GREEN, multiply, get_unit_vector, unit_quat, get_bodies, BROWN, \
    pairwise_collision

# from leap_utils import check_domain
import sys
from os.path import join, isfile
sys.path.insert(0, os.path.abspath(join('..','..','..')))
from pddlstream.algorithms.algorithm import parse_problem, reset_globals
from pddlstream.algorithms.constraints import PlanConstraints

## for pr2 streams
from examples.pybullet.namo.stream import get_custom_limits as get_base_custom_limits
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, point_from_pose, \
    disconnect, get_joint_positions, enable_gravity, save_state, restore_state, HideOutput, remove_body, \
    get_distance, LockRenderer, get_min_limit, get_max_limit, has_gui, WorldSaver, wait_if_gui, add_line, SEPARATOR, \
    BROWN, BLUE, WHITE, TAN, GREY, YELLOW, GREEN, BLACK, RED
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test
from pddlstream.language.constants import Equal, AND, PDDLProblem, is_plan
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.object import SharedOptValue
from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    move_cost_fn
from collections import namedtuple


def place_movable(certified):
    for literal in certified:
        if literal[0] != 'not':
            continue
        fact = literal[1]
        if fact[0] == 'trajposecollision':
            _, b, p = fact[1:]
            p.assign()
        if fact[0] == 'trajarmcollision':
            _, a, q = fact[1:]
            q.assign()
        if fact[0] == 'trajgraspcollision':
            _, a, o, g = fact[1:]
            # TODO: finish this

def move_cost_fn(c):
    [t] = c.commands
    distance = t.distance(distance_fn=lambda q1, q2: get_distance(q1[:2], q2[:2]))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

def extract_point2d(v):
    if isinstance(v, Conf):
        return v.values[:2]
    if isinstance(v, Pose):
        return point_from_pose(v.value)[:2]
    if isinstance(v, SharedOptValue):
        if v.stream == 'sample-pose':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'inverse-kinematics':
            p, = v.values
            return extract_point2d(p)
    if isinstance(v, CustomValue):
        if v.stream == 'p-sp':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'q-ik':
            p, = v.values
            return extract_point2d(p)
    raise ValueError(v.stream)

def opt_move_cost_fn(t):
    # q1, q2 = t.values
    # distance = get_distance(extract_point2d(q1), extract_point2d(q2))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

CustomValue = namedtuple('CustomValue', ['stream', 'values'])

def opt_pose_fn(o, r):
    p = CustomValue('p-sp', (r,))
    return p,

def opt_ik_fn(a, o, p, g):
    q = CustomValue('q-ik', (p,))
    t = CustomValue('t-ik', tuple())
    return q, t

def opt_motion_fn(q1, q2):
    t = CustomValue('t-pbm', (q1, q2))
    return t,

def opt_pose_inside_fn(o, r):
    p = CustomValue('p-spi', (r,))
    return p,

def opt_position_fn(o, r):
    p = CustomValue('pstn-end', (r,))
    return p,

def opt_ik_grasp_fn(a, o, p, g):
    q = CustomValue('q-ik-hg', (p,))
    aq = CustomValue('aq-ik-hg', (p,))
    t = CustomValue('t-ik-hg', tuple())
    return q, aq, t

def opt_ik_wconf_fn(a, o, p, g, w):
    q = CustomValue('q-ik', (p,))
    t = CustomValue('t-ik', tuple())
    return q, t

def opt_motion_wconf_fn(q1, q2, w):
    t = CustomValue('t-pbm', (q1, q2))
    return t,

stream_info = {
    # 'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=verbose),
    # 'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=verbose),
    # 'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=verbose),
    'MoveCost': FunctionInfo(opt_move_cost_fn),
}
stream_info.update({
    'sample-pose': StreamInfo(opt_gen_fn=from_fn(opt_pose_fn)),
    'sample-pose-inside': StreamInfo(opt_gen_fn=from_fn(opt_pose_inside_fn)),
    'inverse-kinematics': StreamInfo(opt_gen_fn=from_fn(opt_ik_fn)),
    # 'inverse-kinematics-wconf': StreamInfo(opt_gen_fn=from_fn(opt_ik_wconf_fn)),
    'plan-base-motion': StreamInfo(opt_gen_fn=from_fn(opt_motion_fn)),
    # 'plan-base-motion-wconf': StreamInfo(opt_gen_fn=from_fn(opt_motion_wconf_fn)),
    'sample-joint-position': StreamInfo(opt_gen_fn=from_fn(opt_position_fn)),
    'inverse-kinematics-grasp-handle': StreamInfo(opt_gen_fn=from_fn(opt_ik_grasp_fn)),
})

#######################################################

class Problem(object):
    def __init__(self, robot, movable=tuple(), openable=tuple(),
                 surfaces=tuple(), spaces=tuple(), floors=tuple(),
                 grasp_types=tuple(['top']), arms=tuple(['left']),  ## 'side',
                 costs=False, body_names={}, body_types=[], base_limits=None):
        self.robot = robot
        self.arms = arms
        self.movable = movable
        self.openable = openable
        self.grasp_types = grasp_types
        self.surfaces = surfaces
        self.spaces = spaces
        self.floors = floors

        # self.sinks = sinks
        # self.stoves = stoves
        # self.buttons = buttons

        # self.goal_conf = goal_conf
        # self.goal_holding = goal_holding
        # self.goal_on = goal_on
        # self.goal_cleaned = goal_cleaned
        # self.goal_cooked = goal_cooked
        self.costs = costs
        self.body_names = body_names
        self.body_types = body_types
        self.base_limits = base_limits
        all_movable = [self.robot] + list(self.movable)
        self.fixed = list(filter(lambda b: b not in all_movable, get_bodies()))
        self.gripper = None
    def get_gripper(self, arm='left', visual=True):
        # upper = get_max_limit(problem.robot, get_gripper_joints(problem.robot, 'left')[0])
        # set_configuration(gripper, [0]*4)
        # dump_body(gripper)
        if self.gripper is None:
            self.gripper = create_gripper(self.robot, arm=arm, visual=visual)
        return self.gripper
    def remove_gripper(self):
        if self.gripper is not None:
            remove_body(self.gripper)
            self.gripper = None
    def __repr__(self):
        return repr(self.__dict__)

#######################################################

def pddlstream_from_state_goal(state, goals, domain_pddl='pr2_kitchen.pddl',
                               stream_pddl='pr2_stream.pddl',
                               base_limits=BASE_LIMITS,
                                init_facts=[], ## avoid duplicates
                                facts=[],  ## completely overwrite
                                collisions=True, teleport=False):
    from zzz.logging import myprint as print

    robot = state.robot
    world = state.world
    problem = state
    custom_limits = get_base_custom_limits(robot, base_limits)

    world.summarize_all_objects()

    if len(facts) == 0:
        facts = state.get_facts(init_facts)
    init = facts

    print(f'pr2_agent.pddlstream_from_state_goal(\n'
          f'\tdomain = {domain_pddl}, \n'
          f'\tstream = {stream_pddl}, base_limits = {base_limits}')

    if isinstance(goals, tuple): ## debugging
        test, name = goals
        # test_initial_region(state, init)
        # test_marker_pull_bconfs(state, init)
        if test == 'test_handle_grasps':
            goals = test_handle_grasps(state, name)
        elif test == 'test_grasps':
            goals = test_grasps(state, name)
        elif test == 'test_grasp_ik':
            goals = test_grasp_ik(state, init, name)
        # test_pulling_handle_ik(state)
        # test_drawer_open(state, goals)
        elif test == 'test_pose_gen':
            goals, ff = test_pose_gen(state, init, name[0], name[1])
            init += ff

    goal = [AND]
    goal += goals
    if goals[0][0] == 'AtBConf':
        init += [('BConf', goals[0][1])]
    elif goals[0][0] == 'AtPosition':
        init += [('Position', goals[0][1], goals[0][2])]
    elif goals[0][0] == 'AtGrasp':
        init += [('Grasp', goals[0][2], goals[0][3])]
    elif goals[0][0] == 'AtHandleGrasp':
        init += [('HandleGrasp', goals[0][2], goals[0][3])]
    elif goals[0][0] == 'AtMarkerGrasp':
        init += [('MarkerGrasp', goals[0][2], goals[0][3])]

    if goal[-1] == ("not", ("AtBConf", "")):
        atbconf = [i for i in init if i[0].lower() == "AtBConf".lower()][0]
        goal[-1] = ("not", atbconf)

    summarize_facts(init, world, name='Facts extracted from observation')

    ## make all pred lower case
    new_init = []
    for fact in init:
        new_tup = [fact[0].lower()]
        new_tup.extend(fact[1:])
        new_init.append(tuple(new_tup))
    init = new_init

    init_added = [n for n in init_facts if n not in init]
    if len(init_facts) != 0:  ## only print the world facts the first time
        summarize_facts(init_added, world, name='Added facts from PDDLStream preimage')
        init = init + init_added

    domain_pddl = read(get_file_path(__file__, join('pddl', 'domains', domain_pddl)))
    stream_pddl = read(get_file_path(__file__, join('pddl', 'streams', stream_pddl)))
    constant_map = {}
    goal = [g for g in goal if not (g[0] == 'not' and g[1][0] == '=')]
    print_goal(goal)
    p = problem
    c = collisions
    l = custom_limits
    t = teleport
    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(p, collisions=c)),
        'sample-pose-inside': from_gen_fn(get_contain_gen(p, collisions=c)),  ##
        'sample-grasp': from_list_fn(get_grasp_gen(p, collisions=True)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(p, collisions=c, teleport= t, custom_limits=l,
                                                        learned=False, max_attempts=60, verbose=False)),
        'inverse-kinematics-wconf': from_gen_fn(get_ik_ir_wconf_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                    learned=False, max_attempts=60, verbose=False, visualize=False)),
        'plan-base-motion': from_fn(get_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-base-motion-wconf': from_fn(get_motion_wconf_gen(p, collisions=c, teleport=t, custom_limits=l)),

        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=c)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(p, collisions=c)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(p.robot, collisions=c)),

        'test-cfree-btraj-pose': from_test(get_cfree_btraj_pose_test(p.robot, collisions=c)),

        # 'get-joint-position-open': from_fn(get_joint_position_open_gen(p)),
        'get-joint-position-open': from_list_fn(sample_joint_position_open_list_gen(p)),
        # 'sample-joint-position-open': from_fn(get_position_gen(p, collisions=c, extent='max')),
        # 'sample-joint-position-closed': from_fn(get_position_gen(p, collisions=c, extent='min')),
        # 'test-joint-position-open': from_test(get_joint_position_test(extent='max')),
        # 'test-joint-position-closed': from_test(get_joint_position_test(extent='min')),

        'sample-handle-grasp': from_list_fn(get_handle_grasp_gen(p, collisions=c)),

        'inverse-kinematics-grasp-handle': from_gen_fn(get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                                  learned=False, verbose=False, ACONF=True, WCONF=False)),
        'inverse-kinematics-ungrasp-handle': from_gen_fn(get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                               verbose=False, WCONF=False)),
        'inverse-kinematics-grasp-handle-wconf': from_gen_fn(get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                                     learned=False, verbose=False, ACONF=True, WCONF=True)),
        'inverse-kinematics-ungrasp-handle-wconf': from_gen_fn(get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                                    verbose=False, WCONF=True)),

        'plan-base-pull-drawer-handle': from_fn(get_pull_drawer_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-base-pull-door-handle': from_fn(get_pull_door_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-arm-turn-knob-handle': from_fn(get_turn_knob_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),

        'sample-marker-grasp': from_list_fn(get_marker_grasp_gen(p, collisions=c)),
        'inverse-kinematics-grasp-marker': from_gen_fn(get_ik_ir_grasp_handle_gen(p, collisions=True, teleport=t, custom_limits=l,
                                                                                    learned=False, verbose=False)),
        'inverse-kinematics-ungrasp-marker': from_fn(get_ik_ungrasp_mark_gen(p, collisions=True, teleport=t, custom_limits=l)),
        'plan-base-pull-marker-random': from_gen_fn(get_pull_marker_random_motion_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                                      learned=False)),

        'sample-marker-pose': from_list_fn(get_marker_pose_gen(p, collisions=c)),
        'plan-base-pull-marker-to-bconf': from_fn(get_pull_marker_to_bconf_motion_gen(p, collisions=c, teleport=t)),
        'plan-base-pull-marker-to-pose': from_fn(get_pull_marker_to_pose_motion_gen(p, collisions=c, teleport=t)),
        'test-bconf-in-region': from_test(get_bconf_in_region_test(p.robot)),
        'test-pose-in-region': from_test(get_pose_in_region_test()),
        'test-pose-in-space': from_test(get_pose_in_space_test()),  ##

        # 'sample-bconf-in-region': from_gen_fn(get_bconf_in_region_gen(p, collisions=c, visualize=False)),
        'sample-bconf-in-region': from_list_fn(get_bconf_in_region_gen(p, collisions=c, visualize=False)),
        'sample-pose-in-region': from_list_fn(get_pose_in_region_gen(p, collisions=c, visualize=False)),

        'update-wconf-p': from_fn(get_update_wconf_p_gen()),
        'update-wconf-p-two': from_fn(get_update_wconf_p_two_gen()),
        'update-wconf-pst': from_fn(get_update_wconf_pst_gen()),

        'MoveCost': move_cost_fn,

        # 'TrajPoseCollision': fn_from_constant(False),
        # 'TrajArmCollision': fn_from_constant(False),
        # 'TrajGraspCollision': fn_from_constant(False),
    }
    # get_press_gen(problem, teleport=teleport)
    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def is_plan_abstract(plan):
    for step in plan:
        if '--no-' in step.name:
            return True
    return False

def solve_pddlstream(problem, state, domain_pddl=None):
    from examples.pybullet.utils.pybullet_tools.utils import CLIENTS
    from pddlstream.algorithms.focused import solve_focused
    from zzz.logging import myprint as print

    start_time = time.time()

    CLIENTS[get_client()] = True # TODO: hack
    saver = WorldSaver()
    pddlstream_problem = problem
    world = state.world
    objects = world.objects

    #########################

    # _, _, _, stream_map, init, goal = pddlstream_problem
    # print(f'Init ({len(init)}):', init)
    # print_goal(goal)
    # print('Streams:', [str(s) for s in set(stream_map)])
    print(SEPARATOR)

    with Profiler(): ##field='tottime', num=10):
        with LockRenderer(lock=True):
            # solution = solve(pddlstream_problem, algorithm='adaptive', unit_costs=True, visualize=False,
            #                  stream_info=stream_info, success_cost=INF, verbose=True, debug=False)
            solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                                     planner='ff-astar1', max_planner_time=10, debug=False,
                                     unit_costs=True, success_cost=INF,
                                     max_time=INF, verbose=True, visualize=False,
                                     unit_efforts=True, effort_weight=1,
                                     bind=True, max_skeletons=INF,  # TODO: double check that max_skeletons=None is working
                                     search_sample_ratio=0)
            saver.restore()

    knowledge = parse_problem(pddlstream_problem, stream_info=stream_info,
                              constraints=PlanConstraints(), unit_costs=True, unit_efforts=True)

    plan, cost, evaluations = solution
    solved = is_plan(plan)

    print('Solved: {}'.format(solved))
    print('Cost: {:.3f}'.format(cost))
    print_plan(plan, world)

    time_log = {'planning': round(time.time()-start_time, 4)}
    start_time = time.time()
    if plan != None:
        preimage = evaluations.preimage_facts

        ## ------ debug why can't find action skeleton
        ## test_grasp_ik(state, state.get_facts()+preimage, name='eggblock')

        ## save_pickle(pddlstream_problem, plan, preimage) ## discarded

        summarize_facts(preimage, world, name='Preimage generated by PDDLStream')

        if is_plan_abstract(plan):
            from bullet.leap.hierarchical import check_preimage
            env = check_preimage(pddlstream_problem, plan, preimage, init=problem.init,
                                 objects=objects, domain_pddl=domain_pddl)
        else:
            env = None  ## preimage
        plan_str = [str(a) for a in plan]
    else:
        env = None
        plan_str = 'FAILED'
        preimage = []

    time_log['preimage'] = round(time.time() - start_time, 4)
    time_log['goal'] = [f'{g[0]}({g[1:]})' for g in problem.goal[1:]]
    time_log['plan'] = plan_str
    time_log['plan_len'] = len(plan) if plan != None else 0

    reset_globals()  ## reset PDDLStream solutions

    return plan, env, knowledge, time_log, preimage

def test_initial_region(state, init):
    world = state.world
    if world.name_to_body('hallway') != None:
        robot = state.robot
        marker = world.name_to_body('marker')
        funk = get_bconf_in_region_test(robot)
        funk2 = get_pose_in_region_test(robot)
        bq = [i for i in init if i[0].lower() == "AtBConf".lower()][0][-1]
        p = [i for i in init if i[0].lower() == "AtPose".lower() and i[1] == marker][0][-1]

        for location in ['hallway', 'storage', 'kitchen']:
            answer1 = funk(bq, world.name_to_body(location))
            answer2 = funk2(marker, p, world.name_to_body(location))
            print(f"RobInRoom({location}) = {answer1}\tInRoom({marker}, {location}) = {answer2}")
        print('---------------\n')


def test_marker_pull_bconfs(state, init):
    world = state.world
    funk = get_pull_marker_random_motion_gen(state)  ## is a from_fn
    o = world.name_to_body('marker')
    p1 = [i for i in init if i[0].lower() == "AtPose".lower() and i[1] == o][0][2]
    g = test_marker_pull_grasps(state, o)
    bq1 = [i for i in init if i[0].lower() == "AtBConf".lower()][0][1]
    p2, bq2, t = funk('left', o, p1, g, bq1)
    rbb = create_pr2()
    set_group_conf(rbb, 'base', bq2.values)

def test_marker_pull_grasps(state, marker, visualize=False):
    funk = get_marker_grasp_gen(state)
    grasps = funk(marker) ## funk(cart) ## is a previous version
    if visualize:
        robot = state.robot
        cart = state.world.BODY_TO_OBJECT[marker].grasp_parent
        for grasp in grasps:
            gripper_grasp = visualize_grasp(robot, get_pose(marker), grasp[0].value)
            set_camera_target_body(gripper_grasp, dx=0, dy=-1, dz=0)
            print('collision with marker', pairwise_collision(gripper_grasp, marker))
            print('collision with cart', pairwise_collision(gripper_grasp, cart))
            remove_body(gripper_grasp)
    print('test_marker_pull_grasps:', grasps)
    return grasps

def test_handle_grasps(state, name='hitman_drawer_top_joint', visualize=True):
    if isinstance(name, str):
        body_joint = state.world.name_to_body(name)
    else: ##if isinstance(name, Object):
        body_joint = name
        name = state.world.BODY_TO_OBJECT[body_joint].shorter_name
    funk = get_handle_grasp_gen(state, visualize=False)
    outputs = funk(body_joint)
    if visualize:
        name_to_object = state.world.name_to_object
        body_pose = name_to_object(name).get_handle_pose()
        visualize_grasps_by_quat(state, outputs, body_pose, RETAIN_ALL=True)
    print('test_handle_grasps:', outputs)
    goals = [("AtHandleGrasp", 'left', body_joint, outputs[0][0])]
    return goals

def test_grasps(state, name='cabbage', visualize=True):
    if isinstance(name, str):
        body = state.world.name_to_body(name)
    else: ## if isinstance(name, Object):
        body = name
    funk = get_grasp_gen(state)
    outputs = funk(body)
    if visualize:
        body_pose = get_pose(body)
        visualize_grasps(state, outputs, body_pose)
    print('test_grasps:', outputs)
    goals = [("AtGrasp", 'left', body, outputs[1][0])]
    return goals

def visualize_grasps(state, outputs, body_pose, RETAIN_ALL=False):
    robot = state.robot
    colors = [BROWN, BLUE, WHITE, TAN, GREY, YELLOW, GREEN, BLACK, RED]
    for i in range(len(outputs)):
        grasp = outputs[i][0]
        if RETAIN_ALL:
            gripper_grasp = visualize_grasp(robot, body_pose, grasp.value, color=colors[i%len(colors)])
        else:
            gripper_grasp = visualize_grasp(robot, body_pose, grasp.value, color=GREEN)
            gripper_approach = visualize_grasp(robot, body_pose, grasp.approach, color=BROWN)
            # set_camera_target_body(gripper_approach, dx=0, dy=-1, dz=0)
            remove_body(gripper_grasp)
            remove_body(gripper_approach)
    if RETAIN_ALL:
        wait_if_gui()

def visualize_grasps_by_quat(state, outputs, body_pose, RETAIN_ALL=False):
    robot = state.robot
    colors = [BROWN, BLUE, WHITE, TAN, GREY, YELLOW, GREEN, BLACK, RED]
    all_grasps = {}
    for i in range(len(outputs)):
        grasp = outputs[i][0]
        quat = grasp.value[1]
        if quat not in all_grasps:
            all_grasps[quat] = []
        all_grasps[quat].append(grasp)

    j = 0
    for k, v in all_grasps.items():
        print(f'{len(v)} grasps of quat {k}')
        visuals = []
        for grasp in v:
            gripper_grasp = visualize_grasp(robot, body_pose, grasp.value, color=colors[j%len(colors)])
            visuals.append(gripper_grasp)
        j += 1
        wait_if_gui()
        for visual in visuals:
            remove_body(visual)


def test_grasp_ik(state, init, name='cabbage', visualize=True):
    goals = test_grasps(state, name, visualize=False)
    body, grasp = goals[0][-2:]
    robot = state.robot
    custom_limits = get_base_custom_limits(robot, BASE_LIMITS)
    pose = [i for i in init if i[0].lower() == "AtPose".lower() and i[1] == body][0][-1]

    wconfs = [i for i in init if "NewWConf".lower() in i[0].lower()]
    if len(wconfs) == 0:
        funk = get_ik_ir_gen(state, verbose=visualize, custom_limits=custom_limits
                             )('left', body, pose, grasp)
        print('test_grasp_ik', body, pose, grasp)
    else:
        wconf = wconfs[0][-1]
        print('test_grasp_ik', body, pose, grasp, wconf)
        wconf.printout()
        funk = get_ik_ir_wconf_gen(state, verbose=visualize, custom_limits=custom_limits
                             )('left', body, pose, grasp, wconf)

    next(funk)
    return goals

def test_pulling_handle_ik(problem):
    name_to_body = problem.name_to_body

    # ## --------- test pulling handle ik
    # for bq in [(1.583, 6.732, 0.848), (1.379, 7.698, -2.74), (1.564, 7.096, 2.781)]:
    #     custom_limits = get_base_custom_limits(problem.robot, BASE_LIMITS)
    #     robot = problem.robot
    #     drawer = name_to_body('hitman_drawer_top_joint')
    #     funk = get_handle_grasp_gen(problem)
    #     grasp = funk(drawer)[0][0]
    #     funk = get_pull_handle_motion_gen(problem)
    #     position1 = Position(name_to_body('hitman_drawer_top_joint'))
    #     position2 = Position(name_to_body('hitman_drawer_top_joint'), 'max')
    #     bq1 = Conf(robot, get_group_joints(robot, 'base'), bq)
    #     t = funk('left', drawer, position1, position2, grasp, bq1)

    ## --------- test modified pulling handle ik
    for bq in [(1.583, 6.732, 0.848), (1.379, 7.698, -2.74), (1.564, 7.096, 2.781)]:
        custom_limits = get_base_custom_limits(problem.robot, BASE_LIMITS)
        robot = problem.robot
        drawer = name_to_body('hitman_drawer_top_joint')
        funk = get_handle_grasp_gen(problem)
        grasp = funk(drawer)[0][0]
        funk = get_pull_drawer_handle_motion_gen(problem, extent='max')
        position1 = Position(name_to_body('hitman_drawer_top_joint'))
        position2 = Position(name_to_body('hitman_drawer_top_joint'), 'max')
        bq1 = Conf(robot, get_group_joints(robot, 'base'), bq)
        t = funk('left', drawer, grasp, bq1)
        break

def test_drawer_open(problem, goals):
    name_to_body = problem.name_to_body

    # -------- test joint position closed
    drawer = name_to_body('hitman_drawer_top_joint')
    get_open = get_position_gen(problem, extent='max')
    test_closed = get_joint_position_test(extent='min')
    test_open = get_joint_position_test(extent='max')
    position_open = get_open(drawer)[0]
    print('test-joint-position-closed', test_closed(drawer, Position(drawer)), not test_open(drawer, Position(drawer)))
    print('test-joint-position-open', test_open(drawer, goals[0][-1]), not test_closed(drawer, goals[0][-1]))
    print('get-joint-position-open', test_open(drawer, position_open), not test_closed(drawer, position_open))

def test_pose_gen(problem, init, o, s):
    pose = [i for i in init if i[0].lower() == "AtPose".lower() and i[1] == o][0][-1]
    if isinstance(o, Object):
        o = o.body
    funk = get_stable_gen(problem)(o, s)
    p = next(funk)[0]
    print(f'test_pose_gen({o}, {s}) | {p}')
    pose.assign()
    return [('AtPose', o, p)], [('Pose', o, p), ('Supported', o, p, s)]
