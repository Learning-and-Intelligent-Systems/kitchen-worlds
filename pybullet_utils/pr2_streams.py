from __future__ import print_function

import copy
import pybullet as p
import random
import time
import os
from itertools import islice, count
import math
import json

import numpy as np

from pybullet_tools.pr2_problems import get_fixed_bodies, create_pr2
from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, SIDE_HOLDING_LEFT_ARM, GET_GRASPS, get_gripper_joints, \
    get_carry_conf, get_top_grasps, get_side_grasps, open_arm, arm_conf, get_gripper_link, get_arm_joints, \
    learned_pose_generator, PR2_TOOL_FRAMES, get_x_presses, PR2_GROUPS, joints_from_names, \
    is_drake_pr2, get_group_joints, get_group_conf, compute_grasp_width, PR2_GRIPPER_ROOTS, \
    TOOL_POSE, MAX_GRASP_WIDTH, GRASP_LENGTH, SIDE_HEIGHT_OFFSET, approximate_as_prism, set_group_conf
from pybullet_tools.pr2_primitives import control_commands, apply_commands, get_stable_gen, Grasp, \
    APPROACH_DISTANCE, TOP_HOLDING_LEFT_ARM, get_tool_from_root, Conf, Commands, State, create_trajectory, \
    Trajectory
from pybullet_tools.ikfast.pr2.ik import is_ik_compiled, pr2_inverse_kinematics
from pybullet_tools.utils import invert, multiply, get_name, set_pose, get_link_pose, is_placement, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, waypoints_from_path, \
    unit_quat, plan_base_motion, plan_joint_motion, base_values_from_pose, pose_from_base_values, \
    uniform_pose_generator, sub_inverse_kinematics, add_fixed_constraint, remove_debug, remove_fixed_constraint, \
    disable_real_time, enable_gravity, joint_controller_hold, get_distance, Point, Euler, set_joint_position, \
    get_min_limit, user_input, step_simulation, get_body_name, get_bodies, BASE_LINK, get_joint_position, \
    add_segments, get_max_limit, link_from_name, BodySaver, get_aabb, Attachment, interpolate_poses, \
    plan_direct_joint_motion, has_gui, create_attachment, wait_for_duration, get_extend_fn, set_renderer, \
    get_custom_limits, all_between, get_unit_vector, wait_if_gui, create_box, set_point, quat_from_euler, \
    set_base_values, euler_from_quat, INF, elapsed_time, get_moving_links, flatten_links, get_relative_pose, \
    get_joint_limits, unit_pose, point_from_pose, clone_body, set_all_color, GREEN, BROWN, get_link_subtree, \
    RED, remove_body, aabb2d_from_aabb, aabb_overlap, aabb_contains_point, get_aabb_center, get_link_name, \
    get_links, check_initial_end, get_collision_fn, BLUE, WHITE, TAN, GREY, YELLOW, aabb_contains_aabb
from bullet.utils import sample_obj_in_body_link_space, nice, set_camera_target_body, is_contained
from bullet.worlds.utils import visualize_point

BASE_EXTENT = 3.5 # 2.5
BASE_LIMITS = (-BASE_EXTENT*np.ones(2), BASE_EXTENT*np.ones(2))
GRASP_LENGTH = 0.03
APPROACH_DISTANCE = 0.1 + GRASP_LENGTH
SELF_COLLISIONS = False
LINK_POSE_TO_JOINT_POSITION = {}

class Position(object):
    num = count()
    def __init__(self, body, value=None):
        self.body, self.joint = body
        if value is None:
            value = get_joint_position(self.body, self.joint)
        elif value == 'max':
            value = self.get_limits()[1]
        elif value == 'min':
            value = self.get_limits()[0]
        self.value = value
        self.index = next(self.num)
    @property
    def bodies(self):
        return flatten_links(self.body)
    @property
    def extent(self):
        if self.value == self.get_limits()[1]:
            return 'max'
        elif self.value == self.get_limits()[0]:
            return 'min'
        return 'middle'
    def assign(self):
        set_joint_position(self.body, self.joint, self.value)
    def iterate(self):
        yield self
    def get_limits(self):
        return get_joint_limits(self.body, self.joint)
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'pstn{}={}'.format(index, nice(self.value))

class LinkPose(object):
    num = count()
    def __init__(self, body, obj, value=None, support=None, init=False):
        self.obj = obj
        self.link = self.obj.handle_link
        self.body, self.joint = body
        if value is None:
            value = get_link_pose(self.body, self.link)
        self.value = tuple(value)
        self.body_pose = get_pose(self.body)
        self.support = support
        self.init = init
        self.index = next(self.num)
    @property
    def bodies(self):
        return flatten_links(self.body)
    def assign(self):
        pass
    def iterate(self):
        yield self
    # def to_base_conf(self):
    #     values = base_values_from_pose(self.value)
    #     return Conf(self.body, range(len(values)), values)
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'lp{}={}'.format(index, nice(self.value))
        # return 'p{}'.format(index)

class HandleGrasp(object):
    def __init__(self, grasp_type, body, value, approach, carry):
        self.grasp_type = grasp_type
        self.body = body
        self.value = tuple(value) # gripper_from_object
        self.approach = tuple(approach)
        self.carry = tuple(carry)
    def get_attachment(self, robot, arm):
        tool_link = link_from_name(robot, PR2_TOOL_FRAMES[arm])
        return Attachment(robot, tool_link, self.value, self.body)
    def __repr__(self):
        return 'hg{}={}'.format(id(self) % 1000, nice(self.value))

class WConf(object):
    def __init__(self, poses, positions):
        self.poses = poses
        self.positions = positions
    def assign(self):
        for p in self.poses.values():
            p.assign()
        for p in self.positions.values():
            p.assign()
    def printout(self, obstacles=None):
        if obstacles == None:
            obstacles = list(self.poses.keys())
            positions = list(self.positions.keys())
        else:
            positions = [o for o in self.positions.keys() if o[0] in obstacles]

        string = f"  {str(self)}"
        poses = {o: nice(self.poses[o].value[0]) for o in obstacles if o in self.poses}
        if len(poses) > 0:
            string += f'\t|\tposes: {str(poses)}'
        positions = {o: nice(self.positions[(o[0], o[1])].value) for o in positions}
        if len(positions) > 0:
            string += f'\t|\tpositions: {str(positions)}'
        # print(string)
        return string

    def __repr__(self):
        return 'wconf{}({})'.format(id(self) % 1000, len(self.poses))


##################################################
def get_stable_gen(problem, collisions=True, num_trials=20, **kwargs):
    from pybullet_tools.pr2_primitives import Pose
    obstacles = problem.fixed if collisions else []
    world = problem.world
    def gen(body, surface):
        if surface is None:
            surfaces = problem.surfaces
        else:
            surfaces = [surface]
        count = num_trials
        while count > 0: ## True
            count -= 1
            surface = random.choice(surfaces) # TODO: weight by area
            if isinstance(surface, tuple): ## (body, link)
                body_pose = sample_placement(body, surface[0], bottom_link=surface[-1], **kwargs)
            else:
                body_pose = sample_placement(body, surface, **kwargs)
            if body_pose is None:
                break

            ## hack to reduce planning time
            body_pose = learned_pose_sampler(world, body, surface, body_pose)

            p = Pose(body, body_pose, surface)
            p.assign()
            if not any(pairwise_collision(body, obst) for obst in obstacles if obst not in {body, surface}):
                yield (p,)
    return gen

def learned_pose_sampler(world, body, surface, body_pose):
    ## hack to reduce planning time
    if 'eggblock' in world.BODY_TO_OBJECT[body].name and 'braiser_bottom' in world.BODY_TO_OBJECT[surface].name:
        (x, y, z), quat = body_pose
        x = 0.55
        body_pose = (x, y, z), quat
    return body_pose

def get_contain_gen(problem, collisions=True, max_attempts=20, verbose=False, **kwargs):
    from pybullet_tools.pr2_primitives import Pose
    obstacles = problem.fixed if collisions else []

    def gen(body, space):
        if space is None:
            spaces = problem.spaces
        else:
            spaces = [space]
        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            space = random.choice(spaces)  # TODO: weight by area
            if isinstance(space, tuple):
                x, y, z, yaw = sample_obj_in_body_link_space(body, space[0], space[-1],
                                        PLACEMENT_ONLY=True, verbose=verbose, **kwargs)
                body_pose = ((x, y, z), quat_from_euler(Euler(yaw=yaw)))
            else:
                body_pose = None
            if body_pose is None:
                break
            p = Pose(body, body_pose, space)
            p.assign()
            if not any(pairwise_collision(body, obst) for obst in obstacles if obst not in {body, space}):
                yield (p,)
        if verbose:
            print(f'  get_contain_gen | reached max_attempts = {max_attempts}')
        yield None
    return gen

def get_pose_in_space_test():
    def test(o, p, r):
        p.assign()
        answer = is_contained(o, r)
        print(f'pr2_streams.get_pose_in_space_test({o}, {p}, {r}) = {answer}')
        return answer
    return test

########################################################################

def get_joint_position_open_gen(problem):
    def fn(o, psn1, fluents=[]):  ## ps1,
        if psn1.extent == 'max':
            psn2 = Position(o, 'min')
        elif psn1.extent == 'min':
            psn2 = Position(o, 'max')
        return (psn2,)
    return fn

def sample_joint_position_open_list_gen(problem, num_samples = 3):
    def fn(o, psn1, fluents=[]):
        psn2 = None
        if psn1.extent == 'max':
            psn2 = Position(o, 'min')
            higher = psn1.value
            lower = psn2.value
        elif psn1.extent == 'min':
            psn2 = Position(o, 'max')
            higher = psn2.value
            lower = psn1.value
        else:
            # return [(psn1, )]
            higher = Position(o, 'max').value
            lower = Position(o, 'min').value
            if lower > higher:
                sometime = lower
                lower = higher
                higher = sometime

        positions = []
        if psn2 == None or abs(psn1.value - psn2.value) > math.pi/2:
            # positions.append((Position(o, lower+math.pi/2), ))
            lower += math.pi/2
            higher = lower + math.pi/8
            ptns = [np.random.uniform(lower, higher) for k in range(num_samples)]
            ptns.append(1.77)
            positions.extend([(Position(o, p), ) for p in ptns])
        else:
            positions.append((psn2,))

        return positions
    return fn

## discarded
def get_position_gen(problem, collisions=True, extent=None):
    obstacles = problem.fixed if collisions else []
    def fn(o, fluents=[]):  ## ps1,
        ps2 = Position(o, extent)
        return (ps2,)
    return fn

## discarded
def get_joint_position_test(extent='max'):
    def test(o, pst):
        pst_max = Position(o, extent)
        if pst_max.value == pst.value:
            return True
        return False
    return test

########################################################################

def pr2_grasp(body, value, grasp_type=None):
    if grasp_type == None:
        euler = euler_from_quat(value[1])
        grasp_type = 'top'
        if euler[0] == euler[1] == 0:
            grasp_type = 'side'
    approach_vector = APPROACH_DISTANCE * get_unit_vector([1, 0, 0])
    return Grasp(grasp_type, body, value, multiply((approach_vector, unit_quat()), value),
                 TOP_HOLDING_LEFT_ARM)

def get_handle_grasps(joint_object, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                      max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH,
                      robot=None, obstacles=[]):
    from pybullet_tools.utils import Pose, get_joints, is_movable

    PI = math.pi
    db_file = os.path.dirname(os.path.abspath(__file__))
    db_file = os.path.join(db_file,'handle_grasps.json')
    db = json.load(open(db_file, 'r'))
    name = joint_object.shorter_name
    if name in db:
        return [(tuple(e[0]), tuple(e[1])) for e in db[name]]

    handle_pose = joint_object.get_handle_pose()

    def check_cfree_gripper(joint_object, grasp, visualize=False, color=GREEN):
        gripper_grasp = visualize_grasp(robot, handle_pose, grasp, color=color)
        # if visualize:
        #     set_camera_target_body(gripper_grasp, dx=0, dy=1, dz=0)

        result = True
        ## when gripper isn't closed, it shouldn't collide
        if any(pairwise_collision(gripper_grasp, b) for b in obstacles):
            result = False

        ## when gripper is closed, it should collide with object
        joints = [joint for joint in get_joints(gripper_grasp) if is_movable(gripper_grasp, joint)]
        set_joint_positions(gripper_grasp, joints, [0] * 4)
        if not pairwise_collision(gripper_grasp, joint_object.body):
            result = False

        remove_body(gripper_grasp)
        return result

    # TODO: compute bounding box width wrt tool frame
    center, (w, l, h) = approximate_as_prism(joint_object.body, body_pose=body_pose, link=joint_object.handle_link)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    num_h = 4
    top_offset = h / num_h
    gl = grasp_length / 3

    grasps = []
    rots = [[0,0,0], [0,0,PI/2], [0,0,PI], [0,PI/2,0], [0,PI,0], [PI/2,0,0], [PI,0,0]]
    title = f'pr2_streams.get_handle_grasps({name}): '
    for i in range(len(rots)):
        for j in range(len(rots)):
            r1 = Pose(euler=rots[i])
            r2 = Pose(euler=rots[j])
            for gh in range(0, num_h):
                translate_z = Pose(point=[gh*top_offset, 0, w / 2 - gl])
                grasp = multiply(tool_pose, translate_z, r1, r2, translate_center, body_pose)
                result = check_cfree_gripper(joint_object, grasp)
                print(f'{title}test grasp ({i*len(rots)+j}/{len(rots)**2}, {gh}), {result}')
                if result:
                    if grasp not in grasps:
                        grasps += [grasp]
                else:
                    break

    db[name] = grasps
    os.remove(db_file)
    with open(db_file, 'w') as f:
        json.dump(db, f, indent=4)
    return grasps

def get_handle_grasp_gen(problem, collisions=False, randomize=True, visualize=False):
    collisions = True
    obstacles = problem.fixed if collisions else []
    BODY_TO_OBJECT = problem.world.BODY_TO_OBJECT
    name_to_body = problem.world.name_to_body
    def fn(body):
        # TODO: max_grasps
        # TODO: return grasps one by one
        grasps = []
        joint = BODY_TO_OBJECT[body]
        body_pose = joint.get_handle_pose()
        #carry_conf = get_carry_conf(arm, 'top')
        approach_vector = APPROACH_DISTANCE * get_unit_vector([1, 0, 0]) ##[2, 0, -1])
        # grasps.extend(HandleGrasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM)
        #               for g in get_handle_grasps(joint, grasp_length=GRASP_LENGTH))  ## , body_pose=body_pose
        grasps.extend(HandleGrasp('side', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM)
                      for g in get_handle_grasps(joint, grasp_length=GRASP_LENGTH,
                                                 robot=problem.robot, obstacles=obstacles))  ## , body_pose=body_pose

        for grasp in grasps:
            grasp.grasp_width = joint.handle_width

        # if randomize:
        #     random.shuffle(grasps)
        return [(g,) for g in grasps]
        #for g in filtered_grasps:
        #    yield (g,)
    return fn

def linkpose_from_position(pose, world):
    pose.assign()
    joint = world.BODY_TO_OBJECT[(pose.body, pose.joint)]
    pose_value = get_link_pose(joint.body, joint.handle_link)
    return pose_value ## LinkPose(pose.body, joint, pose_value)

def iterate_approach_path(robot, arm, gripper, pose_value, grasp, body=None):
    tool_from_root = get_tool_from_root(robot, arm)
    grasp_pose = multiply(pose_value, invert(grasp.value))
    approach_pose = multiply(pose_value, invert(grasp.approach))
    for tool_pose in interpolate_poses(grasp_pose, approach_pose):
        set_pose(gripper, multiply(tool_pose, tool_from_root))
        # if body is not None:
        #     set_pose(body, multiply(tool_pose, grasp.value))
        yield

def get_ir_sampler(problem, custom_limits={}, max_attempts=40, collisions=True,
                   learned=True, verbose=False):
    robot = problem.robot
    world = problem.world
    obstacles = [o for o in problem.fixed if o not in problem.floors] if collisions else []
    gripper = problem.get_gripper(visual=False)
    heading = f'   pr2_streams.get_ir_sampler | '
    def gen_fn(arm, obj, pose, grasp):

        pose.assign()
        if isinstance(obj, tuple): ## may be a (body, joint) or a body with a marker
            obj = obj[0]
        if isinstance(pose, Position):
            pose_value = linkpose_from_position(pose, world)
        else:
            pose_value = pose.value

        approach_obstacles = problem.world.refine_marker_obstacles(obj, obstacles)
        ## {obst for obst in obstacles if obst !=obj} ##{obst for obst in obstacles if not is_placement(obj, obst)}
        if set(obstacles) != set(approach_obstacles):
            print(f'approach_obstacles = {approach_obstacles}')

        for _ in iterate_approach_path(robot, arm, gripper, pose_value, grasp):
            # if verbose:
            for b in approach_obstacles:
                if pairwise_collision(gripper, b):
                    if verbose:
                        print(f'{heading} in approach, gripper {nice(get_pose(gripper))} collide with {b} {nice(get_pose(b))}')
                    return
                if obj == b: continue
                # if pairwise_collision(obj, b):
                #     print(f'       get_ir_sampler | in approach, obj {obj} at {nice(get_pose(obj))} with obj {b} at {nice(get_pose(b))}')
                #     return
            # else:
            #     if any([pairwise_collision(gripper, b) or pairwise_collision(obj, b) for b in approach_obstacles]):
            #         return
        gripper_pose = multiply(pose_value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
        default_conf = arm_conf(arm, grasp.carry)
        arm_joints = get_arm_joints(robot, arm)
        base_joints = get_group_joints(robot, 'base')
        if learned:
            base_generator = learned_pose_generator(robot, gripper_pose, arm=arm, grasp_type=grasp.grasp_type)
        else:
            base_generator = uniform_pose_generator(robot, gripper_pose)
        lower_limits, upper_limits = get_custom_limits(robot, base_joints, custom_limits)
        aconf = nice(get_joint_positions(robot, arm_joints))
        while True:
            count = 0
            for base_conf in islice(base_generator, max_attempts):
                count += 1
                if not all_between(lower_limits, base_conf, upper_limits):
                    continue
                bq = Conf(robot, base_joints, base_conf)
                pose.assign()
                bq.assign()
                set_joint_positions(robot, arm_joints, default_conf)
                if any(pairwise_collision(robot, b) for b in obstacles + [obj]):
                    continue
                if verbose:
                    print(f'{heading} IR attempt {count} | bconf = {nice(base_conf)}, aconf = {aconf}')
                # wconf = problem.get_wconf()
                # wconf.printout()
                # print(f'pr2_streams.get_ir_sampler() -> {bq} is c-free from {obstacles + [obj]}')
                yield (bq,)
                break
            else:
                yield None
    return gen_fn

##################################################

def get_ik_fn(problem, custom_limits={}, collisions=True, teleport=False, verbose=False, ACONF=False):
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    world = problem.world
    title = 'pr2_streams.get_ik_fn:\t'

    def fn(arm, obj, pose, grasp, base_conf):
        if isinstance(obj, tuple): ## may be a (body, joint) or a body with a marker
            obj = obj[0]
        if isinstance(pose, Position):
            pose_value = linkpose_from_position(pose, world)
        else:
            pose_value = pose.value

        addons = [obj]
        if world.BODY_TO_OBJECT[obj].grasp_parent != None:
            addons.append(world.BODY_TO_OBJECT[obj].grasp_parent)

        approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        # approach_obstacles = problem.world.refine_marker_obstacles(obj, approach_obstacles)  ## for steerables

        gripper_pose = multiply(pose_value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
        #approach_pose = multiply(grasp.approach, gripper_pose)
        approach_pose = multiply(pose_value, invert(grasp.approach))

        arm_link = get_gripper_link(robot, arm)
        arm_joints = get_arm_joints(robot, arm)

        default_conf = arm_conf(arm, grasp.carry)
        #sample_fn = get_sample_fn(robot, arm_joints)
        pose.assign()
        base_conf.assign()
        open_arm(robot, arm)
        set_joint_positions(robot, arm_joints, default_conf) # default_conf | sample_fn()
        grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose, custom_limits=custom_limits) #, upper_limits=USE_CURRENT)
                                            #nearby_conf=USE_CURRENT) # upper_limits=USE_CURRENT,
        if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles+addons): ## approach_obstacles): # [obj]
            if verbose:
                if grasp_conf != None:
                    grasp_conf = nice(grasp_conf)
                print(f'{title}Grasp IK failure | {grasp_conf} = pr2_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(gripper_pose[0])}) | pose = {pose}, grasp = {grasp}')
                for b in obstacles+addons:
                    if pairwise_collision(robot, b):
                        # set_renderer(True)
                        print(f'                        robot at {nice(base_conf.values)} colliding with {b} at {nice(get_pose(b))}')
            #if grasp_conf is not None:
            #    print(grasp_conf)
            #    #wait_if_gui()
            return None
        elif verbose:
            print(f'{title}Grasp IK success | {nice(grasp_conf)} = pr2_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(gripper_pose[0])}) | pose = {pose}, grasp = {grasp}')
        #approach_conf = pr2_inverse_kinematics(robot, arm, approach_pose, custom_limits=custom_limits,
        #                                       upper_limits=USE_CURRENT, nearby_conf=USE_CURRENT)
        approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose, custom_limits=custom_limits)
        if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles + addons): ##
            if verbose:
                if approach_conf != None:
                    approach_conf = nice(approach_conf)
                print(f'{title}Approach IK failure', approach_conf)
                for b in obstacles + addons:
                    if pairwise_collision(robot, b):
                        print(f'                        robot at {nice(base_conf.values)} colliding with {b} at {nice(get_pose(b))}')
            # wait_if_gui()
            return None
        elif verbose:
            print(f'{title}Approach IK success | sub_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(approach_pose[0])}) | pose = {pose}, grasp = {nice(grasp.approach)} -> {nice(approach_conf)}')

        # ## -------------------------------------------
        # arm_joints = get_arm_joints(robot, 'left')
        # aconf = Conf(robot, arm_joints, get_joint_positions(robot, arm_joints))
        # print(f'@ pr2_streams.get_ik_fn() -> aconf = {aconf} | bconf = {base_conf}')
        # ## -------------------------------------------

        approach_conf = get_joint_positions(robot, arm_joints)
        attachment = grasp.get_attachment(problem.robot, arm)
        attachments = {}  ## {attachment.child: attachment} TODO: problem with having (body, joint) tuple
        if teleport:
            path = [default_conf, approach_conf, grasp_conf]
        else:
            resolutions = 0.05**np.ones(len(arm_joints))
            grasp_path = plan_direct_joint_motion(robot, arm_joints, grasp_conf, attachments=attachments.values(),
                                                  obstacles=approach_obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits=custom_limits, resolutions=resolutions/2.)
            if grasp_path is None:
                if verbose: print(f'{title}Grasp path failure')
                return None
            set_joint_positions(robot, arm_joints, default_conf)
            approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                              obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                              custom_limits=custom_limits, resolutions=resolutions,
                                              restarts=2, iterations=25, smooth=25)
            if approach_path is None:
                if verbose: print(f'{title}\tApproach path failure')
                return None
            path = approach_path + grasp_path
        mt = create_trajectory(robot, arm_joints, path)
        attachments = {attachment.child: attachment} ## TODO: problem with having (body, joint) tuple
        cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])

        set_joint_positions(robot, arm_joints, default_conf)  # default_conf | sample_fn()

        if ACONF:
            return (mt.path[-1], cmd)
        return (cmd,)
    return fn


##################################################

def get_ik_ir_wconf_gen(problem, max_attempts=25, learned=True, teleport=False,
                        verbose=False, visualize=False, **kwargs):
    # TODO: compose using general fn
    ir_max_attempts = 20
    ir_sampler = get_ir_sampler(problem, learned=learned, max_attempts=ir_max_attempts, verbose=verbose, **kwargs)
    ik_fn = get_ik_fn(problem, teleport=teleport, verbose=False, **kwargs)
    robot = problem.robot
    heading = 'pr2_streams.get_ik_ir_wconf_gen | '
    def gen(*inputs):
        # set_renderer(enable=True)
        if visualize:
            set_renderer(enable=True)
            samples = []

        a, o, p, g, w = inputs

        # ## --- make sure the bconf is cfree in default arm position ------
        # arm_joints = get_arm_joints(robot, a)
        # default_conf = arm_conf(a, g.carry)
        # set_joint_positions(robot, arm_joints, default_conf)
        # ## ---------------------------------------------------------------

        w.assign()
        # w.printout()
        inputs = a, o, p, g
        ir_generator = ir_sampler(*inputs)
        attempts = 0
        while True:
            if max_attempts <= attempts:
                if not p.init:
                    return
                attempts = 0
                print(f'{heading} exceeding max_attempts = {max_attempts}')
                yield None

            attempts += 1
            if verbose: print(f'   {attempts} | get_ik_ir_wconf_gen | inputs = {inputs}')

            try:
                ir_outputs = next(ir_generator)
            except StopIteration:
                if verbose: print('    stopped ir_generator in', attempts, 'attempts')
                print(f'{heading} exceeding ir_generator ir_max_attempts = {ir_max_attempts}')
                return

            if ir_outputs is None:
                continue
            inp = ir_generator.gi_frame.f_locals
            inp = [inp[k] for k in ['pose', 'grasp', 'custom_limits']]
            if verbose:
                print(f'           ir_generator  |  inputs = {inp}  |  ir_outputs = {ir_outputs}')

            if visualize:
                samp = create_box(.1, .1, .1, mass=1, color=(1, 0, 1, 1))
                x,y,_ = ir_outputs[0].values
                set_point(samp, (x,y,0.2))
                samples.append(samp)

            ik_outputs = ik_fn(*(inputs + ir_outputs))
            if ik_outputs is None:
                continue
            if verbose: print('succeed after IK attempts:', attempts)

            if visualize:
                for samp in samples:
                    remove_body(samp)
            yield ir_outputs + ik_outputs
            return
            #if not p.init:
            #    return
    return gen

##################################################

def get_ik_ir_grasp_handle_gen(problem, max_attempts=40, learned=True, teleport=False,
                               verbose=False, ACONF=False, WCONF=False, **kwargs):
    # TODO: compose using general fn
    # ir_sampler = get_ir_sampler(problem, learned=learned, max_attempts=1, **kwargs)
    ir_sampler = get_ir_sampler(problem, learned=learned, max_attempts=40, **kwargs)
    ik_fn = get_ik_fn(problem, teleport=teleport, ACONF=ACONF, **kwargs)
    def gen(*inputs):
        if verbose: set_renderer(enable=True)
        if WCONF:
            b, a, p, g, w = inputs
            w.assign()
            inputs = b, a, p, g
        else:
            b, a, p, g = inputs

        ir_generator = ir_sampler(*inputs)
        attempts = 0
        while True:
            if max_attempts <= attempts:
                # if not p.init:
                #     return
                attempts = 0
                yield None
            attempts += 1
            if verbose: print(f'   {attempts} | get_ik_ir_gen | inputs = {inputs}')

            try:
                ir_outputs = next(ir_generator)
            except StopIteration:
                if verbose: print('    stopped ir_generator in', attempts, 'attempts')
                return

            if ir_outputs is None:
                continue
            inp = ir_generator.gi_frame.f_locals
            inp = [inp[k] for k in ['pose', 'grasp', 'custom_limits']]
            if verbose:
                print(f'           ir_generator  |  inputs = {inp}  |  ir_outputs = {ir_outputs}')
                samp = create_box(.1, .1, .1, mass=1, color=(1, 0, 1, 1))
                x,y,_ = ir_outputs[0].values
                set_point(samp, (x,y,0.2))

            ik_outputs = ik_fn(*(inputs + ir_outputs))
            if ik_outputs is None:
                continue
            # print('                         ik_outputs = ik_fn(*(inputs + ir_outputs)) =', ik_outputs, ' | commands =', ik_outputs[0].commands)
            if verbose: print('succeed after IK attempts:', attempts)
            yield ir_outputs + ik_outputs
            return
            #if not p.init:
            #    return
    return gen

##################################################

def get_arm_ik_fn(problem, custom_limits={}, collisions=True, teleport=False, verbose=False):
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    world = problem.world
    title = 'pr2_streams.get_arm_ik_fn:\t'

    def fn(arm, obj, pose, grasp, base_conf, grasp_conf):
        if isinstance(obj, tuple): ## may be a (body, joint) or a body with a marker
            obj = obj[0]
        if isinstance(pose, Position):
            pose_value = linkpose_from_position(pose, world)
        else:
            pose_value = pose.value

        addons = [obj]
        if world.BODY_TO_OBJECT[obj].grasp_parent != None:
            addons.append(world.BODY_TO_OBJECT[obj].grasp_parent)

        # approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        approach_obstacles = {o for o in obstacles if o not in addons}
        # approach_obstacles = problem.world.refine_marker_obstacles(obj, approach_obstacles)  ## for steerables

        gripper_pose = multiply(pose_value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
        approach_pose = multiply(pose_value, invert(grasp.approach))
        arm_link = get_gripper_link(robot, arm)
        arm_joints = get_arm_joints(robot, arm)

        default_conf = arm_conf(arm, grasp.carry)
        pose.assign()
        base_conf.assign()
        open_arm(robot, arm)
        grasp_conf = grasp_conf.values
        set_joint_positions(robot, arm_joints, grasp_conf) # default_conf | sample_fn()
        # grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose, custom_limits=custom_limits) #, upper_limits=USE_CURRENT)
        #                                     #nearby_conf=USE_CURRENT) # upper_limits=USE_CURRENT,
        if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles): ## approach_obstacles): # [obj]
            if verbose:
                if grasp_conf != None:
                    grasp_conf = nice(grasp_conf)
                print(f'{title}Grasp IK failure | {grasp_conf} = pr2_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(gripper_pose[0])}) | pose = {pose}, grasp = {grasp}')
                for b in obstacles:
                    if pairwise_collision(robot, b):
                        # set_renderer(True)
                        print(f'                        robot at {nice(base_conf.values)} colliding with {b} at {nice(get_pose(b))}')
            return None
        else:
            if verbose:
                print(f'{title}Grasp IK success | {nice(grasp_conf)} = pr2_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(gripper_pose[0])}) | pose = {pose}, grasp = {grasp}')

        approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose, custom_limits=custom_limits) ##, max_iterations=500
        if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles): ##
            if verbose:
                if approach_conf != None:
                    approach_conf = nice(approach_conf)
                print(f'{title}Approach IK failure | sub_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(approach_pose[0])}) | pose = {pose}, grasp = {nice(grasp.approach)} -> {approach_conf}')
                for b in obstacles:
                    if pairwise_collision(robot, b):
                        print(f'                        robot at {nice(base_conf.values)} colliding with {b} at {nice(get_pose(b))}')
            #wait_if_gui()
            return None
        else:
            if verbose:
                print(f'{title}Approach IK success | sub_inverse_kinematics({robot} at {nice(base_conf.values)}, {arm}, {nice(approach_pose[0])}) | pose = {pose}, grasp = {nice(grasp.approach)} -> {nice(approach_conf)}')

        approach_conf = get_joint_positions(robot, arm_joints)
        attachment = grasp.get_attachment(problem.robot, arm)
        attachments = {}  ## {attachment.child: attachment} TODO: problem with having (body, joint) tuple
        if teleport:
            path = [default_conf, approach_conf, grasp_conf]
        else:
            resolutions = 0.05**np.ones(len(arm_joints))
            grasp_path = plan_direct_joint_motion(robot, arm_joints, grasp_conf, attachments=attachments.values(),
                                                  obstacles=approach_obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits=custom_limits, resolutions=resolutions/2.)
            if grasp_path is None:
                if verbose: print(f'{title}Grasp path failure')
                return None
            set_joint_positions(robot, arm_joints, default_conf)
            approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                              obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                              custom_limits=custom_limits, resolutions=resolutions,
                                              restarts=2, iterations=25, smooth=25)
            if approach_path is None:
                if verbose: print(f'{title}Approach path failure')
                return None
            path = approach_path + grasp_path
        mt = create_trajectory(robot, arm_joints, path)
        attachments = {attachment.child: attachment} ## TODO: problem with having (body, joint) tuple
        cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
        return (mt.path[-1], cmd)
    return fn

def get_ik_ungrasp_handle_gen(problem, max_attempts=25, teleport=False, WCONF=False, **kwargs):
    ik_fn = get_arm_ik_fn(problem, teleport=teleport, **kwargs)
    def gen(*inputs):
        if WCONF:
            a, o, p, g, q, aq1, w = inputs
            w.assign()
            inputs = a, o, p, g, q, aq1
        # return ik_fn(*(inputs))
        attempts = 0
        while True:
            if max_attempts <= attempts:
                return None
            yield ik_fn(*(inputs))
            return
    return gen

def get_ik_ungrasp_mark_gen(problem, max_attempts=25, teleport=False, **kwargs):
    ik_fn = get_ik_fn(problem, teleport=teleport, **kwargs)
    def gen(*inputs):
        return ik_fn(*(inputs))
        # attempts = 0
        # while True:
        #     if max_attempts <= attempts:
        #         return None
        #     yield ik_fn(*(inputs))
        #     return
    return gen

##################################################

def bconf_to_pose(bq):
    from pybullet_tools.utils import Pose
    x,y,yaw = bq.values
    return Pose(point=Point(x,y,0), euler=Euler(yaw=yaw))

def pose_to_bconf(rpose, robot):
    (x, y, z), quant = rpose
    yaw = euler_from_quat(quant)[-1]
    return Conf(robot, get_group_joints(robot, 'base'), (x, y, yaw))

def add_pose(p1, p2):
    point = np.asarray(p1[0]) + np.asarray(p2[0])
    euler = np.asarray(euler_from_quat(p1[1])) + np.asarray(euler_from_quat(p2[1]))
    return (tuple(point.tolist()), quat_from_euler(tuple(euler.tolist())))

def sample_new_bconf(bq1):
    limits = [0.05] * 3
    def rand(limit):
        return np.random.uniform(-limit, limit)
    values = (bq1.values[i] + rand(limits[i]) for i in range(len(limits)))
    return Conf(bq1.body, bq1.joints, values)

def get_pull_door_handle_motion_gen(problem, custom_limits={}, collisions=True, teleport=False,
                                    num_intervals=30, max_ir_trial=30, visualize=False, verbose=False):
    if teleport:
        num_intervals = 1
    robot = problem.robot
    world = problem.world
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    BODY_TO_OBJECT = problem.world.BODY_TO_OBJECT
    def fn(a, o, pst1, pst2, g, bq1, aq1, fluents=[]):
        if pst1.value == pst2.value:
            return None

        saver.restore()
        pst1.assign()
        bq1.assign()
        aq1.assign()

        arm_joints = get_arm_joints(robot, a)
        resolutions = 0.05 ** np.ones(len(arm_joints))

        joint_object = BODY_TO_OBJECT[o]
        old_pose = get_link_pose(joint_object.body, joint_object.handle_link)
        tool_from_root = get_tool_from_root(robot, a)
        if visualize:
            set_renderer(enable=True)
            gripper_before = visualize_grasp(robot, old_pose, g.value)
        gripper_before = multiply(old_pose, invert(g.value))  ## multiply(, tool_from_root)
        world_from_base = bconf_to_pose(bq1)
        gripper_from_base = multiply(invert(gripper_before), world_from_base)
        # print('gripper_before', nice(gripper_before))
        # print('invert(gripper_before)', nice(invert(gripper_before)))

        MOVE_BASE = True

        ## saving the mapping between robot bconf to object pst for execution
        mapping = {}
        rpose_rounded = tuple([round(n, 3) for n in bq1.values])
        mapping[rpose_rounded] = pst1.value

        bpath = []
        apath = []
        bq_after = Conf(bq1.body, bq1.joints, bq1.values)
        aq_after = Conf(aq1.body, aq1.joints, aq1.values)
        for i in range(num_intervals):
            step_str = f"pr2_streams.get_pull_door_handle_motion_gen | step {i}/{num_intervals}\t"
            value = (i + 1) / num_intervals * (pst2.value - pst1.value) + pst1.value
            pst_after = Position((pst1.body, pst1.joint), value)
            pst_after.assign()
            new_pose = get_link_pose(joint_object.body, joint_object.handle_link)
            if visualize:
                gripper_after = visualize_grasp(robot, new_pose, g.value, color=BROWN)
                set_camera_target_body(gripper_after, dx=0.2, dy=0, dz=1) ## look top down
                remove_body(gripper_after)
            gripper_after = multiply(new_pose, invert(g.value))  ## multiply(, tool_from_root)

            ## try to transform the base the same way as gripper to a cfree pose
            if MOVE_BASE:
                world_from_base = multiply(gripper_after, gripper_from_base)
                bq_after = pose_to_bconf(world_from_base, robot)

                bq_after.assign()
                if any(pairwise_collision(robot, b) for b in obstacles):
                    collided = []
                    for b in obstacles:
                        if pairwise_collision(robot, b):
                            collided.append(b)
                    collided = [world.BODY_TO_OBJECT[c].shorter_name for c in collided]
                    print(f'{step_str} base collide at {nice(world_from_base)} with {collided}')
                    MOVE_BASE = False
                    if len(bpath) > 1:
                        bpath[-1].assign()
                else:
                    bpath.append(bq_after)
                    apath.append(aq_after)
                    if verbose: print(f'{step_str} : {nice(bq_after.values)}\t{nice(aq_after.values)}')

            ## move the arm with IK
            if not MOVE_BASE and False:
                aq_after.assign()
                arm_conf = pr2_inverse_kinematics(robot, a, gripper_after)
                trial = 0
                while arm_conf is None: ## need to shift bq a bit
                    trial += 1
                    if trial > max_ir_trial:
                        break
                    bq_proposed = sample_new_bconf(bq_after)
                    bq_proposed.assign()
                    print(f'{step_str} Cant find arm_conf ({trial}) at {nice(bq_after.values)}, try {nice(bq_proposed.values)}')
                    arm_conf = pr2_inverse_kinematics(robot, a, gripper_after)
                    if arm_conf is not None:
                        collision_fn = get_collision_fn(robot, arm_joints, obstacles, self_collisions=SELF_COLLISIONS)
                        if collision_fn(arm_conf):
                            bq_after = bq_proposed
                            bpath.append(bq_after)
                            apath.append(aq_after)
                            print(f'{step_str} : {nice(bq_after.values)}\t{nice(aq_after.values)}')
                            break
                if trial > max_ir_trial:
                    break
                aq_after.assign()
                path = plan_joint_motion(robot, arm_joints, arm_conf, obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                         resolutions=resolutions, restarts=2, max_iterations=25, smooth=25)
                if path is None:
                    print(f'{step_str} Cant find arm path from {nice(aq_after.values)} to {nice(arm_conf)}')
                    break
                # elif len(path) > num_intervals/2:
                #     print(f'{step_str} adding arm_path of len ({len(path)})')
                blank = ''.join([' ']*90)
                path_str = f"\n{blank}".join([str(nice(ac)) for ac in path])
                print(f'{step_str} adding arm_path of len ({len(path)}) {path_str}')
                apath += create_trajectory(robot, arm_joints, path[1:]).path
                bpath += [bq_after] * len(path)
                aq_after = apath[-1]

            rpose_rounded = tuple([round(n, 3) for n in bq_after.values])
            mapping[rpose_rounded] = value

        if visualize:
            remove_body(gripper_before)

        if len(apath) < num_intervals: ## * 0.75:
            return None

        body, joint = o
        if body not in LINK_POSE_TO_JOINT_POSITION:
            LINK_POSE_TO_JOINT_POSITION[body] = {}
        # mapping = sorted(mapping.items(), key=lambda kv: kv[1])
        LINK_POSE_TO_JOINT_POSITION[body][joint] = mapping
        # print(f'pr2_streams.get_pull_door_handle_motion_gen | last bconf = {rpose_rounded}, pstn value = {value}')

        # apath.append(apath[-1])
        # bpath.append(bpath[-1]) ## replicate the last one because somehow it's missing
        bt = Trajectory(bpath)
        at = Trajectory(apath) ## create_trajectory(robot, get_arm_joints(robot, a), apath)
        base_cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
        arm_cmd =  Commands(State(), savers=[BodySaver(robot)], commands=[at])
        bq2 = bt.path[-1]
        aq2 = at.path[-1]
        if aq2.values == aq1.values:
            aq2 = aq1
        step_str = f"pr2_streams.get_pull_door_handle_motion_gen | step {len(bpath)}/{num_intervals}\t"
        if not verbose: print(f'{step_str} : {nice(bq2.values)}\t{nice(aq2.values)}')
        return (bq2, base_cmd, aq2, arm_cmd)

    return fn

def get_turn_knob_handle_motion_gen(problem, custom_limits={}, collisions=True, teleport=False,
                                    num_intervals=15, visualize=False, verbose=False):
    if teleport:
        num_intervals = 1
    robot = problem.robot
    world = problem.world
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    BODY_TO_OBJECT = problem.world.BODY_TO_OBJECT
    def fn(a, o, pst1, pst2, g, bq1, aq1, fluents=[]):
        if pst1.value == pst2.value:
            return None

        saver.restore()
        pst1.assign()
        bq1.assign()
        aq1.assign()

        arm_joints = get_arm_joints(robot, a)
        resolutions = 0.05 ** np.ones(len(arm_joints))

        joint_object = BODY_TO_OBJECT[o]
        old_pose = get_link_pose(joint_object.body, joint_object.handle_link)
        if visualize:
            set_renderer(enable=True)
            gripper_before_body = visualize_grasp(robot, old_pose, g.value)
        gripper_before = multiply(old_pose, invert(g.value))  ## multiply(, tool_from_root)

        ## saving the mapping between robot bconf to object pst for execution
        mapping = {}
        rpose_rounded = tuple([round(n, 3) for n in aq1.values])
        mapping[rpose_rounded] = pst1.value

        apath = []
        aq_after = Conf(aq1.body, aq1.joints, aq1.values)
        for i in range(num_intervals):
            step_str = f"pr2_streams.get_pull_door_handle_motion_gen | step {i}/{num_intervals}\t"
            change = (i + 1) / num_intervals * (pst2.value - pst1.value)
            value = change + pst1.value
            pst_after = Position((pst1.body, pst1.joint), value)
            pst_after.assign()
            new_pose = get_link_pose(joint_object.body, joint_object.handle_link)
            if visualize:
                if i == 0: remove_body(gripper_before_body)
                gripper_after = visualize_grasp(robot, new_pose, g.value, color=BROWN)
                set_camera_target_body(gripper_after, dx=0.2, dy=0, dz=1) ## look top down
                remove_body(gripper_after)
            gripper_after = multiply(new_pose, invert(g.value))  ## multiply(, tool_from_root)
            aconf_after = list(aq1.values)
            aconf_after[-1] -= change
            aq_after = Conf(aq1.body, aq1.joints, aconf_after)
            aq_after.assign()

            if any(pairwise_collision(robot, b) for b in obstacles):
                collided = []
                for b in obstacles:
                    if pairwise_collision(robot, b):
                        collided.append(b)
                collided = [world.BODY_TO_OBJECT[c].shorter_name for c in collided]
                print(f'{step_str} arm collide at {nice(aconf_after)} with {collided}')
                if len(apath) > 1:
                    apath[-1].assign()
                    break

            apath.append(aq_after)
            rpose_rounded = tuple([round(n, 3) for n in aq_after.values])
            mapping[rpose_rounded] = value
            if verbose: print(f'{step_str} : {nice(aq_after.values)}')

        if len(apath) < num_intervals * 0.25:
            return None

        body, joint = o
        if body not in LINK_POSE_TO_JOINT_POSITION:
            LINK_POSE_TO_JOINT_POSITION[body] = {}
        LINK_POSE_TO_JOINT_POSITION[body][joint] = mapping

        at = Trajectory(apath) ## create_trajectory(robot, get_arm_joints(robot, a), apath)
        arm_cmd =  Commands(State(), savers=[BodySaver(robot)], commands=[at])
        aq2 = at.path[-1]
        if aq2.values == aq1.values:
            aq2 = aq1
        step_str = f"pr2_streams.get_turn_knob_handle_motion_gen | step {len(apath)}/{num_intervals}\t"
        if not verbose: print(f'{step_str} : {nice(aq2.values)}')
        return (aq2, arm_cmd)

    return fn

def get_pull_drawer_handle_motion_gen(problem, custom_limits={}, collisions=True,
                                      teleport=False, num_intervals=30, extent=None):
    if teleport:
        num_intervals = 1
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    BODY_TO_OBJECT = problem.world.BODY_TO_OBJECT
    def fn(a, o, pst1, pst2, g, bq1, fluents=[]):  ##
        if extent == 'max':
            pst1 = Position(o, 'min')
            pst2 = Position(o, 'max')
        elif extent == 'min':
            pst1 = Position(o, 'max')
            pst2 = Position(o, 'min')
        else:
            if pst1.value == pst2.value:
                return None
        saver.restore()
        pst1.assign()
        bq1.assign()
        joint_object = BODY_TO_OBJECT[o]
        tool_from_root = get_tool_from_root(robot, a)
        old_pose = get_link_pose(joint_object.body, joint_object.handle_link)
        gripper_before = multiply(multiply(old_pose, invert(g.value)), tool_from_root)
        rpose = bconf_to_pose(bq1)

        bpath = []
        mapping = {}
        rpose_rounded = tuple([round(n, 3) for n in bq1.values])
        mapping[rpose_rounded] = pst1.value
        for i in range(num_intervals):
            value = (i+1)/num_intervals * (pst2.value-pst1.value) + pst1.value
            pst_after = Position((pst1.body, pst1.joint), value)
            pst_after.assign()
            new_pose = get_link_pose(joint_object.body, joint_object.handle_link)
            gripper_after = multiply(multiply(new_pose, invert(g.value)), tool_from_root)
            transform = multiply(gripper_after, invert(gripper_before))

            rpose_after = add_pose(rpose, transform)
            bq_after = pose_to_bconf(rpose_after, robot)
            bpath.append(bq_after)

            rpose_rounded = tuple([round(n, 3) for n in bq_after.values])
            mapping[rpose_rounded] = value

        body, joint = o
        if body not in LINK_POSE_TO_JOINT_POSITION:
            LINK_POSE_TO_JOINT_POSITION[body] = {}
        LINK_POSE_TO_JOINT_POSITION[body][joint] = mapping

        bt = Trajectory(bpath)
        cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
        return (bq_after, cmd)
    return fn

##################################################

# def get_marker_grasp_gen(problem, collisions=False, randomize=True, visualize=False):
#     collisions = True
#     obstacles = problem.fixed if collisions else []
#     world = problem.world
#     def fn(body):
#         grasps = []
#         markers = world.BODY_TO_OBJECT[body].grasp_markers
#         obs = copy.deepcopy(obstacles)
#         if body in obs: obs.remove(body) ## grasp can collide with the object
#         for marker in markers:
#             approach_vector = APPROACH_DISTANCE * get_unit_vector([1, 0, 0]) ##[2, 0, -1])
#             grasps.extend(HandleGrasp('side', marker, g, multiply((approach_vector, unit_quat()), g), SIDE_HOLDING_LEFT_ARM)
#                           for g in get_marker_grasps(marker, grasp_length=GRASP_LENGTH, robot=world.robot, obstacles=obs))  ## , body_pose=body_pose
#             for grasp in grasps:
#                 grasp.grasp_width = 1
#         return [(g,) for g in grasps]
#     return fn

def get_marker_grasp_gen(problem, collisions=False, randomize=True, visualize=False):
    collisions = True
    obstacles = problem.fixed if collisions else []
    world = problem.world
    def fn(marker):
        grasps = []
        obs = copy.deepcopy(obstacles)
        approach_vector = APPROACH_DISTANCE * get_unit_vector([1, 0, 0]) ##[2, 0, -1])
        grasps.extend(HandleGrasp('side', marker, g, multiply((approach_vector, unit_quat()), g), SIDE_HOLDING_LEFT_ARM)
                      for g in get_marker_grasps(marker, grasp_length=GRASP_LENGTH, robot=world.robot, obstacles=obs))  ## , body_pose=body_pose
        for grasp in grasps:
            grasp.grasp_width = 1
        return [(g,) for g in grasps]
    return fn

def get_marker_grasps(body, under=False, tool_pose=TOOL_POSE, body_pose=unit_pose(),
                    max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH, top_offset=SIDE_HEIGHT_OFFSET,
                    robot=None, obstacles=[]):

    from pybullet_tools.utils import Pose

    def check_cfree_gripper(grasp, visualize=False):
        gripper_grasp = visualize_grasp(robot, get_pose(body), grasp)
        if visualize:
            set_camera_target_body(gripper_grasp, dx=0, dy=-1, dz=1)

        for b in obstacles:
            if pairwise_collision(gripper_grasp, b):
                print('making marker grasp collide with', b)
                remove_body(gripper_grasp)
                return False
        remove_body(gripper_grasp)
        return True

    # TODO: compute bounding box width wrt tool frame
    center, (w, l, h) = approximate_as_prism(body, body_pose=body_pose)
    translate_center = Pose(point=point_from_pose(body_pose)-center)
    grasps = []
    x_offset = h/2 - top_offset
    under = True
    for j in range(1 + under):
        swap_xz = Pose(euler=[0, -math.pi / 2 + j * math.pi, 0])
        if l <= max_width:
            for gl in [grasp_length]:  ## [grasp_length/3, grasp_length/2, grasp_length]:
                translate_z = Pose(point=[x_offset, 0, w / 2 - gl])
                for i in range(1, 2):
                    rotate_z = Pose(euler=[0, 0, i * math.pi/2])
                    grasp = multiply(tool_pose, translate_z, rotate_z, swap_xz, translate_center, body_pose)
                    if check_cfree_gripper(grasp):
                        grasps += [grasp]  # , np.array([l])
    return grasps

def visualize_grasp(robot, body_pose, grasp, arm='left', color=GREEN):
    link_name = PR2_GRIPPER_ROOTS[arm]
    links = get_link_subtree(robot, link_from_name(robot, link_name))
    gripper_grasp = clone_body(robot, links=links, visual=True, collision=True)
    set_all_color(gripper_grasp, color)
    tool_from_root = get_tool_from_root(robot, arm)
    grasp_pose = multiply(multiply(body_pose, invert(grasp)), tool_from_root)
    set_pose(gripper_grasp, grasp_pose)
    return gripper_grasp

##################################################

def sample_points_along_line(body, marker, num_intervals=None, bq=None,
                             limit=(2.5, 3), learned=False):

    x1, y1, z1 = get_pose(body)[0]
    (x2, y2, z2), quat = get_pose(marker)
    k = (y2 - y1) / (x2 - x1)

    def sample_point():
        lo, hi = limit
        dx = np.random.uniform(lo, hi)
        if learned: dx = lo
        return dx, (x2 + dx, y2 + k * dx)

    dx, (x, y) = sample_point()
    pose2 = ((x, y, z2), quat)

    if num_intervals != None and bq != None:
        rx, ry, ryaw = bq
        bqs = []  ## base config of robot
        for n in range(num_intervals):
            x = rx + dx * ((n+1)/num_intervals)
            y = ry + k * dx * ((n+1)/num_intervals)
            bqs.append((x, y, ryaw))
        return pose2, bqs

    return pose2

def get_bqs_given_p2(marker, parent, bq, pose2, num_intervals):
    x1, y1, z1 = get_pose(parent)[0]
    (x2, y2, z2), quat = get_pose(marker)
    k = (y2 - y1) / (x2 - x1)

    ## reverse engineer the dx
    ((x, y, z2), quat) = pose2
    dx = x - x2

    rx, ry, ryaw = bq
    bqs = []  ## base config of robot
    for n in range(num_intervals):
        x = rx + dx * ((n + 1) / num_intervals)
        y = ry + k * dx * ((n + 1) / num_intervals)
        bqs.append((x, y, ryaw))

    return bqs

def get_bqs_given_bq2(marker, parent, bq1, bq2, num_intervals):
    x1, y1, z1 = get_pose(parent)[0]
    (x2, y2, z2), quat = get_pose(marker)
    k = (y2 - y1) / (x2 - x1)

    ## reverse engineer the dx
    dx = bq2[0] - bq1[0]
    x = x2 + dx
    y = y2 + k * dx
    pose2 = ((x, y, z2), quat)

    rx, ry, ryaw = bq1
    bqs = []
    for n in range(num_intervals):
        x = rx + dx * ((n + 1) / num_intervals)
        y = ry + k * dx * ((n + 1) / num_intervals)
        bqs.append((x, y, ryaw))

    return pose2, bqs

def get_marker_pose_gen(problem, num_samples=70, collisions=False, visualize=False):
    from pybullet_tools.pr2_primitives import Pose
    collisions = True
    world = problem.world
    def fn(o, p1):
        poses = []
        parent = world.BODY_TO_OBJECT[o].grasp_parent
        p1.assign()

        for i in range(num_samples):
            pose2 = sample_points_along_line(parent, o)
            if visualize:
                visualize_point(pose2[0], world)
            p2 = Pose(o, pose2)
            poses.append((p2,))
        return poses
    return fn

def get_parent_new_pose(p1, p2, p3):
    x1, y1, _ = p1[0]
    x2, y2, _ = p2[0]
    (x3, y3, z3), quat = p3
    return ((x3+x2-x1, y3+y2-y1, z3), quat)

def get_pull_marker_random_motion_gen(problem, custom_limits={}, collisions=True, max_attempts=30,
                               teleport=False, num_intervals=30, learned=False):
    from pybullet_tools.pr2_primitives import Pose

    if teleport:
        num_intervals = 1
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    world = problem.world
    def fn(a, o, p1, g, bq1, o2, p3, fluents=[]):

        parent = world.BODY_TO_OBJECT[o].grasp_parent
        approach_obstacles = copy.deepcopy(obstacles)
        if parent in approach_obstacles: approach_obstacles.remove(parent)

        attempts = 0
        while attempts < max_attempts:
            saver.restore()
            p1.assign()
            bq1.assign()

            ## sample a p2 along the direction and deduce bconf
            pose2, bqs = sample_points_along_line(parent, o, num_intervals, bq1.values, learned=learned)
            p2 = Pose(o, pose2)
            p4 = Pose(o2, get_parent_new_pose(p1.value, pose2, p3.value))

            bpath = [Conf(robot, get_group_joints(robot, 'base'), bq) for bq in bqs]
            collided = False

            ## TODO: do collision checking with other streams
            for bq in bpath:
                bq.assign()
                if any(pairwise_collision(robot, b) or pairwise_collision(parent, b) for b in approach_obstacles):
                    collided = True
            if not collided:
                bt = Trajectory(bpath)
                cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
                yield (p2, bpath[-1], p4, cmd)
        yield None
    return fn

def get_pull_marker_to_pose_motion_gen(problem, custom_limits={}, collisions=True,
                               teleport=False, num_intervals=30):
    from pybullet_tools.pr2_primitives import Pose

    if teleport:
        num_intervals = 1
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    world = problem.world
    def fn(a, o, p1, p2, g, bq1, o2, p3, fluents=[]):

        parent = world.BODY_TO_OBJECT[o].grasp_parent
        approach_obstacles = copy.deepcopy(obstacles)
        if parent in approach_obstacles: approach_obstacles.remove(parent)

        saver.restore()
        p1.assign()
        bq1.assign()

        ## get list of bconf sample along the direction given p2
        bqs = get_bqs_given_p2(o, parent, bq1.values, p2.value, num_intervals)
        p4 = Pose(o2, get_parent_new_pose(p1.value, p2.value, p3.value))

        bpath = [Conf(robot, get_group_joints(robot, 'base'), bq) for bq in bqs]
        collided = False

        ## TODO: do collision checking with other streams
        for bq in bpath:
            bq.assign()
            if any(pairwise_collision(robot, b) or pairwise_collision(parent, b) for b in approach_obstacles):
                collided = True
        if not collided:
            bt = Trajectory(bpath)
            cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
            return (bpath[-1], p4, cmd)
        return None

    return fn

def get_pull_marker_to_bconf_motion_gen(problem, custom_limits={}, collisions=True,
                               teleport=False, num_intervals=30):
    from pybullet_tools.pr2_primitives import Pose

    if teleport:
        num_intervals = 1
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    world = problem.world
    def fn(a, o, p1, g, bq1, bq2, o2, p3, fluents=[]):

        parent = world.BODY_TO_OBJECT[o].grasp_parent
        approach_obstacles = copy.deepcopy(obstacles)
        if parent in approach_obstacles: approach_obstacles.remove(parent)

        saver.restore()
        p1.assign()
        bq1.assign()

        ## get list of bconf sample along the direction given p2
        pose2, bqs = get_bqs_given_bq2(o, parent, bq1.values, bq2.values, num_intervals)
        p2 = Pose(o, pose2)
        p4 = Pose(o2, get_parent_new_pose(p1.value, pose2, p3.value))

        bpath = [Conf(robot, get_group_joints(robot, 'base'), bq) for bq in bqs]
        collided = False

        ## TODO: do collision checking with other streams
        for bq in bpath:
            bq.assign()
            if any(pairwise_collision(robot, b) or pairwise_collision(parent, b) for b in approach_obstacles):
                collided = True
        if not collided:
            bt = Trajectory(bpath)
            cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
            return (p2, p4, cmd)
        return None

    return fn



##################################################

def get_pose_in_region_test():
    def test(o, p, r):
        p.assign()
        obj_aabb = aabb2d_from_aabb(get_aabb(o))
        region_aabb = aabb2d_from_aabb(get_aabb(r))
        # return aabb_overlap(obj_aabb, region_aabb)

        obj_center = get_aabb_center(obj_aabb)
        return aabb_contains_point(obj_aabb, region_aabb)
    return test

def get_bconf_in_region_test(robot):
    rob = robot
    def test(bq, r):
        bq.assign()
        ## needs to be only rob base because arm may stick over in the air
        rob_aabb = aabb2d_from_aabb(get_aabb(rob, link_from_name(rob, "base_link")))
        region_aabb = aabb2d_from_aabb(get_aabb(r))
        # return aabb_overlap(rob_aabb, region_aabb)

        rob_center = get_aabb_center(rob_aabb)
        return aabb_contains_point(rob_center, region_aabb)
    return test

def get_bconf_in_region_gen(problem, collisions=True, max_attempts=10, verbose=False, visualize=False):
    obstacles = problem.fixed if collisions else []
    robot = problem.robot
    yaw = 0

    def gen(region):
        lower, upper = get_aabb(region)
        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            x = np.random.uniform(lower[0], upper[0])
            y = np.random.uniform(lower[1], upper[1])
            bq = Conf(robot, get_group_joints(robot, 'base'), (x, y, yaw))
            bq.assign()
            if not any(pairwise_collision(robot, obst) for obst in obstacles):
                if visualize:
                    rbb = create_pr2()
                    set_group_conf(rbb, 'base', bq.values)
                yield (bq,)
        yield None

    def list_fn(region):
        lower, upper = get_aabb(region)
        attempts = 0
        bqs = []
        while attempts < max_attempts:
            attempts += 1
            x = np.random.uniform(lower[0], upper[0])
            y = np.random.uniform(lower[1], upper[1])
            bq = Conf(robot, get_group_joints(robot, 'base'), (x, y, yaw))
            bq.assign()
            if not any(pairwise_collision(robot, obst) for obst in obstacles):
                if visualize:
                    rbb = create_pr2()
                    set_group_conf(rbb, 'base', bq.values)
                bqs.append((bq,))
        return bqs
    return list_fn ## gen

def get_pose_in_region_gen(problem, collisions=True, max_attempts=40, verbose=False, visualize=False):
    from pybullet_tools.pr2_primitives import Pose
    obstacles = problem.fixed if collisions else []
    robot = problem.robot
    def gen(o, r):
        ((_, _, z), quat) = get_pose(o)
        lower, upper = get_aabb(r)
        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            x = np.random.uniform(lower[0], upper[0])
            y = np.random.uniform(lower[1], upper[1])
            pose = (x, y, z), quat
            # x, y, z, yaw = sample_obj_in_body_link_space(robot, region,
            #     PLACEMENT_ONLY=True, XY_ONLY=True, verbose=verbose, **kwargs)
            p = Pose(o, pose)
            p.assign()
            if not any(pairwise_collision(o, obst) for obst in obstacles):
                yield (p,)
        yield None
    # return gen

    def list_fn(region):
        ((_, _, z), quat) = get_pose(o)
        lower, upper = get_aabb(r)
        attempts = 0
        poses = []
        while attempts < max_attempts:
            attempts += 1
            x = np.random.uniform(lower[0], upper[0])
            y = np.random.uniform(lower[1], upper[1])
            pose = (x, y, z), quat
            # x, y, z, yaw = sample_obj_in_body_link_space(robot, region,
            #     PLACEMENT_ONLY=True, XY_ONLY=True, verbose=verbose, **kwargs)
            p = Pose(o, pose)
            p.assign()
            if not any(pairwise_collision(o, obst) for obst in obstacles):
                poses.append((p,))
        return poses
    return list_fn ## gen

##################################################

def get_motion_wconf_gen(problem, custom_limits={}, collisions=True, teleport=False):
    from pybullet_tools.pr2_primitives import Pose
    # TODO: include fluents
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    def fn(bq1, bq2, w, fluents=[]):
        saver.restore()
        bq1.assign()

        ## the only thing added from get_motion_wconf
        w.assign()
        bconf = get_joint_positions(robot, get_group_joints(robot, 'base'))
        aconf = get_joint_positions(robot, get_arm_joints(robot, 'left'))

        num_trials = 2  ## sometimes it can't find a path to get around the open door
        params = [(4, 50), (6, 75)] ## , (10, 100)
        while num_trials > 0:
            param = params[-num_trials]
            raw_path = plan_joint_motion(robot, bq2.joints, bq2.values, attachments=[],
                                         obstacles=obstacles, custom_limits=custom_limits, self_collisions=SELF_COLLISIONS,
                                         restarts=param[0], iterations=param[1], smooth=50)
                                         # restarts=4, iterations=50, smooth=50)
            # break
            num_trials -= 1
            if raw_path != None:
                break
            bq1.assign()

        print(f'pr2_streams.get_motion_wconf_gen\t under {w.printout(obstacles)}, '
              f'from bconf = {nice(bconf)}, aconf = {nice(aconf)}, num_trials = {2-num_trials}')
        if raw_path is None:
            print('Failed motion plan (with world config)!', get_bodies())
            # set_renderer(True)
            # for bq in [bq1, bq2]:
            #     bq.assign()
            #     wait_if_gui()
            # set_renderer(False)
            return None
        path = [Conf(robot, bq2.joints, q) for q in raw_path]
        bt = Trajectory(path)
        cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
        return (cmd,)
    return fn

def get_update_wconf_p_gen(verbose=True):
    def fn(w1, o, p):
        poses = copy.deepcopy(w1.poses)
        if verbose:
            print('pr2_streams.get_update_wconf_p_gen\tbefore:', {o0: nice(p0.value[0]) for o0,p0 in poses.items()})
        if o != p.body:
            return None
        elif o in poses and poses[o].value == p.value:
            poses.pop(o)
        else:
            poses[o] = copy.deepcopy(p)
        w2 = WConf(poses, w1.positions)
        if verbose:
            print('pr2_streams.get_update_wconf_p_gen\t after:', {o0: nice(p0.value[0]) for o0,p0 in w2.poses.items()})
        return (w2,)
    return fn

def get_update_wconf_p_two_gen(verbose=False):
    def fn(w1, o, p, o2, p2):
        poses = copy.deepcopy(w1.poses)
        if verbose:
            print('pr2_streams.get_update_wconf_p_two_gen\tbefore:', {o0: nice(p0.value[0]) for o0,p0 in poses.items()})
        poses[o] = p
        poses[o2] = p2
        w2 = WConf(poses, w1.positions)
        if verbose:
            print('pr2_streams.get_update_wconf_p_two_gen\t after:', {o0: nice(p0.value[0]) for o0,p0 in poses.items()})
        return (w2,)
    return fn

def get_update_wconf_pst_gen(verbose=False):
    def fn(w1, o, pst):
        positions = copy.deepcopy(w1.positions)
        if verbose:
            print('pr2_streams.get_update_wconf_pst_gen\tbefore:', {o0: nice(p0.value) for o0,p0 in positions.items()})
        positions[o] = pst
        w2 = WConf(w1.poses, positions)
        if verbose:
            print('pr2_streams.get_update_wconf_pst_gen\t after:', {o0: nice(p0.value) for o0,p0 in w2.positions.items()})
        return (w2,)
    return fn

##################################################



##################################################

def get_cfree_btraj_pose_test(robot, collisions=True, verbose=True):
    def test(c, b2, p2):
        # TODO: infer robot from c
        if not collisions:
            return True
        state = c.assign()
        if b2 in state.attachments:
            return True
        p2.assign()

        if verbose:
            robot_pose = robot.get_pose()
            print('    get_cfree_btraj_pose_test   \    pose of robot', nice(robot_pose))
        for _ in c.apply(state):
            state.assign()
            for b1 in state.attachments:
                if pairwise_collision(b1, b2):
                    if verbose:
                        print(f'      collision with {b1}, {b2}')
                        print(f'         pose of {b1}', nice(get_pose(b1)))
                        print(f'         pose of {b2}', nice(get_pose(b2)))
                    #wait_for_user()
                    return False
            if pairwise_collision(robot, b2):
                if verbose:
                    print(f'      collision {robot}, {b2}')
                    print(f'         pose of robot', nice(robot.get_pose()))
                    print(f'         pose of {b2}', nice(get_pose(b2)))
                return False
        # TODO: just check collisions with moving links
        return True
    return test

# def get_motion_list_gen(problem, custom_limits={}, num_attempts=1, collisions=True, teleport=False):
#     robot = problem.robot
#     saver = BodySaver(robot)
#     obstacles = problem.fixed if collisions else []
#     def list_fn(bq1, bq2):
#         paths = []
#         # print('pr2_streams.get_motion_gen\tobstacles:', obstacles)
#         for i in range(num_attempts):
#             saver.restore()
#             bq1.assign()
#             raw_path = plan_joint_motion(robot, bq2.joints, bq2.values, attachments=[],
#                                          obstacles=obstacles, custom_limits=custom_limits, self_collisions=SELF_COLLISIONS,
#                                          restarts=4, max_iterations=50, smooth=50)
#             if raw_path != None:
#                 path = [Conf(robot, bq2.joints, q) for q in raw_path]
#                 bt = Trajectory(path)
#                 cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
#                 paths.append((cmd,))
#         if len(paths) == 0:
#             return None
#         return paths
#     return list_fn
