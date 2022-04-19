

from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.object import SharedOptValue
from pddlstream.language.external import defer_shared, never_defer
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test
from collections import namedtuple

from pybullet_tools.pr2_primitives import get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, get_base_custom_limits, \
    get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, move_cost_fn

from pybullet_tools.pr2_streams import get_stable_gen, get_contain_gen, get_position_gen, \
    Position, get_handle_grasp_gen, LinkPose, get_ik_ir_grasp_handle_gen, get_pull_drawer_handle_motion_gen, \
    get_joint_position_test, get_marker_grasp_gen, get_bconf_in_region_test, get_pull_door_handle_motion_gen, \
    get_bconf_in_region_gen, get_pose_in_region_gen, visualize_grasp, get_motion_wconf_gen, get_update_wconf_p_two_gen, \
    get_marker_pose_gen, get_pull_marker_to_pose_motion_gen, get_pull_marker_to_bconf_motion_gen,  \
    get_pull_marker_random_motion_gen, get_ik_ungrasp_handle_gen, get_pose_in_region_test, \
    get_cfree_btraj_pose_test, get_joint_position_open_gen, get_ik_ungrasp_mark_gen, \
    sample_joint_position_open_list_gen, get_update_wconf_pst_gen, get_ik_ir_wconf_gen, \
    get_update_wconf_p_gen, get_ik_ir_wconf_gen, get_pose_in_space_test, get_turn_knob_handle_motion_gen

from pybullet_tools.utils import get_max_limit

def get_stream_map(p, c, l, t):
    # p = problem
    # c = collisions
    # l = custom_limits
    # t = teleport
    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(p, collisions=c)),
        'sample-pose-inside': from_gen_fn(get_contain_gen(p, collisions=c)),  ##
        'sample-grasp': from_list_fn(get_grasp_gen(p, collisions=True)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                        learned=False, max_attempts=60, verbose=False)),
        'inverse-kinematics-wconf': from_gen_fn(get_ik_ir_wconf_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                    learned=False, max_attempts=60, verbose=False,
                                                                    visualize=False)),
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

        'inverse-kinematics-grasp-handle': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                       learned=False, verbose=False, ACONF=True, WCONF=False)),
        'inverse-kinematics-ungrasp-handle': from_gen_fn(
            get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                      verbose=False, WCONF=False)),
        'inverse-kinematics-grasp-handle-wconf': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                       learned=False, verbose=False, ACONF=True, WCONF=True)),
        'inverse-kinematics-ungrasp-handle-wconf': from_gen_fn(
            get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                      verbose=False, WCONF=True)),

        'plan-base-pull-drawer-handle': from_fn(
            get_pull_drawer_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-base-pull-door-handle': from_fn(
            get_pull_door_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-arm-turn-knob-handle': from_fn(
            get_turn_knob_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),

        'sample-marker-grasp': from_list_fn(get_marker_grasp_gen(p, collisions=c)),
        'inverse-kinematics-grasp-marker': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=True, teleport=t, custom_limits=l,
                                       learned=False, verbose=False)),
        'inverse-kinematics-ungrasp-marker': from_fn(
            get_ik_ungrasp_mark_gen(p, collisions=True, teleport=t, custom_limits=l)),
        'plan-base-pull-marker-random': from_gen_fn(
            get_pull_marker_random_motion_gen(p, collisions=c, teleport=t, custom_limits=l,
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
    return stream_map

def get_stream_info(partial, defer):
    stream_info = {
        # 'test-cfree-pose-pose': StreamInfo(p_success=1e-3, verbose=verbose),
        # 'test-cfree-approach-pose': StreamInfo(p_success=1e-2, verbose=verbose),
        # 'test-cfree-traj-pose': StreamInfo(p_success=1e-1, verbose=verbose),

        'MoveCost': FunctionInfo(opt_move_cost_fn),
    }
    stream_info.update({
                           'sample-pose': StreamInfo(opt_gen_fn=PartialInputs('?r')),
                           'inverse-kinematics': StreamInfo(opt_gen_fn=PartialInputs('?p')),
                           'plan-base-motion': StreamInfo(opt_gen_fn=PartialInputs('?q1 ?q2'),
                                                          defer_fn=defer_shared if defer else never_defer),
                       } if partial else {
        'sample-pose': StreamInfo(opt_gen_fn=from_fn(opt_pose_fn)),
        'inverse-kinematics': StreamInfo(opt_gen_fn=from_fn(opt_ik_fn)),
        'plan-base-motion': StreamInfo(opt_gen_fn=from_fn(opt_motion_fn)),
    })
    return stream_info


#######################################################

CustomValue = namedtuple('CustomValue', ['stream', 'values'])

def opt_move_cost_fn(t):
    # q1, q2 = t.values
    # distance = get_distance(extract_point2d(q1), extract_point2d(q2))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

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


#######################################################

def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            c = args[-1]
            new_commands = c.commands
        elif name == 'pick':
            a, b, p, g, _, c = args
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, a, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot, a)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
            detach = Detach(problem.robot, a, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        elif name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif name == 'press_clean':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Clean(body), t.reverse()]
        elif name == 'press_cook':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands