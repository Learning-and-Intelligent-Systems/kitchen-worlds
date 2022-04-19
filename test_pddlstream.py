#!/usr/bin/env python

from __future__ import print_function
from os.path import join
import sys
sys.path.append('pddlstream')
sys.path.append(join('pybullet-planning'))

from pybullet_tools.pr2_primitives import get_base_custom_limits, control_commands, apply_commands, State
from pybullet_tools.pr2_utils import get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf
from pybullet_tools.utils import connect, get_pose, is_placement, point_from_pose, \
    disconnect, get_joint_positions, enable_gravity, save_state, restore_state, HideOutput, quat_from_euler, \
    get_distance, LockRenderer, get_min_limit, get_max_limit, has_gui, WorldSaver, wait_if_gui, add_line, SEPARATOR

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser

## new versions used by Yang
from pybullet_tools.pr2_primitives import Pose, Conf
from experiments.pr2_agent import get_stream_info, get_stream_map, post_process

from lisdf_loader import load_lisdf_pybullet

def pddlstream_from_dir(problem, exp_dir, collisions=True, teleport=False):

    world = problem.world

    domain_pddl = read(join(exp_dir, 'domain.pddl')) ## get_file_path(__file__, 'domain.pddl')
    stream_pddl = read(join(exp_dir, 'stream.pddl'))
    constant_map = {}

    init, goal = pddl_to_init_goal(exp_dir, world)
    problem.add_init(init)

    base_limits = ((-5, -5), (5, 5))
    custom_limits = get_base_custom_limits(world.robot, base_limits)
    stream_map = get_stream_map(problem, collisions, custom_limits, teleport)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

class Problem():
    def __init__(self, world):
        self.world = world

    def add_init(self, init):
        pass

#######################################################

def main(exp_dir, partial=False, defer=False, verbose=True):
    parser = create_parser()
    parser.add_argument('-cfree', action='store_true', help='Disables collisions during planning')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)

    world = load_lisdf_pybullet(join(exp_dir, 'scene.lisdf'))
    saver = WorldSaver()
    problem = Problem(world)

    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, collisions=not args.cfree, teleport=args.teleport)

    stream_info = get_stream_info(partial, defer)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', str_from_object(set(stream_map)))
    print(SEPARATOR)

    with Profiler():
        with LockRenderer(lock=not args.enable):
            solution = solve(pddlstream_problem, algorithm=args.algorithm, unit_costs=args.unit,
                             stream_info=stream_info, success_cost=INF, verbose=True, debug=False)
            saver.restore()

    print_solution(solution)
    plan, cost, evaluations = solution
    if (plan is None) or not has_gui():
        disconnect()
        return

    print(SEPARATOR)
    with LockRenderer(lock=not args.enable):
        commands = post_process(problem, plan)
        problem.remove_gripper()
        saver.restore()

    #restore_state(state_id)
    saver.restore()
    wait_if_gui('Execute?')
    if args.simulate:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)
    wait_if_gui('Finish?')
    disconnect()
    # TODO: need to wrap circular joints

def pddl_to_init_goal(exp_dir, world):
    sys.path.append('lisdf')
    import lisdf.components as C
    from lisdf.parsing import load_all

    lisdf, domain, problem = load_all(
        join(exp_dir, 'scene.lisdf'),
        join(exp_dir, 'domain.pddl'),
        join(exp_dir, 'problem.pddl'),
    )
    robot = world.robot

    def prop_to_list(v):
        args = [v.predicate.name]
        for arg in v.arguments:
            if isinstance(arg, C.PDDLObject):
                elem = arg.name
            else:
                typ = ''.join([i for i in arg.name if not i.isdigit()])
                index = ''.join([i for i in arg.name if i.isdigit()])
                value = list(arg.value.value)
                if typ == 'q':
                    elem = Conf(robot, get_group_joints(robot, 'base'), value, index=index)
                elif typ == 'aq':
                    elem = Conf(robot, get_arm_joints(robot, args[-1]), value, index=index)
                elif typ == 'p':
                    body = world.name_to_body[args[-1]]
                    if len(value) == 4:
                        value = value[:3] + [0, 0] + value[-1:]
                    elem = Pose(body, (tuple(value[:3]), quat_from_euler(value[-3:])), index=index)
            args.append(elem)
        return args

    init = [prop_to_list(v) for v in problem.init]
    problem = [prop_to_list(v) for v in problem.conjunctive_goal]

    return init, problem

if __name__ == '__main__':
    exp_dir = join('experiments', 'blocks_kitchen')
    main(exp_dir=exp_dir)