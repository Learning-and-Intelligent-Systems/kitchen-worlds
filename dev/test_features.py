import os

from pybullet_tools.utils import reset_simulation, AABB, \
    get_aabb_extent, get_aabb_center, get_aabb, get_point, get_joint_positions
from pybullet_tools.bullet_utils import get_door_links

from world_builder.world import State
from pigi_tools.data_utils import get_indices, get_init_tuples
from lisdf_tools.lisdf_loader import load_lisdf_pybullet, create_gripper_robot
import shutil
from os import listdir
from os.path import join, isdir, isfile

# from utils import load_lisdf_synthesizer
from data_generator.run_utils import copy_dir_for_process
from pigi_tools.run_utils import process_all_tasks
from examples.test_utils import get_data_processing_parser

# DEFAULT_TASK = 'mm'
DEFAULT_TASK = 'tt'
# DEFAULT_TASK = 'ff'
# DEFAULT_TASK = 'ww_two_fridge_in'

CASES = None  ## ['0'] | None

PARALLEL = True
USE_VIEWER = False

parser = get_data_processing_parser(task_name=DEFAULT_TASK, parallel=PARALLEL, use_viewer=USE_VIEWER)
args = parser.parse_args()

USE_VIEWER = args.viewer
CHECK_TIME = 1664399646.826951


def add_features(test_dir, viz_dir, verbose=False):
    ori_file = join(viz_dir, 'features.txt')
    print('\n'+ori_file)

    if isfile(ori_file) and os.path.getmtime(ori_file) > CHECK_TIME:
        return

    REACHABILITY = 'reachability'
    AABBY = 'aabby'
    DISTANCEY = 'distancey'

    data = {k: {} for k in [REACHABILITY, AABBY, DISTANCEY]}
    lines_ori = []
    if isfile(ori_file):
        lines_ori = [l.strip() for l in open(ori_file, 'r').readlines()]
        for l in lines_ori:
            tup = l.split(' ')
            key = tup[0]
            if 'reachable' in key:
                data[REACHABILITY][tup[1]] = key
            elif AABBY in key:
                data[key][tup[1]] = tup[2]
            elif DISTANCEY in key:
                if tup[1] not in data[key]:
                    data[key][tup[1]] = {}
                data[key][tup[1]][tup[2]] = tup[3]

    REACHABLE_OBJ = "reachable {object}"
    UNREACHABLE_OBJ = "unreachable {object}"
    REACHABLE_SPACE = "reachable {space}"
    UNREACHABLE_SPACE = "unreachable {space}"
    LINE_AABBY = AABBY + " {name} {width}"
    LINE_DISTANCEY = DISTANCEY + " {movable} {obstacle} {distance}"

    world = load_lisdf_pybullet(test_dir, use_gui=USE_VIEWER, verbose=False)
    world.check_world_obstacles()
    old_file = join(test_dir, 'features.txt')
    custom_limits = world.robot.custom_limits
    init_q = get_joint_positions(world.robot, world.robot.get_base_joints())[:-1]
    init_q = list(init_q) + [0] * 3
    world.remove_object(world.robot)

    robot = create_gripper_robot(world, custom_limits=custom_limits, initial_q=init_q)
    problem = State(world, grasp_types=robot.grasp_types)  ## , 'side' , 'top'
    indices = get_indices(test_dir)
    indices_inv = {v: eval(k) for k, v in indices.items()}
    init = get_init_tuples(test_dir)
    spaces = world.cat_to_bodies('space', init)
    doors = world.cat_to_bodies('door', init)
    movable = [o for n, o in world.name_to_body.items() if 'veggie' in n or 'meat' in n]
    obstacles = [o for n, o in world.name_to_body.items() if o not in movable and o != robot]
    lines = []

    """ ============== reachability of movable objects ============= """
    for body in movable:
        name = world.body_to_name[body]
        # if name in data[REACHABILITY]:
        #     lines.append(' '.join([data[REACHABILITY][name], name]))
        #     continue
        result = robot.check_reachability(body, problem, obstacles=obstacles)

        if result:
            lines.append(REACHABLE_OBJ.format(object=name))
        else:
            lines.append(UNREACHABLE_OBJ.format(object=name))

    """ ============== reachability of spaces ============= """

    names = {
        'minifridge': 'minifridge::fridgestorage',
        'cabinet': 'cabinet::cabinetstorage',
    }
    print('reachability of spaces')
    for space in spaces:
        # if space in data[REACHABILITY]:
        #     lines.append(' '.join([data[REACHABILITY][space], space]))
        #     continue
        body_link = indices_inv[space]
        if space not in world.name_to_body:
            body_name = names[space[:space.index('::')]]
            world.add_body(body_link, body_name)

        result = robot.check_reachability_space(body_link, problem, obstacles=obstacles)
        if result:
            lines.append(REACHABLE_SPACE.format(space=space))
        else:
            lines.append(UNREACHABLE_SPACE.format(space=space))

    """ ============== width of objects and links ============= """
    aabbs = {}
    for space in spaces:
        body_link = indices_inv[space]
        aabbs[space] = get_aabb(body_link[0], body_link[-1])
    for door in doors:
        body_joint = indices_inv[door]
        links = get_door_links(body_joint[0], body_joint[1])
        group_lower = [1000] * 3
        group_upper = [-1000] * 3
        for l in links:
            lower, upper = get_aabb(body_joint[0], link=l)
            for i in range(3):
                if group_upper[i] < upper[i]:
                    group_upper[i] = upper[i]
                if group_lower[i] > lower[i]:
                    group_lower[i] = lower[i]
        aabbs[door] = AABB(group_lower, group_upper)

    bodies = movable + [indices_inv[s] for s in spaces] + [indices_inv[d] for d in doors]
    for body in bodies:
        name = indices[str(body)]
        # if name in data[AABBY]:
        #     lines.append(LINE_AABBY.format(name=name, width=data[AABBY][name]))
        #     continue
        if isinstance(body, int):
            aabb = get_aabb(body)
        else:
            aabb = aabbs[name]
        lines.append(LINE_AABBY.format(name=name, width=round(get_aabb_extent(aabb)[1], 3)))

    """ ============ distance between movable and spaces and joints =========== """
    for body in movable:
        name = world.body_to_name[body]
        y_body = get_aabb_center(get_aabb(body))[1]
        for o, o_aabb in aabbs.items():
            if name in data[AABBY]:
                lines.append(LINE_DISTANCEY.format(movable=name, obstacle=o,
                                                   distance=data[DISTANCEY][name][o]))
                continue
            y = y_body - get_aabb_center(o_aabb)[1]
            lines.append(LINE_DISTANCEY.format(movable=name, obstacle=o, distance=round(y, 3)))

    """ ============== save in file ============= """
    if isfile(ori_file):
        os.remove(ori_file)
    lines = '\n'.join(lines)
    with open(old_file, 'w') as f:
        f.writelines(lines)
    print(lines, '\n')
    shutil.copy(old_file, ori_file)
    reset_simulation()
    # sys.exit()


def adjust_table_scale(test_dir, viz_dir):
    import untangle
    print('adjust_table_scale', test_dir)
    world = load_lisdf_pybullet(test_dir, use_gui=USE_VIEWER, verbose=False)
    old_file = join(test_dir, 'scene.lisdf')
    new_file = join(test_dir, 'scene_tmp.lisdf')
    if not isfile(new_file) or True:
        shutil.copy(old_file, new_file)
        old_lines = open(old_file, 'r').readlines()
        models = untangle.parse(old_file).sdf.world.include
        other = {'counter': 'minifridge', 'table': 'cabinet'}
        for model in models:
            if 'kitchencounter' in model.uri.cdata.lower():
                scale = eval(model.scale.cdata)
                name = model['name']
                # if scale != 1:
                #     print(name, 'put back', scale)
                #     old_lines = replace_scale(old_lines, name, 1)
                if scale == 1:
                    body = world.name_to_body[name]
                    h = get_aabb_extent(get_aabb(body))[2]
                    ceiling_name = other[name]
                    if ceiling_name in world.name_to_body:
                        ceiling = world.name_to_body[ceiling_name]
                        h_space = get_aabb(ceiling).lower[2]
                        new_scale = h_space / h
                    else:
                        z = get_point(body)[2]
                        new_scale = z / (z - get_aabb(body).lower[2])
                    print(name, 'new_scale', new_scale)
                    old_lines = replace_scale(old_lines, name, new_scale)
        with open(old_file, 'w') as f:
            f.writelines(old_lines)
    ori_file = join(viz_dir, 'scene.lisdf')
    tmp_file = join(viz_dir, 'scene_tmp.lisdf')
    if not isfile(tmp_file) or True:
        os.remove(ori_file)
        shutil.copy(old_file, ori_file)
        shutil.copy(new_file, tmp_file)
    reset_simulation()


def replace_scale(lines, name, new_scale):
    to_find = f'<include name="{name}">'
    for i in range(len(lines)):
        if to_find in lines[i] and '<scale>' in lines[i+3]:
            l = lines[i+3]
            old_scale = l[l.index('<scale>')+7: l.index('</scale>')]
            lines[i+3] = l.replace(old_scale, str(new_scale))
    return lines


def process(viz_dir):
    test_dir = copy_dir_for_process(viz_dir)

    # load_lisdf_synthesizer(test_dir)
    # adjust_table_scale(test_dir, viz_dir)
    add_features(test_dir, viz_dir)
    shutil.rmtree(test_dir)


def duplicate_baseline(viz_dir, old_dir, new_dir):
    old_dir = join(viz_dir, old_dir)
    new_dir = join(viz_dir, new_dir)
    if not isdir(new_dir):
        os.mkdir(new_dir)
    files = [f for f in listdir(old_dir) if '=None.' in f or '=oracle.' in f]
    for file in files:
        if not isfile(join(new_dir, file)):
            print('duplicate_baseline', join(old_dir, file), join(new_dir, file))
            shutil.copy(join(old_dir, file), join(new_dir, file))


def duplicate_process(viz_dir):
    duplicate_baseline(viz_dir, 'rerun_1', 'rerun_2')


if __name__ == "__main__":
    process_all_tasks(process, args.t, parallel=args.p, cases=CASES)
    # process_all_tasks(duplicate_process, args.t, parallel=args.p, cases=CASES)
