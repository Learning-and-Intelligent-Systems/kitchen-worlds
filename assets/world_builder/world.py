import time
from itertools import product
from collections import defaultdict
import copy

from bullet.actions import Action
from pybullet_tools.utils import get_max_velocities, WorldSaver, elapsed_time, get_pose, LockRenderer, \
    CameraImage, get_joint_positions, euler_from_quat, get_link_name, get_joint_position, \
    BodySaver, set_pose, INF, add_parameter, irange, wait_for_duration, get_bodies, remove_body, \
    read_parameter, pairwise_collision, str_from_object, get_joint_name, get_name, get_link_pose, \
    get_joints, multiply, invert, is_movable
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_arm_joints, ARM_NAMES, get_group_joints, \
    get_group_conf, get_top_grasps, get_side_grasps, create_gripper
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, link_from_name, \
    get_gripper_joints, GripperCommand, apply_commands, State
from pddlstream.language.constants import Equal, AND
# from pybullet_tools.retime import interpolate_path, sample_curve
from bullet.utils import set_zero_world, nice, open_joint, get_pose2d, summarize_joints, get_point_distance, \
    is_placement, is_contained, add_body, close_joint, toggle_joint, ObjAttachment, check_joint_state, \
    set_camera_target_body, xyzyaw_to_pose
from bullet.entities import Region, Environment, Robot, Surface, ArticulatedObjectPart, Door, Drawer, Knob
from bullet.processes.pddlstream_agent.pr2_streams import get_stable_gen, get_contain_gen, get_position_gen, \
    Position, get_handle_grasp_gen, LinkPose, pr2_grasp, WConf

class World(object):
    def __init__(self, args, time_step=1e-3, prevent_collisions=False,
                 conf_noise=None, pose_noise=None, depth_noise=None, action_noise=None): # TODO: noise model class?
        self.args = args
        self.time_step = time_step
        self.prevent_collisions = prevent_collisions
        self.scramble = False
        # TODO: methods for testing whether something has a certain property

        self.BODY_TO_OBJECT = {}
        self.ROBOT_TO_OBJECT = {}
        self.OBJECTS_BY_CATEGORY = {}
        self.ATTACHMENTS = {}
        self.sub_categories = {}
        self.sup_categories = {}
        self.SKIP_JOINTS = False

    def set_skip_joints(self):
        ## not automatically add doors and drawers, save planning time
        self.SKIP_JOINTS = True

    @property
    def objects(self):
        return [k for k in self.BODY_TO_OBJECT.keys() if k not in self.ROBOT_TO_OBJECT]
    @property
    def movable(self):  ## include steerables if want to exclude them when doing base motion plannig
        return [self.robot] + self.cat_to_bodies('moveable')  ## + self.cat_to_bodies('steerable')
        # return [obj for obj in self.objects if obj not in self.fixed]
    @property
    def floors(self):
        return self.cat_to_bodies('floor')
    @property
    def fixed(self):
        objs = [obj for obj in self.objects if not isinstance(obj, tuple)]
        objs = [o for o in objs if o not in self.floors and o not in self.movable]
        return objs

    def add_box(self, object, pose=None):
        obj = self.add_object(object, pose=pose)
        obj.is_box = True
        return obj

    def add_object(self, object, pose=None):
        OBJECTS_BY_CATEGORY = self.OBJECTS_BY_CATEGORY
        BODY_TO_OBJECT = self.BODY_TO_OBJECT
        category = object.category
        name = object.name
        body = object.body
        joint = object.joint
        link = object.link
        class_name = object.__class__.__name__.lower()

        if category not in OBJECTS_BY_CATEGORY:
            OBJECTS_BY_CATEGORY[category] = []

        ## be able to find eggs as moveables
        if class_name != category:
            if category not in self.sup_categories:
                self.sup_categories[category] = class_name
            if class_name not in self.sub_categories:
                self.sub_categories[class_name] = []
            if category not in self.sub_categories[class_name]:
                self.sub_categories[class_name].append(category)

        ## automatically name object
        if name is None:
            next = len([o.name for o in OBJECTS_BY_CATEGORY[category] if '#' in o.name])
            name = '{}#{}'.format(category, next + 1)
        object.name = name.lower()
        OBJECTS_BY_CATEGORY[category].append(object)

        ## -------------- different types of object --------------
        ## object parts: doors, drawers
        if joint != None:
            BODY_TO_OBJECT[(body, joint)] = object
            object.name = f"{BODY_TO_OBJECT[body].name}--{object.name}"

        ## object parts: surface, space
        elif link != None:
            BODY_TO_OBJECT[(body, None, link)] = object
            object.name = f"{BODY_TO_OBJECT[body].name}--{object.name}"
            if category == 'surface':
                BODY_TO_OBJECT[body].surfaces.append(link)
            if category == 'space':
                BODY_TO_OBJECT[body].spaces.append(link)

        ## object
        elif not isinstance(object, Robot):
            BODY_TO_OBJECT[body] = object
            self.get_doors_drawers(object.body)

        ## robot
        else:
            self.ROBOT_TO_OBJECT[body] = object

        if pose != None:
            add_body(object, pose)

        object.world = self
        return object

    def add_to_cat(self, body, cat):
        object = body
        if isinstance(body, int) or isinstance(body, tuple):
            object = self.BODY_TO_OBJECT[body]

        if cat not in self.OBJECTS_BY_CATEGORY:
            self.OBJECTS_BY_CATEGORY[cat] = []
        self.OBJECTS_BY_CATEGORY[cat].append(object)

    def add_robot(self, robot, max_velocities=None):
        self.robot = robot  # TODO: multi-robot
        self.ROBOT_TO_OBJECT[robot.body] = robot
        self.max_velocities = get_max_velocities(robot, robot.joints) if (max_velocities is None) else max_velocities
        robot.world = self

    def add_joint_object(self, body, joint, category=None):
        if category == None:
            category, state = check_joint_state(body, joint)
        joints = {k:[] for k in ['door', 'drawer', 'knob']}
        if 'door' in category:
            joints['door'].append(joint)
            if (body, joint) not in self.BODY_TO_OBJECT:
                door = Door(body, joint=joint)
                self.add_object(door)
        elif 'drawer' in category:
            joints['drawer'].append(joint)
            if (body, joint) not in self.BODY_TO_OBJECT:
                drawer = Drawer(body, joint=joint)
                self.add_object(drawer)
        elif 'knob' in category:
            joints['knob'].append(joint)
            if (body, joint) not in self.BODY_TO_OBJECT:
                knob = Knob(body, joint=joint)
                self.add_object(knob)
                self.add_to_cat(knob, 'joint')
        return joints

    def get_doors_drawers(self, body):
        if self.SKIP_JOINTS:
            return [], []
        obj = self.BODY_TO_OBJECT[body]
        if obj.doors != None:
            return obj.doors, obj.drawers
        doors = []
        drawers = []
        for joint in get_joints(body):
            joints = self.add_joint_object(body, joint)
            doors.extend(joints['door'])
            drawers.extend(joints['drawer'])

        obj.doors = doors
        obj.drawers = drawers
        return doors, drawers

    def add_joints_by_keyword(self, body_name, joint_name, category=None):
        body = self.name_to_body(body_name)
        joints = [j for j in get_joints(body) if is_movable(body, j) and joint_name in get_joint_name(body, j)]
        for j in joints:
            self.add_joint_object(body, j, category=category)
        return [(body, j) for j in joints]

    def add_surface_by_keyword(self, body_name, link_name):
        body = self.name_to_body(body_name)
        link = link_from_name(body, link_name)
        surface = Surface(body, link=link)
        self.add_object(surface)
        return surface

    def summarize_all_types(self):
        printout = ''
        for typ in ['moveable', 'surface', 'door', 'drawer']:
            num = len(self.cat_to_bodies(typ))
            if num > 0:
                printout += "{type}({num}), ".format(type=typ, num=num)
        return printout

    def summarize_all_objects(self):
        from bullet.utils import nice
        from zzz.logging import myprint as print

        BODY_TO_OBJECT = self.BODY_TO_OBJECT
        ROBOT_TO_OBJECT = self.ROBOT_TO_OBJECT

        bodies = get_bodies()
        print('----------------')
        print(f'PART I: pybullet bodies | obstacles = {self.fixed}')
        print('----------------')
        for body in bodies:
            if body in BODY_TO_OBJECT:
                object = BODY_TO_OBJECT[body]
                n = object.name
                pose = nice(get_pose(body))
                line = f'{body}  |  Name: {n}, Pose: {pose}'

                doors, drawers = self.get_doors_drawers(body)
                doors = [get_joint_name(body, j).replace(f'{n}--', '') for j in doors]
                drawers = [get_joint_name(body, j).replace(f'{n}--', '') for j in drawers]

                if len(doors) > 0:
                    line += f'  |  Doors: {doors}'
                if len(drawers) > 0:
                    line += f'  |  Drawers: {drawers}'
                print(line)

            elif body in ROBOT_TO_OBJECT:
                object = ROBOT_TO_OBJECT[body]
                n = object.name
                pose = nice(object.get_pose())
                line = f'{body}  |  Name: {n}, Pose: {pose}'
                print(line)

            else:
                print(f'{body}  |  Name: {get_name(body)}, not in BODY_TO_OBJECT')

        print('----------------')
        print(f'PART II: world objects | {self.summarize_all_types()}')
        for body, object in BODY_TO_OBJECT.items():
            ## print class inheritance, if any
            typ_str = object._type()
            # if object._type().lower() in self.sup_categories:
            #     typ = object._type().lower()
            #     typ_str = ''
            #     while typ in self.sup_categories:
            #         typ_str = f"{self.sup_categories[typ].capitalize()} -> {typ_str}"
            #         typ = self.sup_categories[typ]

            line = f'{body}\t  |  {typ_str}: {object.name}'
            if isinstance(body, tuple) and len(body) == 2:
                body, joint = body
                pose = get_joint_position(body, joint)
                if hasattr(object, 'handle_link') and object.handle_link is not None:
                    line += f'\t|  Handle: {get_link_name(body, object.handle_link)}'
            elif isinstance(body, tuple) and len(body) == 3:
                body, _, link = body
                pose = get_link_pose(body, link)
            else:
                pose = get_pose(body)
            print(f"{line}\t|  Pose: {nice(pose)}")
        print('----------------')

    def remove_body_from_planning(self, body):
        if body == None: return
        category = self.BODY_TO_OBJECT[body].category
        obj = self.BODY_TO_OBJECT.pop(body)
        self.OBJECTS_BY_CATEGORY[category] = [
            o for o in self.OBJECTS_BY_CATEGORY[category] if not
            (o.body == obj.body and o.link == obj.link and o.joint == obj.joint)
        ]

    def remove_object(self, object):
        body = object.body

        ## remove all objects initiated by the body
        bodies = [body]
        if object.doors != None:
            bodies += [(body, j) for j in object.doors]
        if object.drawers != None:
            bodies += [(body, j) for j in object.drawers]
        bodies += [(body, None, l) for l in object.surfaces + object.spaces]
        bodies += [bb for bb in self.BODY_TO_OBJECT.keys() if isinstance(bb, tuple) and bb[0] == body and bb not in bodies]
        for b in bodies:
            if b in self.BODY_TO_OBJECT:
                category = self.BODY_TO_OBJECT[b].category
                obj = self.BODY_TO_OBJECT.pop(b)
                self.OBJECTS_BY_CATEGORY[category] = [
                    o for o in self.OBJECTS_BY_CATEGORY[category] if not
                    (o.body == obj.body and o.link == obj.link and o.joint == obj.joint)
                ]
                if hasattr(obj, 'supporting_surface') and isinstance(obj.supporting_surface, Surface):
                    surface = obj.supporting_surface
                    surface.supported_objects.remove(obj)

        remove_body(body)

    def name_to_body(self, name):
        name = name.lower()
        possible = {}
        for body, obj in self.BODY_TO_OBJECT.items():
            if name == obj.shorter_name:
                return body
            if name in obj.shorter_name:
                possible[body] = obj
        if len(possible) >= 1:
            counts = {b: len(o.shorter_name) for b, o in possible.items()}
            counts = dict(sorted(counts.items(), key=lambda item: item[1]))
            return list(counts.keys())[0]
            # return possible[0]
        return None

    def name_to_object(self, name):
        if self.name_to_body(name) == None:
            return name  ## object doesn't exist
        return self.BODY_TO_OBJECT[self.name_to_body(name)]

    def cat_to_bodies(self, cat):
        bodies = []
        objects = []
        if cat in self.OBJECTS_BY_CATEGORY:
            objects.extend(self.OBJECTS_BY_CATEGORY[cat])
        if cat in self.sub_categories:
            for c in self.sub_categories[cat]:
                objects.extend(self.OBJECTS_BY_CATEGORY[c])
        for o in objects:
            if o.link != None:
                bodies.append((o.body, o.joint, o.link))
            elif o.joint != None:
                bodies.append((o.body, o.joint))
            else:
                bodies.append(o.body)
        bodies = list(set(bodies))
        return bodies

    def cat_to_objects(self, cat):
        bodies = self.cat_to_bodies(cat)
        return [self.BODY_TO_OBJECT[b] for b in bodies]

    def open_doors_drawers(self, body):
        doors, drawers = self.get_doors_drawers(body)
        for joint in doors + drawers:
            open_joint(body, joint)

    def close_doors_drawers(self, body):
        doors, drawers = self.get_doors_drawers(body)
        for joint in doors + drawers:
            close_joint(body, joint)

    def close_all_doors_drawers(self):
        doors = []
        drawers = []
        if 'door' in self.OBJECTS_BY_CATEGORY:
            doors = [(o.body, o.joint) for o in self.OBJECTS_BY_CATEGORY['door']]
        if 'drawer' in self.OBJECTS_BY_CATEGORY:
            drawers = [(o.body, o.joint) for o in self.OBJECTS_BY_CATEGORY['drawer']]
        for body, joint in doors + drawers:
            close_joint(body, joint)

    def open_all_doors_drawers(self):
        doors = [(o.body, o.joint) for o in self.OBJECTS_BY_CATEGORY['door']]
        drawers = [(o.body, o.joint) for o in self.OBJECTS_BY_CATEGORY['drawer']]
        for body, joint in doors + drawers:
            open_joint(body, joint)

    def open_joint_by_name(self, name, pstn=None):
        body, joint = self.name_to_body(name)
        open_joint(body, joint, pstn=pstn)

    def close_joint_by_name(self, name):
        body, joint = self.name_to_body(name)
        close_joint(body, joint)

    def toggle_joint_by_name(self, name):
        body, joint = self.name_to_body(name)
        toggle_joint(body, joint)

    def put_on_surface(self, obj, surface='hitman_tmp', OAO=False):
        if isinstance(obj, str):
            obj = self.name_to_object(obj)
        if isinstance(obj, int):
            obj = self.BODY_TO_OBJECT[obj]
        surface_obj = self.name_to_object(surface)
        surface_obj.place_obj(obj)
        world_to_surface = surface_obj.get_pose()

        point, quat = obj.get_pose()
        x,y,z = point
        ## hack to be closer to edge
        if 'shelf' in surface:
            surface_to_obj = ((-0.2, 0, -0.2), (0, 0, 1, 0))
            (a, b, _), _ = multiply(world_to_surface, surface_to_obj)
            obj.set_pose(((a, b, z), quat))
            # obj.set_pose(((1, 4.4, z), quat))
            # obj.set_pose(((1.6, 4.5, z), quat)) ## vertical orientation
        elif 'braiser_bottom' in surface: ## for testing
            (a, b, c), _ = world_to_surface
            obj.set_pose(((0.55, b, z), (0, 0, 0.36488663206619243, 0.9310519565198234)))
        elif 'braiser' in surface:
            (a, b, c), quat = world_to_surface
            obj.set_pose(((a, b, z), quat))
        elif 'faucet_platform' in surface:
            (a, b, c), quat = world_to_surface
            obj.set_pose(((a-0.2, b, z), quat))
        elif 'front_' in surface and '_stove' in surface:
            obj.set_pose(((0.55, y, z), quat))
        elif 'hitman_tmp' in surface: ## microwave or toaster
            quat = (0, 0, 1, 0) ## facing out
            obj.set_pose(((0.4, 6.4, z), quat))
        elif 'tmp' in surface: ## egg
            if y > 9: y = 8.9
            obj.set_pose(((0.7, y, z), quat))

        if OAO: ## one and only
            self.remove_body_from_planning(self.name_to_body(surface))

    def put_in_space(self, obj, space='hitman_drawer_top', xyzyaw=None, learned=False):
        left_drawer = self.name_to_object(space)
        if learned:
            ## one possible pose put into hitman_drawer_top
            pose = {'hitman_drawer_top':
                        ((1.0113178491592407, 7.475691795349121, 0.7000163197517395),
                        (0, 0, 0.3178013671332143, 0.9481573134497528))
                    }[space]
            xyzyaw = list(pose[0])
            xyzyaw.append(euler_from_quat(pose[1])[-1])
            ## xyzyaw = (1.093, 7.088, 0.696, 2.8)
        left_drawer.place_obj(obj, xyzyaw)

    def refine_marker_obstacles(self, marker, obstacles):
        ## for steerables
        parent = self.BODY_TO_OBJECT[marker].grasp_parent
        if parent != None and parent in obstacles:
            obstacles.remove(parent)
        return obstacles

    @property
    def max_delta(self):
        return self.max_velocities * self.time_step
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.robot)


#######################################################

class State(object):
    def __init__(self, world, objects=[], attachments={}, facts=[], variables={}):
        self.world = world
        if len(objects) == 0:
            objects = get_bodies()
            objects.remove(self.robot)
        self.objects = list(objects)

        if len(attachments) == 0:
            attachments = copy.deepcopy(world.ATTACHMENTS)
        self.attachments = dict(attachments) # TODO: relative pose
        self.facts = list(facts) # TODO: make a set?
        self.variables = defaultdict(lambda: None)
        self.variables.update(variables)
        self.assign()
        self.saver = WorldSaver(bodies=self.bodies)

        ## serve as problem for streams
        self.gripper = None
        self.grasp_types = ['top'] ##, 'side']
        ## allowing both types causes trouble when the AConf used for generating IK isn't the same as the one during execution


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

    @property
    def robot(self):
        return self.world.robot # TODO: use facts instead
    @property
    def robots(self):
        return [self.world.robot]
    @property
    def bodies(self):
        return [self.robot] + self.objects
    @property
    def regions(self):
        return [obj for obj in self.objects if isinstance(obj, Region)]
    @property
    def floors(self):
        return self.world.floors
    @property
    def fixed(self):   ## or the robot will go through tables
        objs = [obj for obj in self.objects if obj not in self.movable]
        objs = [o for o in objs if self.world.BODY_TO_OBJECT[o].category != 'floor']
        return objs
        # return [obj for obj in self.objects if isinstance(obj, Region) or isinstance(obj, Environment)]
    @property
    def movable(self): ## include steerables if want to exclude them when doing base motion plannig
        return self.world.movable
        # return [self.robot] + self.world.cat_to_bodies('moveable') ## + self.world.cat_to_bodies('steerable')
        # return [obj for obj in self.objects if obj not in self.fixed]
    @property
    def obstacles(self):
        return {obj for obj in self.objects if obj not in self.regions} - set(self.attachments)
    def restore(self): # TODO: could extend WorldSaver
        self.saver.restore()
    def scramble(self):
        set_zero_world(self.bodies)
    def copy(self): # __copy__
        return State(self.world, objects=self.objects, attachments=self.attachments,
                     facts=self.facts, variables=self.variables) # TODO: use instead of new_state
    def new_state(self, objects=None, attachments=None, facts=None, variables=None):
        # TODO: could also just update the current state
        if objects is None:
            objects = self.objects
        if attachments is None:
            attachments = self.attachments
        if facts is None:
            facts = self.facts
        if variables is None:
            variables = self.variables
        return State(self.world, objects=objects, attachments=attachments, facts=facts, variables=variables)
    def assign(self):
        # TODO: topological sort
        for attachment in self.attachments.values():
            attachment.assign()
        return self
    def filter_facts(self, predicate): # TODO: predicates
        return [fact[1:] for fact in self.facts if fact[0].lower() == predicate.lower()]
    def apply_action(self, action): # Transition model
        if action is None:
            return self
        assert isinstance(action, Action)
        return action.transition(self.copy())
    def camera_observation(self, include_rgb=False, include_depth=False, include_segment=False):
        if not (self.world.args.camera or include_rgb or include_depth or include_segment):
            return None
        [camera] = self.robot.cameras
        rgb, depth, seg, pose, matrix = camera.get_image(
            segment=(self.world.args.segment or include_segment), segment_links=False)
        if not include_rgb:
            rgb = None
        if not include_depth:
            depth = None
        if not include_segment:
            seg = None
        return CameraImage(rgb, depth, seg, pose, matrix)
    def sample_observation(self, include_conf=False, include_poses=False,
                           include_facts=False, include_variables=False, **kwargs): # Observation model
        # TODO: could technically also not require robot, camera_pose, or camera_matrix
        # TODO: make confs and poses state variables
        #robot_conf = self.robot.get_positions() if include_conf else None
        robot_conf = BodySaver(self.robot) if include_conf else None # TODO: unknown base but known arms
        obj_poses = {obj: get_pose(obj) for obj in self.objects} if include_poses else None
        facts = list(self.facts) if include_facts else None
        variables = dict(self.variables) if include_variables else None
        image = None  ##self.camera_observation(**kwargs)
        return Observation(self, robot_conf=robot_conf, obj_poses=obj_poses,
                           facts=facts, variables=variables, image=image)

    def get_facts(self, init_facts=[], conf_saver=None, obj_poses=None):
        from bullet.worlds.kitchen_worlds import GRASPABLES
        robot = self.world.robot.body
        cat_to_bodies = self.world.cat_to_bodies
        cat_to_objects = self.world.cat_to_objects
        name_to_body = self.world.name_to_body
        BODY_TO_OBJECT = self.world.BODY_TO_OBJECT

        def get_conf(joints):
            if conf_saver == None:
                return get_joint_positions(robot, joints)
            return [conf_saver.conf[conf_saver.joints.index(n)] for n in joints]

        def equal(tup1, tup2, epsilon=0.001):
            if isinstance(tup1, float):
                return abs(tup1 - tup2) < epsilon
            if len(tup1) == 2:
                return equal(tup1[0], tup2[0]) and equal(tup1[1], tup2[1])
            return all([abs(tup1[i]-tup2[i]) < epsilon for i in range(len(tup1))])

        def get_base_conf():
            base_joints = get_group_joints(robot, 'base')
            initial_bq = Conf(robot, base_joints, get_conf(base_joints))
            for fact in init_facts:
                if fact[0] == 'bconf' and equal(fact[1].values, initial_bq.values):
                    return fact[1]
            return initial_bq

        def get_arm_conf(arm):
            arm_joints = get_arm_joints(robot, arm)
            conf = Conf(robot, arm_joints, get_conf(arm_joints))
            for fact in init_facts:
                if fact[0] == 'aconf' and fact[1] == arm and equal(fact[2].values, conf.values):
                    return fact[2]
            return conf

        def get_body_pose(body):
            if obj_poses == None:
                pose = Pose(body, get_pose(body))
            else:  ## in observation
                pose = obj_poses[body]

            for fact in init_facts:
                if fact[0] == 'pose' and fact[1] == body and equal(fact[2].value, pose.value):
                    return fact[2]
            return pose

        def get_link_position(body):
            position = Position(body)
            for fact in init_facts:
                if fact[0] == 'position' and fact[1] == body and equal(fact[2].value, position.value):
                    return fact[2]
            return position

        def get_link_pose(body):
            pose = LinkPose(body, BODY_TO_OBJECT[body])
            for fact in init_facts:
                if fact[0] == 'linkpose' and fact[1] == body and equal(fact[2].value, pose.value):
                    return fact[2]
            return pose

        def get_grasp(body, attachment):
            grasp = pr2_grasp(body, attachment.grasp_pose)
            for fact in init_facts:
                if fact[0] == 'grasp' and fact[1] == body and equal(fact[2].value, grasp.value):
                    return fact[2]
            return grasp

        ## ---- robot conf ------------------
        initial_bq = get_base_conf()
        init = [('CanMove',), ('CanPull',),
                ('BConf', initial_bq), ('AtBConf', initial_bq),
                Equal(('PickCost',), 1), Equal(('PlaceCost',), 1)]
        # if name_to_body('sink') != None:
        #     init += ('Sink', name_to_body('sink'))

        for arm in ARM_NAMES:
            # for arm in problem.arms:
            conf = get_arm_conf(arm)
            init += [('Arm', arm), ('AConf', arm, conf),
                     ('DefaultConf', arm, conf), ('AtAConf', arm, conf)]
            if arm in ['left']:
                init += [('Controllable', arm)]

        HANDS_EMPTY = {arm: True for arm in ARM_NAMES}
        for arm, empty in HANDS_EMPTY.items():
            if empty: init += [('HandEmpty', arm)]
            # else: print(f' \n\n {arm} hand isnt empy')


        ## ---- object poses / grasps ------------------
        graspables = [o.body for o in cat_to_objects('object') if o.category in GRASPABLES]
        graspables = set(cat_to_bodies('moveable') + graspables)
        for body in graspables:

            init += [('Graspable', body)]
            pose = get_body_pose(body)

            if body in self.attachments:
                attachment = self.attachments[body]
                if not isinstance(attachment, ObjAttachment):
                    grasp = get_grasp(body, attachment)
                    arm = 'right'
                    if get_link_name(robot, attachment.parent_link).startswith('l_'):
                        arm = 'left'
                    HANDS_EMPTY[arm] = False ## only able to grasp one thing
                    init += [('Grasp', body, grasp), ('AtGrasp', arm, body, grasp)]
                else:
                    init += [('Pose', body, pose), ('AtPose', body, pose)]
            else:
                init += [('Pose', body, pose), ('AtPose', body, pose)]

            ## potential places to put on
            for surface in cat_to_bodies('supporter') + cat_to_bodies('surface'):
                init += [('Stackable', body, surface)]
                if is_placement(body, surface):
                    # print('   found supported', body, surface)
                    init += [('Supported', body, pose, surface)]

            ## potential places to put in
            for space in cat_to_bodies('container') + cat_to_bodies('space'):
                init += [('Containable', body, space)]
                # if is_contained(body, space):
                #     print('   found contaied', body, space)
                #     init += [('Contained', body, pose, space)]


        ## ---- cart poses / grasps ------------------
        for body in cat_to_bodies('steerable'):
            pose = get_body_pose(body)
            init += [('Pose', body, pose), ('AtPose', body, pose)]

            obj = BODY_TO_OBJECT[body]
            for marker in obj.grasp_markers:
                init += [('Marked', body, marker.body)]


        ## ---- object joint positions ------------- TODO: may need to add to saver
        knobs = cat_to_bodies('knob')
        for body in cat_to_bodies('drawer') + cat_to_bodies('door') + knobs:
            if BODY_TO_OBJECT[body].handle_link == None:
                continue
            ## initial position
            position = get_link_position(body)  ## Position(body)
            pose = get_link_pose(body)  ## LinkPose(body)
            init += [('Joint', body), ('LinkPose', body, pose), ('AtLinkPose', body, pose),
                     ('Position', body, position), ('AtPosition', body, position),
                     ('IsClosedPosition', body, position),
                     ('IsJointTo', body, body[0])
                     ]
            if body in knobs:
                controlled = BODY_TO_OBJECT[body].controlled
                if controlled != None:
                    init += [('ControlledBy', controlled, body)]

        ## ---- object types -------------
        for cat in self.world.OBJECTS_BY_CATEGORY:
            if cat.lower() == 'moveable': continue
            objects = cat_to_bodies(cat)
            init += [(cat, obj) for obj in objects]

        ## ---- those added to state.variables[label, body]
        for k in self.variables:
            init += [(k[0], k[1])]

        ## --- world configuration
        wconf = self.get_wconf(init)
        init += [('WConf', wconf), ('InWConf', wconf)]
        print('initial wconf', wconf.printout())

        ## --- for testing IK
        # lid = self.world.name_to_body('braiserlid')
        # surface = self.world.name_to_body('indigo_tmp')
        # pose = Pose(lid, xyzyaw_to_pose((0.694, 8.694, 0.814, 1.277)))
        # init += [('Pose', lid, pose), ('MagicPose', lid, pose), ('Supported', lid, pose, surface)]

        # ## --- for testing containment
        # egg = self.world.name_to_body('egg')
        # fridge = self.world.name_to_body('fridge')
        # init += [('MagicalObj1', egg), ('MagicalObj2', fridge)]

        return init

    def get_joint_facts(self):
        init = []
        for body in self.world.cat_to_bodies('drawer') + self.world.cat_to_bodies('door'):
            if self.world.BODY_TO_OBJECT[body].handle_link == None:
                continue
            ## initial position
            init += [('AtPosition', body, Position(body))]
        return init

    def get_wconf(self, init=None):
        if init == None:
            init = self.get_joint_facts()
        poses = {i[1]: i[2] for i in init if i[0] == 'AtPose'}
        positions = {i[1]: i[2] for i in init if i[0] == 'AtPosition'}
        wconf = WConf(poses, positions)
        return wconf

    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, self.objects)

#######################################################

class Outcome(object):
    def __init__(self, collisions=[], violations=[]):
        self.collisions = list(collisions)
        self.violations = list(violations)
    def __len__(self):
        return max(map(len, [self.collisions, self.violations]))
    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, str_from_object(self.__dict__))

def analyze_outcome(state):
    # TODO: possibly move to State and rename inconsistencies
    robots = {state.robot}
    violations = set()
    for body in robots:
        if not body.within_limits():
            violations.add(body)

    movable = robots | set(state.movable)
    bodies = robots | set(state.obstacles) # TODO: attachments
    collisions = set()
    for body1, body2 in product(movable, bodies):
        if (body1 != body2) and pairwise_collision(body1, body2, max_distance=0.):
            collisions.add(frozenset([body1, body2]))

    return Outcome(collisions=collisions, violations=violations)

#######################################################

class Observation(object):
    # TODO: just update a dictionary for everything
    def __init__(self, state, robot_conf=None, obj_poses=None, image=None, facts=None, variables=None, collision=False):
        self.state = state
        self.robot_conf = robot_conf
        self.obj_poses = obj_poses
        self.rgb_image = self.depth_image = self.seg_image = self.camera_matrix = self.camera_pose = None
        if image is not None:
            self.rgb_image, self.depth_image, self.seg_image, self.camera_matrix, self.camera_pose = image
        # self.facts = facts
        self.variables = variables
        self.collision = collision
        # TODO: noisy conf, pose, RGB, and depth observations
        # TODO: map observation
    @property
    def facts(self):
        return self.state.get_facts(conf_saver=self.robot_conf.conf_saver,
                                    obj_poses=self.obj_poses)
    @property
    def objects(self):
        if self.obj_poses is None:
            return None
        return sorted(self.obj_poses) # TODO: attachments
    # @property
    # def regions(self):
    #     if self.objects is None:
    #         return None
    #     return [obj for obj in self.objects if isinstance(obj, Region)]
    @property
    def obstacles(self):
        if self.objects is None:
            return None
        return [obj for obj in self.objects if obj in self.state.obstacles]
    def assign(self): # TODO: rename to update_pybullet
        if self.robot_conf is not None:
            self.robot_conf.restore() # TODO: sore all as Savers instead?
            #self.robot.set_positions(observation.robot_conf)
        if self.obj_poses is not None:
            for obj, pose in self.obj_poses.items():
                set_pose(obj, pose)
        return self
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.robot_conf, sorted(self.obj_poses))

#######################################################

class Process(object):
    def __init__(self, world, name=None, **kwargs):
        self.world = world
        self.name = name
        self.runtimes = []
        self.outcomes = [] # TODO: outcome visiblity
    @property
    def robot(self):
        return self.world.robot
    @property
    def time_step(self):
        return self.world.time_step
    @property
    def max_velocities(self):
        return self.world.max_velocities
    @property
    def max_delta(self):
        return self.world.max_delta
    @property
    def num_steps(self):
        return len(self.runtimes)
    @property
    def current_time(self):
        return self.time_step*self.num_steps
    def initialize(self, state):
        # TODO: move creation of bodies to agents
        self.state = state ## YANG< HPN
        return state # TODO: return or just update?
    def evolve(self, state, ONCE=False, verbose=False):
        start_time = time.time()
        # new_state = self.wrapped_transition(state)
        # with LockRenderer(lock=not self.world.args.viewer):  ## 0.05 sec delay
        if True:
            new_state = self.wrapped_transition(state, ONCE=ONCE, verbose=verbose)
            if verbose: print(f'  evolve \ finished wrapped_transition inner in {round(time.time() - start_time, 4)} sec')
        if verbose: print(f'  evolve \ finished wrapped_transition 2 in {round(time.time()-start_time, 4)} sec')

        ## --------- added by YANG to stop simulation if action is None ---------
        if ONCE and new_state is None:
            return None
            # new_state = state
        ## ----------------------------------------------------------------------

        outcome = analyze_outcome(new_state) # TODO: include delta from state (i.e. don't penalize ongoing issues)
        self.outcomes.append(outcome)
        if self.world.prevent_collisions and outcome:
            print(outcome)
            new_state = state.assign()
        self.runtimes.append(elapsed_time(start_time))
        return new_state
    def wrapped_transition(self, state, ONCE=False, verbose=False):
        raise NotImplementedError()

#######################################################

class Exogenous(Process):
    def __init__(self, world, **kwargs):
        super(Exogenous, self).__init__(world, **kwargs)
        self.states = []
    def wrapped_transition(self, state, **kwargs):
        #self.states.append(state) # TODO: before, after, or both
        new_state = self.transition(state.copy())
        if new_state is None:
            new_state = state
        assert isinstance(new_state, State)
        self.states.append(state)
        return new_state
    def transition(self, state): # Operates directly on the state
        raise NotImplementedError()

#######################################################

class Agent(Process): # Decision
    # TODO: make these strings
    requires_conf = requires_poses = requires_facts = requires_variables = \
        requires_rgb = requires_depth = requires_segment = False # requires_cloud
    def __init__(self, world, **kwargs):
        super(Agent, self).__init__(world, **kwargs)
        self.world = world
        self.observations = []
        self.actions = []
    def wrapped_transition(self, state, ONCE=False, verbose=False):
        # TODO: move this to another class
        start_time = time.time()
        observation = state.sample_observation(
            include_conf=self.requires_conf, include_poses=self.requires_poses,
            include_facts=self.requires_facts, include_variables=self.requires_variables,
            include_rgb=self.requires_rgb, include_depth=self.requires_depth,
            include_segment=self.requires_segment)  # include_cloud=self.requires_cloud,
        if verbose: print(f'   wrapped_transition \ made observation in {round(time.time() - start_time, 4)} sec')
        start_time = time.time()
        if self.world.scramble:
            # if not self.requires_conf or self.requires_cloud:
            state.scramble()
        action = self.policy(observation)
        if verbose: print(f'   wrapped_transition \ chosen action in {round(time.time() - start_time, 4)} sec')
        start_time = time.time()
        state.restore()
        self.observations.append(observation)
        self.actions.append(action)
        result = state.apply_action(action)
        if verbose: print(f'   wrapped_transition \ applied action in {round(time.time() - start_time, 4)} sec')

        ## --------- added by YANG to stop simulation if action is None ---------
        if ONCE and action == None: result = None
        ## ----------------------------------------------------------------------
        return result
    def policy(self, observation): # Operates indirectly on the state
        raise NotImplementedError()

#######################################################

def evolve_processes(state, processes=[], max_steps=INF, ONCE=False, verbose=False):
    # TODO: explicitly separate into exogenous and agent?
    world = state.world
    time_step = world.time_step
    # parameter = add_parameter(name='Real-time / sim-time', lower=0, upper=5, initial=0)
    # button = add_button(name='Pause / Play')
    start_time = time.time()
    current_time = 0.
    for agent in processes:
        state = agent.initialize(state)

    facts = state.facts
    for step in irange(max_steps):
        if verbose:
            print('Step: {} | Current time: {:.3f} | Elapsed time: {:.3f}'.format(step, current_time, elapsed_time(start_time)))

        # TODO: sample nearby and then extend
        for agent in processes:
            state = agent.evolve(state, ONCE=ONCE, verbose=verbose)

            ## --------- added by YANG to stop simulation if action is None ---------
            if ONCE and state == None: return None
            ## ----------------------------------------------------------------------------

        # if verbose: print('state add', [f for f in state.facts if f not in facts])
        # if verbose: print('state del', [f for f in facts if f not in state.facts])

        current_time += time_step
        # wait_for_duration(read_parameter(parameter) * time_step)
        #wait_if_gui()
    return state
