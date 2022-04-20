from examples.pybullet.namo.stream import BASE_LINK
from pybullet_tools.pr2_utils import attach_viewcone, draw_viewcone, get_viewcone_base, set_group_conf, get_arm_joints
from pybullet_tools.utils import get_joint_name, get_joint_position, get_link_name, get_link_pose, get_pose, set_pose, \
    joint_from_name, get_movable_joints, get_joint_positions, set_joint_position, set_joint_positions, link_from_name, \
    get_all_links, BodySaver, get_collision_data, tform_oobb, oobb_from_data, aabb_from_oobb, get_aabb, \
    remove_handles, remove_body, get_custom_limits, get_subtree_aabb, get_image_at_pose, draw_pose, multiply, Pose, RED, \
    all_between, get_name, dump_link, dump_joint, dump_body, PoseSaver, get_color, GREEN, unit_pose, \
    add_text, AABB, Point, Euler, PI, add_line, YELLOW, BLACK, remove_handles, get_com_pose, Pose, invert, \
    stable_z, get_joint_descendants, get_link_children, get_joint_info, get_links, link_from_name, \
    get_min_limit, get_max_limit, get_link_parent
import numpy as np
import pybullet as p

class Index(object):
    # TODO: unify with some of the following?
    # https://pypi.org/project/pybullet-planning/
    # https://github.com/rachelholladay/pb_robot
    # https://github.com/carismoses/pb_robot
    # https://github.com/rachelholladay/tampExample
    def __int__(self):
        raise NotImplementedError()
    def __eq__(self, other):
        if self is other:
            return True
        try:
            return int(self) == int(other)
        except ValueError:
            return False
    def __ne__(self, other):
        return not self == other
    def __lt__(self, other): # For heapq on python3
        return int(self) < int(other)
    def __hash__(self):
        return hash(int(self))


class Joint(Index):
    def __init__(self, body, index):
        self.body = body
        self.index = index
    @property
    def name(self):
        return get_joint_name(self.body, self.index)
    def __int__(self):
        return self.index
    def get_position(self):
        return get_joint_position(self.body, self.index)
    def __repr__(self):
        return self.name


class Link(Index):
    def __init__(self, body, index):
        self.body = body
        self.index = index
    @property
    def name(self):
        return get_link_name(self.body, self.index)
    def __int__(self):
        return self.index
    def get_pose(self):
        return get_link_pose(self.body, self.index)
    def __repr__(self):
        return self.name

#######################################################

class Object(Index):
    def __init__(self, body, joint=None, link=None, category=None, name=None, collision=True):
        self.body = body
        self.joint = joint
        self.link = link

        ## automatically categorize object by class
        if category is None:
            category = self.__class__.__name__.lower()
        category = category.lower()
        self.category = category
        self.name = name

        self.collision = collision
        self.handles = []
        self.text_handle = None
        self.text = ''
        self.draw()

        ## in order to move object with objects attached in it
        self.world = None
        self.grasp_markers = []
        self.grasp_parent = None
        self.doors = None
        self.drawers = None
        self.surfaces = []
        self.spaces = []
        self.supported_objects = []
        self.events = []
        self.is_box = False

    ## =============== put other object on top of object =============
    ##
    def support_obj(self, obj):
        obj.supporting_surface = self
        if obj not in self.supported_objects:
            self.supported_objects.append(obj)

    def place_new_obj(self, obj_name):
        from bullet.utils import sample_obj_on_body_link_surface
        BODY_TO_OBJECT = self.world.BODY_TO_OBJECT
        body = sample_obj_on_body_link_surface(obj_name, self.body, self.link)
        obj = self.world.add_object(Object(body, category=obj_name))
        self.support_obj(obj)
        return obj

    def place_obj(self, obj, xyzyaw=None):
        from bullet.utils import sample_obj_on_body_link_surface, nice
        if isinstance(obj, str):
            obj = self.place_new_obj(obj)
        # obj_name = obj.category.capitalize().replace('bottle','Bottle')
        # if obj_name == 'Moveable':
        #     obj_name = obj
        x, y, z, yaw = sample_obj_on_body_link_surface(
            obj, self.body, self.link, PLACEMENT_ONLY=True)
        z = stable_z(obj, self.body, self.link)
        print(f'placed {obj.name} on surface {self.name} at point {nice((x, y, z))}')
        obj.set_pose(Pose(point=Point(x=x, y=y, z=z), euler=Euler(yaw=yaw)))
        self.support_obj(obj)
        return obj
    ##
    ## ====================================================================

    def __int__(self):
        if self.body is None:
            return id(self) # TODO: hack
        if self.joint != None:
            return (self.body, self.joint)
        return self.body

    def __repr__(self):
        #return repr(int(self))
        return self.name

    def _type(self):
        return self.__class__.__name__

    def get_pose(self):
        return get_pose(self.body)

    def set_pose(self, conf):
        links = [get_link_name(self.body, l) for l in get_links(self.body)]
        if 'base' in links:
            set_group_conf(self.body, 'base', conf)
        else:
            set_pose(self.body, conf)
    def get_joint(self, joint): # int | str
        # TODO: unify with get_joint in pybullet-planning
        try:
            return int(joint)
        except ValueError:
            return joint_from_name(self.body, joint)
    def get_joints(self, joints=None):
        if joints is None:
            return get_movable_joints(self.body) # get_joints | get_movable_joints
        return tuple(map(self.get_joint, joints))
    def get_joint_position(self, *args, **kwags):
        return get_joint_positions(self.body, [self.get_joint(*args, **kwags)])[0]
    def get_joint_positions(self, *args, **kwags):
        return get_joint_positions(self.body, self.get_joints(*args, **kwags))

    def set_joint_position(self, joint, *args, **kwargs):
        ans = set_joint_position(self.body, self.get_joint(joint), *args, **kwargs)

        ## when joints move, objects on child link are generated again
        BODY_TO_OBJECT = self.world.BODY_TO_OBJECT
        child_links = get_joint_descendants(self.body, joint)
        for link in child_links:
            if (self.body, None, link) in BODY_TO_OBJECT:
                space = BODY_TO_OBJECT[(self.body, None, link)]
                for obj in space.objects_inside:
                    space.place_obj(obj)
        return ans

    def set_joint_positions(self, joints, *args, **kwargs):
        return set_joint_positions(self.body, self.get_joints(joints), *args, **kwargs)

    def get_link(self, link): # int | str
        try:
            return int(link)
        except ValueError:
            return link_from_name(self.body, link)
    #link_from_name = get_link
    def get_links(self, links=None):
        if links is None:
            return get_all_links(self.body)
        return tuple(map(self.get_joint, links))
    def get_link_pose(self, link=BASE_LINK):
        return get_link_pose(self.body, self.get_link(link))

    def create_saver(self, **kwargs):
        # TODO: inherit from saver
        return BodySaver(self.body, **kwargs)
    def get_link_oobb(self, link, index=0):
        # TODO: get_trimesh_oobb
        link = self.get_link(link)
        surface_data = get_collision_data(self.body, link=link)[index]
        pose = get_link_pose(self, link) # TODO: combine for multiple links
        surface_oobb = tform_oobb(pose, oobb_from_data(surface_data))
        # draw_oobb(surface_oobb, color=RED)
        return surface_oobb
    def get_link_aabb(self, *args, **kwargs):
        return aabb_from_oobb(self.get_link_oobb(*args, **kwargs))
    def get_aabb(self, *args, **kwargs):
        # TODO: is there an easier way to to bind all of these methods?
        return get_aabb(self.body, *args, **kwargs)

    def draw(self, text='', **kwargs):
        self.erase()
        if self.name is not None and self.category != 'door':
            # TODO: attach to the highest link (for the robot)
            self.handles.append(add_body_label(self.body, name=self.name, text=text, **kwargs))
        #self.handles.extend(draw_pose(Pose(), parent=self.body, **kwargs))
        return self.handles
    def erase(self):
        remove_handles(self.handles)
        self.handles = []
    def add_text(self, text):
        if self.text_handle != None:
            p.removeUserDebugItem(self.text_handle)
            self.text += '_'
        self.text += text
        p.addUserDebugText(self.text, textPosition=(0, 0, .5), textColorRGB=(1, 0, 0),  # textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
    def is_active(self):
        return self.body is not None
    def remove(self): # TODO: overload del
        self.erase()
        if self.is_active():
            remove_body(self.body)
            self.body = None
    def add_grasp_marker(self, object):
        if object not in self.grasp_markers:
            self.grasp_markers.append(object)
        self.world.BODY_TO_OBJECT[object].grasp_parent = self.body
    def add_events(self, events):
        self.events.extend(events)
    def add_event(self, event):
        self.events.append(event)

    @property
    def pybullet_name(self, event):
        if self.joint == None and self.link != None:
            return (self.body, self.joint, self.link)
        elif self.joint != None and self.link == None:
            return (self.body, self.joint)
        else:
            return self.body
    @property
    def shorter_name(self):
        return self.name.replace('counter#1--', '')
    @property
    def debug_name(self):
        return f'{self.name}|{self.body}'
        # return f'{self.shorter_name}|{self.body}'

class Moveable(Object):
    def __init__(self, body, **kwargs):
        super(Moveable, self).__init__(body, collision=False, **kwargs)
        self.supporting_surface = None

class Steerable(Object):
    def __init__(self, body, **kwargs):
        super(Steerable, self).__init__(body, collision=False, **kwargs)

class Supporter(Object):
    def __init__(self, body, **kwargs):
        super(Supporter, self).__init__(body, collision=False, **kwargs)
        self.supported_objects = None

class Region(Object):
    def __init__(self, body, **kwargs):
       super(Region, self).__init__(body, collision=False, **kwargs)

class Stove(Region):
    def __init__(self, body, **kwargs):
        super(Stove, self).__init__(body, **kwargs)

class Floor(Region):
    def __init__(self, body, **kwargs):
        super(Floor, self).__init__(body, **kwargs)
        self.category = 'floor'
        self.name = 'floor1'

class Environment(Region):
    def __init__(self, body, **kwargs):
        super(Environment, self).__init__(body, **kwargs)

#######################################################

class Surface(Region):
    """ to support objects on top, like kitchentop and fridge shelves """
    def __init__(self, body, link, **kwargs):
        super(Region, self).__init__(body, link=link, **kwargs)
        self.name = get_link_name(body, link)
        self.supported_objects = []

    def support_obj(self, obj):
        obj.supporting_surface = self
        if obj not in self.supported_objects:
            self.supported_objects.append(obj)

    def place_new_obj(self, obj_name):
        from bullet.utils import sample_obj_on_body_link_surface
        BODY_TO_OBJECT = self.world.BODY_TO_OBJECT
        body = sample_obj_on_body_link_surface(obj_name, self.body, self.link)
        obj = self.world.add_object(Object(body, category=obj_name))
        self.world.put_on_surface(obj, surface=self.shorter_name)
        self.support_obj(obj)
        return obj

    def place_obj(self, obj):
        from bullet.utils import sample_obj_on_body_link_surface, nice
        if isinstance(obj, str):
            obj = self.place_new_obj(obj)
        # obj_name = obj.category.capitalize().replace('bottle','Bottle')
        # if obj_name == 'Moveable':
        #     obj_name = obj
        x, y, z, yaw = sample_obj_on_body_link_surface(
            obj, self.body, self.link, PLACEMENT_ONLY=True)
        z = stable_z(obj, self.body, self.link)
        print(f'placed {obj.name} on surface {self.name} at point {nice((x, y, z))}')
        obj.set_pose(Pose(point=Point(x=x,y=y,z=z), euler=Euler(yaw=yaw)))
        self.support_obj(obj)
        return obj

class Space(Region):
    """ to support object inside, like cabinets and drawers """
    def __init__(self, body, link, **kwargs):
        super(Region, self).__init__(body, link=link, **kwargs)
        self.name = get_link_name(body, link)
        self.objects_inside = []

    def place_and_attach(self, obj):
        from bullet.utils import create_attachment
        self.objects_inside.append(obj)
        attachment = create_attachment(self, self.link, obj, OBJ=True)
        self.world.ATTACHMENTS[obj] = attachment

    def place_new_obj(self, obj_name):
        from bullet.utils import sample_obj_in_body_link_space
        BODY_TO_OBJECT = self.world.BODY_TO_OBJECT
        body = sample_obj_in_body_link_space(obj_name, self.body, self.link)
        obj = self.world.add_object(Object(body, category=obj_name))
        self.place_and_attach(obj)
        return obj

    def place_obj(self, obj, xyzyaw=None):
        from bullet.utils import sample_obj_in_body_link_space, nice
        if xyzyaw == None:
            if isinstance(obj, str):
                self.place_new_obj(obj)
            obj_name = obj.category.capitalize().replace('bottle','Bottle')
            if obj_name == 'Moveable':
                obj_name = obj
            x, y, z, yaw = sample_obj_in_body_link_space(
                obj_name, self.body, self.link, PLACEMENT_ONLY=True)
        else:
            x, y, z, yaw = xyzyaw
        print(f'placed {obj.name} in space {self.name} at point {nice((x, y, z))}')
        obj.set_pose(Pose(point=Point(x=x,y=y,z=z), euler=Euler(yaw=yaw)))
        self.place_and_attach(obj)
        return obj

#######################################################

class ArticulatedObjectPart(Object):
    def __init__(self, body, joint, min_limit=None, max_limit=None, **kwargs):
        super(ArticulatedObjectPart, self).__init__(body, joint, collision=True, **kwargs)
        self.name = get_joint_name(body, joint)
        if min_limit == None:
            min_limit = get_min_limit(body, joint)
            max_limit = get_max_limit(body, joint)
        self.min_limit = min_limit
        self.max_limit = max_limit
        self.handle_link = self.find_handle_link(body, joint)
        self.handle_horizontal, self.handle_width = self.get_handle_orientation(body)

    def find_handle_link(self, body, joint):
        link = get_joint_info(body, joint).linkName.decode("utf-8")

        ## the only handle
        links = [l for l in get_links(body) if 'handle' in get_link_name(body, l)]
        if len(links) == 1:
            return links[0]

        ## when the substring matches
        if link.endswith('_link'):
            name = link[:link.index('_link')]
            links = [l for l in get_links(body) if name in get_link_name(body, l)]
            links = [l for l in links if 'handle' in get_link_name(body, l) or 'knob' in get_link_name(body, l)]
            if len(links) == 1:
                return links[0]
        else: ## try to find in children links
            links += [l for l in get_link_children(body, link)]

        words = self.name.split('_')
        if len(links) > 0:
            counts = {links[i]: sum([w in words for w in get_link_name(body, links[i]).split('_')]) for i in range(len(links))}
            counts = dict(sorted(counts.items(), key=lambda item: item[1], reverse=True))
            return list(counts.keys())[0]

        return link_from_name(body, link)

    def get_handle_orientation(self, body):
        aabb = get_aabb(body, self.handle_link)
        x,y,z = [aabb.upper[i] - aabb.lower[i] for i in range(3)]
        if y > z or x > z:
            return True, z
        return False, np.sqrt(x**2 + y**2)

    def get_handle_pose(self):
        return get_link_pose(self.body, self.handle_link)

    def get_pose(self):
        return self.get_handle_pose()


class Door(ArticulatedObjectPart):
    def __init__(self, body, **kwargs):
        super(Door, self).__init__(body, **kwargs)

class Drawer(ArticulatedObjectPart):
    def __init__(self, body, **kwargs):
        super(Drawer, self).__init__(body, **kwargs)

class Knob(ArticulatedObjectPart):
    def __init__(self, body, **kwargs):
        super(Knob, self).__init__(body, **kwargs)
        self.controlled = None

    def find_handle_link(self, body, joint):
        link = get_joint_info(body, joint).linkName.decode("utf-8")
        return get_link_parent(body, link_from_name(body, link))

    def add_controlled(self, body):
        self.controlled = body

#######################################################


class Robot(Object):
    def __init__(self, body, base_link=BASE_LINK, joints=None,
                 custom_limits={}, disabled_collisions={},
                 resolutions=None, weights=None, cameras=[], **kwargs):
        name = get_name(body)
        super(Robot, self).__init__(body, name=name, **kwargs)
        self.base_link = self.get_link(base_link)
        self.joints = self.get_joints(get_movable_joints(self.body) if joints is None else joints)
        self.custom_limits = dict(custom_limits)
        self.disabled_collisions = dict(disabled_collisions)
        self.resolutions = resolutions # TODO: default values if None
        self.weights = weights
        self.cameras = list(cameras)
        self.objects_in_hand = {'left': -1, 'right': -1}  ## problem with equal
    #@property
    #def joints(self):
    #    return self.active_joints
    def get_pose(self):
        return self.get_link_pose(self.base_link)
    def get_positions(self, joint_group='base', roundto=None):
        if joint_group == 'base':
            joints = self.joints
        else: ## if joint_group == 'left':
            joints = get_arm_joints(self.body, joint_group)
        positions = self.get_joint_positions(joints)
        if roundto == None:
            return positions
        return tuple([round(n, roundto) for n in positions])
    def set_base_positions(self, xytheta):
        set_group_conf(self.body, 'base', xytheta)
    def set_positions(self, positions, joints=None):
        if joints == None:
            joints = self.joints
        self.set_joint_positions(joints, positions)
    def get_limits(self, joints=None):
        if joints is None:
            joints = self.joints
        return get_custom_limits(self.body, joints, self.custom_limits)
    def get_aabb(self, *args, **kwargs):
        return get_subtree_aabb(self.body, self.base_link) # Computes the robot's axis-aligned bounding box (AABB)
    def within_limits(self, positions=None):
        if positions is None:
            positions = self.get_positions()
        lower_limits, upper_limits = self.get_limits()
        return all_between(lower_limits, positions, upper_limits)

    def get_objects_in_hands(self):
        objects = []
        for gripper in ['left', 'right']:
            if self.objects_in_hand[gripper] != -1:
                objects.append(self.objects_in_hand[gripper])
        return objects

    def has_object_in_hand(self, obj):
        return obj in [o.category.lower() for o in self.get_objects_in_hands()]
    #def draw(self, *args, **kwargs):
    #    super(Robot, self).draw(*args, **kwargs)
    #    # TODO: add text to base_link


class Camera(object):
    def __init__(self, body, camera_frame, camera_matrix, max_depth=2., name=None, draw_frame=None, **kwargs):
        self.body = body
        self.camera_link = link_from_name(self.body, camera_frame) # optical_frame
        self.camera_matrix = camera_matrix
        self.max_depth = max_depth
        self.name = camera_frame if name is None else name
        self.draw_link = link_from_name(self.body, draw_frame if draw_frame is None else draw_frame)
        self.kwargs = dict(kwargs)
        #self.__dict__.update(**kwargs)
        # self.handles = []
        self.handles = self.draw()
        self.get_boundaries()

    def get_pose(self):
        pose = get_link_pose(self.body, self.camera_link)
        pose = multiply(pose, Pose(point=Point(z=0.05)))  ## so that PR2's eyeball won't get in the way
        return pose

    def get_image(self, segment=True, segment_links=False, **kwargs):
        # TODO: apply maximum depth
        #image = get_image(self.get_pose(), target_pos=[0, 0, 1])
        return get_image_at_pose(self.get_pose(), self.camera_matrix,
                                 tiny=False, segment=segment, segment_links=segment_links, **kwargs)

    def draw(self):
        handles = []
        robot = self.body
        eyes_from_camera = multiply(Pose(euler=Euler(yaw=PI / 2)), Pose(euler=Euler(roll=PI / 2)))
        handles.extend(draw_viewcone(eyes_from_camera, depth=self.max_depth, camera_matrix=self.camera_matrix,
                                     parent=self.body, parent_link=self.draw_link))
        # handles.extend(draw_pose(self.get_pose(), length=1))  ## draw frame of camera
        # handles.extend(draw_pose(unit_pose(), length=1, parent=robot, parent_link=robot.base_link))  ## draw robot base frame
        return handles

    def get_boundaries(self):
        """ return the normal vectors of four faces of the viewcone """
        normals = []
        cone_base = get_viewcone_base(depth=self.max_depth, camera_matrix=self.camera_matrix)
        self.cone_base = cone_base
        pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
        for A, B in pairs:
            A = cone_base[A]
            B = cone_base[B]
            C = np.asarray([0, 0, 0])
            dir = np.cross((B - A), (C - A))
            N = dir / np.linalg.norm(dir)
            normals.append(N)
        self.boundaries = normals

    def point_in_camera_frame(self, point):
        p_world_point = Pose(point=point)
        X_world_eye = self.get_pose()
        p_eye_point = multiply(invert(X_world_eye), p_world_point)
        return p_eye_point[0]

    def point_in_view(self, point_in_world):
        p = self.point_in_camera_frame(point_in_world)
        outside = False
        for normal in self.boundaries:
            if np.dot(normal, p) < 0:
                outside = True
        in_view = not outside
        # from bullet.utils import nice
        # print(f'  point in world {point_in_world}, in camera {nice(p)}, in view? {in_view}')
        return in_view

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.name)

#######################################################

def add_body_label(body, name, text='', offset=[0,0,0], **kwargs):
    with PoseSaver(body):
        if get_color(body) == GREEN:
            set_pose(body, unit_pose())
        lower, upper = get_aabb(body)
    position = ((lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2, upper[2])
    position = [position[i]+offset[i] for i in range(len(position))]
    return add_text(name+text, position=position, parent=body, **kwargs)  # removeUserDebugItem
