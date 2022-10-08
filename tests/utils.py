import os.path

import numpy as np
import trimesh
import untangle
from trimesh import transformations

def test_is_robot(name, robots=["pr2"]):
    if robots is None:
        return True
    return any(name.startswith(prefix) for prefix in robots)


def load_lisdf(lisdf_dir, scene_scale=1., robots=False, skip=[], verbose=True):
    # TODO: apply within load_lisdf_synthesizer
    lisdf_path = os.path.join(lisdf_dir, 'scene.lisdf')
    world_xml = untangle.parse(lisdf_path).sdf.world

    model_states = {}
    if len(world_xml.state) > 0:
        model_states = world_xml.state.model
        model_states = {s['name']: {j['name']: eval(j.angle.cdata) for j in s.joint} for s in model_states}

    for obj_xml in world_xml.include:
        name = obj_xml._attributes["name"]
        if (name in skip) or (not robots and test_is_robot(name)): # TODO: generalize
            continue

        path = os.path.abspath(os.path.join(os.path.dirname(lisdf_path), obj_xml.uri.cdata))
        if hasattr(obj_xml, "scale"):
            scale = scene_scale * float(obj_xml.scale.cdata)
        else:
            scale = scene_scale
        if hasattr(obj_xml, "static"):
            is_fixed = obj_xml.static.cdata == "true"
        else:
            is_fixed = False
        if verbose:
            print(f"Name: {name} | Fixed: {is_fixed} | Scale: {scale:.3f} | Path: {path}")

        pose_data = np.array(obj_xml.pose.cdata.split(" ")).astype(float)
        position = scene_scale * pose_data[:3]
        euler = pose_data[3:]
        pose = transformations.translation_matrix(position) @ \
               transformations.euler_matrix(*euler)

        positions = model_states[name] if name in model_states else {}

        yield name, path, scale, is_fixed, pose, positions

##################################################

def look_at_scene(scene, pose=None, use_aabb=False):
    if pose is None:
        pose = transformations.euler_matrix(np.pi/4, 0, np.pi/2)
    scene.camera_transform = scene.camera.look_at(
        points=scene.bounds_corners if use_aabb else scene.convex_hull.vertices, # TODO: rename to visible
        rotation=pose,
        distance=None,
        center=None,
        pad=None,
    )
    return scene.camera_transform

def load_lisdf_synthesizer(lisdf_dir, scene_scale=1., robots=False, surfaces=False, texture=True, display=True):
    import scene_synthesizer as synth
    lisdf_path = os.path.join(lisdf_dir, 'scene.lisdf')

    scene = synth.Scene()
    world_xml = untangle.parse(lisdf_path).sdf.world
    for obj_xml in world_xml.include:
        name = obj_xml._attributes["name"]
        if not robots and any(name.startswith(prefix) for prefix in ["pr2"]):
           continue

        # obj_id = name.replace("#", "") # NOTE(caelan): required for USD export
        obj_id = name
        asset_path = os.path.abspath(os.path.join(os.path.dirname(lisdf_path), obj_xml.uri.cdata))

        if hasattr(obj_xml, "scale"):
            scale = scene_scale * float(obj_xml.scale.cdata)
        else:
            scale = scene_scale
        if hasattr(obj_xml, "static"):
            is_fixed = obj_xml.static.cdata == "true"
        else:
            is_fixed = False
        print(f"Name: {obj_id} | Fixed: {is_fixed} | Scale: {scale:.3f} | Path: {asset_path}")
        asset = synth.Asset(
            fname=asset_path,
            origin=("center", "center", "center"),
            scale=scale,
        )

        pose_data = np.array(obj_xml.pose.cdata.split(" ")).astype(float)
        position = scene_scale * pose_data[:3]
        euler = pose_data[3:]
        pose = transformations.translation_matrix(position) @ transformations.euler_matrix(*euler)

        scene.add_object(
            obj_id=obj_id,
            obj_model=asset,
            transform=pose,
            joint_type="fixed" if is_fixed else "floating",
            use_collision_geometry=None,
            keep_on_ground=False,
        )
        if not texture:
            scene.remove_visuals()

    if not texture:
        scene.colorize()
    if surfaces:
        scene.label_support(label="support", layer="visual")
    look_at_scene(scene.scene)
    if display:
        scene.show()

    return scene

##################################################

def load_urdf_links(world, model, verbose=False):
    from pybullet_planning.pybullet_tools.utils import Pose, Euler, PI, pose_from_tform, multiply
    for link_data in model.robot.links:
        for visual_data in link_data.visuals:  # visuals | collisions
            mesh_data = visual_data.geometry.mesh
            if mesh_data is None:
                continue
            mesh_filename = mesh_data.filename
            mesh_path = os.path.abspath(model._filename_handler(mesh_filename))
            # if not os.path.exists(mesh_path):
            #    continue
            if verbose:
                print(link_data.name, mesh_path)
            pose = None # TODO: obtain the link poses
            world.import_asset(mesh_path, pose=Pose(euler=Euler(roll=PI / 2)))
    raise NotImplementedError()


def load_lisdf_nvisii(lisdf_dir, **kwargs):
    from srl_stream.visii_world import VisiiPyBulletWorld, VisiiWorld, visii_from_pybullet, create_link
    #from srl_stream.visii_render import VisiiRenderer
    from srl_stream.trimesh_world import TrimeshWorld
    from pybullet_planning.pybullet_tools.utils import Pose, Euler, PI, pose_from_tform, multiply, invert
    import yourdfpy

    # visii_world = visii_from_pybullet(
    #     # VisiiPyBulletWorld
    #     bodies=None, floor=True,
    #     width=640, height=480,
    #     spp=2**5, fov=[60., 45.], gamma=2.2,
    #     viewer=True, inertial=True,
    #     use_yourdfpy=True,
    #     load_texture=False,
    #     randomize_materials=False,
    #     # VisiiWorld
    #     lazy=False,
    #     denoise=True,
    # )

    visii_world = VisiiWorld(
        # fov=math.radians(fov[1]),
        # width=width, height=height,
        # spp=spp,
        # gamma=gamma,
        viewer=True,
        lazy=False,
        # **kwargs
    )
    world_scene = trimesh.Scene()
    for name, path, scale, _, pose in load_lisdf(lisdf_dir, robots=False, **kwargs):
        print(name, path, scale) # pose
        if path.endswith('.urdf'):
            # assert scale == 1
            model = yourdfpy.URDF.load(
                path,
                build_scene_graph=True,
                build_collision_scene_graph=True,
                load_meshes=True,
                load_collision_meshes=True,
                # filename_handler=None,
                # mesh_dir='',
                force_mesh=False,
                force_collision_mesh=False,
            )
            # RuntimeError: Error: "/home/caelan/Programs/interns/yang/kitchen-worlds/assets/models/KitchenCounter/26608/mobility.urdf
            # "The specified model file extension ".urdf" is currently unsupported.
            # visii_world.import_asset(path)

            # model.show()
            # load_urdf_links(visii_world, model, verbose=True)
            scene = model.scene
        else:
            scene = trimesh.load(path, force='scene')
        scene = scene.copy()
        # scene = scene.scaled(scale)
        scene.apply_scale(scale)
        # scene.apply_transform(pose)

        #print(scene.graph.nodes)
        #print(scene.graph.nodes_geometry)
        #for frame in scene.graph.nodes:
        for frame, mesh in scene.geometry.items():
            mesh_path = mesh.metadata['file_path'] # ['processed', 'file_path', 'file_name', 'file_element']
            # frame_pose, _ = scene.graph[frame]
            frame_pose, _ = scene.graph.get(frame_to=frame, frame_from=None)
            visii_world.import_asset(mesh_path, pose=multiply(
                #Pose(euler=Euler(roll=PI / 2)),
                invert(pose_from_tform(pose)),
                pose_from_tform(frame_pose),
            ))
            #entity = create_link(visii_world, mesh, name=frame, pose=frame_pose) # color=color,

        world_scene = scene
        # world_scene.add_geometry(scene) # No path between nodes visii_world and drawer_front-70!
        # world_scene = world_scene + scene
        camera_transform = look_at_scene(world_scene)
        visii_world.set_pose(visii_world.camera, camera_transform)
        world_scene.show()
    return visii_world


if __name__ == "__main__":
    p = os.path.abspath('/home/zhutiany/Documents/mamao-data/one_fridge_pick_pr2/1000')
    load_lisdf_synthesizer(p)