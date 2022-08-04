import os.path

import numpy as np
import scene_synthesizer as synth
import untangle
from trimesh import transformations

def load_lisdf_synthesizer(lisdf_dir, scene_scale=1., robots=False, surfaces=False, texture=True, display=True):
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
    scene.scene.camera_transform = scene.scene.camera.look_at(
        scene.scene.convex_hull.vertices,
        rotation=transformations.euler_matrix(np.pi/4, 0, np.pi/2),
    )
    if display:
        scene.show()

    return scene
