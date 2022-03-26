import os
import sys
from os.path import join, abspath, dirname, isdir, isfile
sys.path.append('lisdf')
from lisdf.parsing.sdf_j import load_sdf
from lisdf.components.model_urdf import URDFModel

def make_sdf_world(sdf_model):
    """ temporary fix for LISDF format """
    # sdf_model = sdf_model.replace(',', '')
    # last_line = ''
    # sdf_model_fixed = ''
    # for line in sdf_model.split('\n'):
    #     if not ('<geometry>' in last_line and '<pose>' in line) and not 'visual>' in line:
    #         sdf_model_fixed += line + '\n'
    #     last_line = line
    # sdf_model = sdf_model_fixed
    print(sdf_model)

    return f"""<?xml version="1.0" ?>
<!-- tmp sdf file generated from LISDF -->
<sdf version="1.9">
  <world name="tmp_world">

{sdf_model}

  </world>
</sdf>"""

def load_lisdf_pybullet(sdf_path):
    from pybullet_tools.utils import load_pybullet, connect, wait_if_gui, \
        disconnect, HideOutput

    tmp_path = join('assets', 'tmp')

    connect(use_gui=True, shadows=False, width=1980, height=1238)

    # with HideOutput():
        # load_pybullet(join('models', 'Basin', '102379', 'mobility.urdf'))
    # load_pybullet(sdf_path)  ## failed
    # load_pybullet(join(tmp_path, 'table#1_1.sdf'))

    world = load_sdf(sdf_path).worlds[0]
    for model in world.models:
        print(model.name)
        if isinstance(model, URDFModel):
            uri = model.uri
        else:
            uri = join(tmp_path, f'{model.name}.sdf')
            with open(uri, 'w') as f:
                f.write(make_sdf_world(model.to_sdf()))

        # with HideOutput():

        body = load_pybullet(uri)

        # if tmp_path in uri:
        #     os.remove(tmp_path)

        wait_if_gui('load next model?')

    wait_if_gui('exit pybullet?')
    disconnect()


if __name__ == "__main__":

    for sdf_test in ['kitchen_counter_test']: ## 'm0m_joint_test'
        sdf_path = join('assets', 'scenes', f'{sdf_test}.lisdf')
        world = load_lisdf_pybullet(sdf_path)
