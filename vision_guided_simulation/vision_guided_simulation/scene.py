"""Generate the scene and extend the installed Panda description for Gazebo."""

from pathlib import Path
import xml.etree.ElementTree as ET
import math

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def material(color):
    return f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'


def bottle_visuals():
    model = Path(get_package_share_directory('vision_guided_simulation')) / 'models/water_bottle/bottle.dae'
    if not model.is_file():
        raise FileNotFoundError(f'Bottle model missing: {model}. Rebuild the simulation package.')
    return f'''<visual name="bottle_mesh"><geometry><mesh>
        <uri>file://{model}</uri></mesh></geometry></visual>'''


def create_world(scene):
    b, c, t = (scene[k] for k in ('bottle', 'camera', 'table'))
    overview_yaw = math.atan2(1.25, -1.10)
    overview_pitch = math.atan2(.70, math.hypot(1.25, 1.10))
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="vision_lab">
  <physics name="physics" type="ignored"><max_step_size>0.01</max_step_size>
    <real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre</render_engine></plugin>
  <scene><ambient>0.65 0.65 0.65 1</ambient><background>0.82 0.86 0.90 1</background><shadows>false</shadows></scene>
  <light name="sun" type="directional"><pose>0 0 4 0 0 0</pose>
    <diffuse>.9 .9 .9 1</diffuse><specular>.2 .2 .2 1</specular>
    <direction>-.4 -.2 -1</direction><cast_shadows>false</cast_shadows></light>
  <model name="worktable"><static>true</static><link name="top">
    <pose>{t['center_x']} 0 {t['center_z']} 0 0 0</pose>
    <collision name="collision"><geometry><box><size>{t['size_x']} {t['size_y']} {t['size_z']}</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>{t['size_x']} {t['size_y']} {t['size_z']}</size></box></geometry>{material('.7 .66 .57 1')}</visual>
  </link></model>
  <model name="bottle"><static>true</static>
    <pose>{b['x']} {b['y']} {b['bottom_z']} 0 0 0</pose><link name="body">
    <collision name="collision"><pose>0 0 .11 0 0 0</pose>
      <geometry><cylinder><radius>{b['radius']}</radius><length>{b['height']}</length></cylinder></geometry></collision>
    {bottle_visuals()}
  </link></model>
  <model name="camera"><static>true</static>
    <pose>{c['x']} {c['y']} {c['z']} 0 {c['pitch']} {c['yaw']}</pose>
    <link name="camera_link"><sensor name="rgbd" type="rgbd_camera">
      <always_on>true</always_on><update_rate>2</update_rate><topic>/camera</topic>
      <gz_frame_id>camera_optical_frame</gz_frame_id>
      <camera><horizontal_fov>{c['horizontal_fov']}</horizontal_fov>
        <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>3</far></clip>
        <depth_camera><clip><near>0.05</near><far>3</far></clip></depth_camera>
      </camera></sensor></link>
  </model>
  <model name="overview"><static>true</static>
    <pose>1.35 -1.25 1.05 0 {overview_pitch} {overview_yaw}</pose>
    <link name="overview_link"><sensor name="overview" type="camera">
      <always_on>true</always_on><update_rate>2</update_rate><topic>/overview/image</topic>
      <camera><horizontal_fov>1.05</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>5</far></clip>
      </camera></sensor></link>
  </model>
</world></sdf>'''


def create_robot(controller_file):
    share = Path(get_package_share_directory('moveit_resources_panda_description'))
    root = ET.fromstring(xacro.process_file(str(share / 'urdf/panda.urdf.xacro')).toxml())
    ET.SubElement(root, 'link', name='world')
    fixed = ET.SubElement(root, 'joint', name='world_to_panda', type='fixed')
    ET.SubElement(fixed, 'parent', link='world')
    ET.SubElement(fixed, 'child', link='panda_link0')
    # Explicitly anchor the base; fixed URDF joints otherwise get lumped by SDFormat.
    gazebo = ET.SubElement(root, 'gazebo', reference='world_to_panda')
    ET.SubElement(gazebo, 'preserveFixedJoint').text = 'true'
    control = ET.SubElement(root, 'ros2_control', name='PandaGazeboSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    initial = [0, -.785, 0, -2.356, 0, 1.571, .785]
    joints = [(f'panda_joint{i+1}', v) for i, v in enumerate(initial)]
    joints += [('panda_finger_joint1', .04), ('panda_finger_joint2', .04)]
    for name, value in joints:
        joint = ET.SubElement(control, 'joint', name=name)
        if name != 'panda_finger_joint2':
            ET.SubElement(joint, 'command_interface', name='position')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(value)
        ET.SubElement(joint, 'state_interface', name='velocity')
    plugin = ET.SubElement(ET.SubElement(root, 'gazebo'), 'plugin',
                           filename='libgz_ros2_control-system.so',
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = str(controller_file)
    return ET.tostring(root, encoding='unicode')


def load_scene(path):
    return yaml.safe_load(Path(path).read_text())
