"""Panda, RGB-D camera, MoveIt and diagnostics in one Gazebo world."""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from vision_guided_simulation.scene import create_robot, create_world, load_scene


def setup(context):
    share = Path(get_package_share_directory('vision_guided_simulation'))
    scene_path = LaunchConfiguration('scene').perform(context)
    scene = load_scene(scene_path)
    run_dir = Path(tempfile.mkdtemp(prefix='vision-guided-'))
    world = run_dir / 'world.sdf'
    world.write_text(create_world(scene))
    robot = create_robot(share / 'config/controllers.yaml')
    (run_dir / 'panda.urdf').write_text(robot)
    config = (MoveItConfigsBuilder('panda', package_name='moveit_resources_panda_moveit_config')
              .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
              .to_moveit_configs())
    config.robot_description = {'robot_description': robot}
    parameters = config.to_dict()
    parameters.update({'use_sim_time': True,
                       'publish_robot_description_semantic': True,
                       'trajectory_execution.execution_duration_monitoring': False})
    c = scene['camera']
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gazebo_args = ['gz', 'sim', '-r', '-v', '3', '--render-engine', 'ogre', str(world)]
    if not gui:
        gazebo_args += ['-s']
    actions = [
        ExecuteProcess(cmd=gazebo_args, output='log'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'vision_lab', '-name', 'panda', '-file', str(run_dir / 'panda.urdf')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/overview/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ], parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--x', str(c['x']), '--y', str(c['y']), '--z', str(c['z']),
            '--roll', '0', '--pitch', str(c['pitch']), '--yaw', str(c['yaw']),
            '--frame-id', 'panda_link0', '--child-frame-id', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '-1.5707963267948966',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[parameters]),
        Node(package='vision_guided_simulation', executable='scene_monitor',
             parameters=[{'use_sim_time': True, 'scene_path': scene_path}]),
        Node(package='robot_manipulation', executable='pose_controller',
             parameters=[{'use_sim_time': True, 'approach_offset': .10, 'lift_height': .15}]),
        Node(package='vision_guided_simulation', executable='demo_recorder',
             parameters=[{'use_sim_time': True}]),
    ]
    actions.append(Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'panda_arm_controller',
                                   'panda_hand_controller', '--controller-manager-timeout', '180',
                                   '--service-call-timeout', '120', '--switch-timeout', '120']))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        panda_share = Path(get_package_share_directory('moveit_resources_panda_moveit_config'))
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(panda_share / 'launch/moveit.rviz')],
                            parameters=[parameters]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('vision_guided_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('scene', default_value=str(share / 'config/scene.yaml')),
        OpaqueFunction(function=setup),
    ])
