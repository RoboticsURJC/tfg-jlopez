
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    package_name = "pibotj_description"
    robot_model_urdf = "the_simplest_diffbot.xacro"

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    package_path = os.path.join( get_package_share_directory(package_name) )

    xacro_file = os.path.join( package_path, 'urdf', robot_model_urdf )

    doc = xacro.parse( open(xacro_file) )
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py' ])
    )

    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", 'robot', ])


    print("STARTING ALL NODES ...")

    return LaunchDescription([
        simu_time,
        gazebo_node,
        robot_state_publisher_node,
        robot_spawner
    ])
