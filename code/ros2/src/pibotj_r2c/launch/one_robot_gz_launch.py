# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

package_name = 'pibotj_r2c'
robot_model='caster_diffbot' #the_simplest_diffbot, caster_diffbot
pose = ['0.0', '0.0', '0.0', '0.0'] #Initial robot pose in gz: x,y,z,th
gz_robot_name = robot_model #robot name used in Gazebo

#Launch argument
world_file = 'empty.world' #empty.world

def generate_launch_description():

    pkg_robot_simulation = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_robot_simulation, 'worlds', world_file), ''],
        description='Custom SDF world file')    

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_description_path =  os.path.join( pkg_robot_simulation, "urdf", robot_model + '.xacro', )
    
    robot_description = {"robot_description": xacro.process_file(robot_description_path).toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", gz_robot_name, 
            "-x", pose[0], "-y", pose[1], "-z", pose[2], "-Y", pose[3] ])



    # NUEVOS NODOS AÑADIDOS


    # Include spawners for the controllers
    #diff_drive_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["diff_cont"],
    #    output="screen"
    #)

    #pan_tilt_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["joint_broad"],
    #    output="screen"
    #)

    # FIN DE LOS NUEVOS NODOS AÑADIDOS

    print("STARTING ALL NODES ...")

    return LaunchDescription([
        world_arg,
        simu_time,
        gazebo_node,
        robot_state_publisher_node,
        robot_spawner,
        #controller_manager,
        #diff_drive_spawner,
        #pan_tilt_spawner
    ])
