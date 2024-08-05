import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    # Fijamos el path a nuestro paquete
    pkgPath = launch_ros.substitutions.FindPackageShare(package='pibotj_urdf_complete').find('pibotj_urdf_complete')
    # Fijamos el path a nuestro urdf
    urdfModelPath = os.path.join(pkgPath, 'urdf/model.urdf')
    # Fijamos el path a la configuración de rviz (opcional)
    rvizConfigPath = os.path.join(pkgPath, 'config/config.rviz')

    # Traza para comprobar
    print(urdfModelPath)

    # SE usa para leer el path del modelo urdf y se va a almacenar el robot_desc
    with open(urdfModelPath, 'r') as infp: 
        robot_desc = infp.read()

    # Los parámetros son la descripción del robot
    params = {'robot_description': robot_desc}

    # Defino los nodos 

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments=[urdfModelPath]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        arguments=[urdfModelPath]
    )
    
    #joint_state_publisher_gui_node = launch_ros.actions.Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui',
    #    arguments=[urdfModelPath],
    #    condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    #)

    gazebo_server_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='/usr/bin/gzserver',
        name='gazebo',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so'],
        #additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkgPath, 'meshes')}
    )
    gazebo_client_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='/usr/bin/gzclient',
        name='gazebo',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so'],
        additional_env={'GAZEBO_MODEL_PATH': os.path.join(pkgPath, 'meshes')}
    )


    spawn_robot_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-entity', 'pibotj_robot']
    )

   # rviz_node = launch_ros.actions.Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    name='rviz2',
    #    output='screen',
        # RViz buscará el fichero de configuración en ese path
    #    arguments=['-d', rvizConfigPath]
    #)
    
    return launch.LaunchDescription([
        # Por defecto empezará gui por el joint state publisher
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True', 
                                             description='This is a flag for joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        gazebo_server_node,
        gazebo_client_node,
        spawn_robot_node,
        #rviz_node
    ])
