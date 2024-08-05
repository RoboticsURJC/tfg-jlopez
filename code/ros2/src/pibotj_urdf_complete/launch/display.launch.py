# Sirve para configurar apropiadamente rviz cuando lanzamos nuestro paquete
import launch 
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    #fijamos el path a nuestro paquete
    pkgPath = launch_ros.substitutions.FindPackageShare(package='pibotj_urdf_complete').find('pibotj_urdf_complete')
    # fijamos el path a nuestro urdf
    urdfModelPath=os.path.join(pkgPath, 'urdf/model.urdf')
    # fijamos el path a la configuración de rviz (opcional)
    rvizConfigPath=os.path.join(pkgPath, 'config/config.rviz')

    # Traza para comprobar
    print(urdfModelPath)
    # SE usa para leer el path del modelo urdf y se va a almacenar el robot_desc
    with open(urdfModelPath, 'r') as infp: 
        robot_desc = infp.read()

    # los parámetros son la descripción del robot
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
    
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdfModelPath],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # rviz buscará el fichero de configuración en ese path
        arguments=['-d',rvizConfigPath]
    )

    
    return launch.LaunchDescription([
        # por defecto empezará gui por el joint state publisher
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True', 
                                             description='This is a flag for joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])


    
    

    
    

