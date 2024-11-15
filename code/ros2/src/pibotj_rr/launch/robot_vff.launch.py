from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Ejecutar rosbridge_server
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),

        # Ejecutar gps_node
        Node(
            package='pibotj_rr',
            executable='gps_node',
            name='gps_node',
            output='screen'
        ),

        # Ejecutar servidor HTTP en puerto 8001
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8001'],
            output='screen'
        ),

    
        # Ejecutar camera_tfv4_node
        Node(
            package='pibotj_rr',
            executable='camera_tfv4_node',
            name='camera_tfv4_node',
            output='screen'
        ),

        # Ejecutar camera_pinhole_vff_node
        Node(
            package='pibotj_rr',
            executable='camera_pinhole_vff_node',
            name='camera_pinhole_vff_node',
            output='screen'
        ),

        # Ejecutar camera_pinhole_web_node
        Node(
            package='pibotj_rr',
            executable='camera_pinhole_web_node',
            name='camera_pinhole_web_node',
            output='screen'
        ),

        # Ejecutar camera_dl_node
        Node(
            package='pibotj_rr',
            executable='camera_dl_node',
            name='camera_dl_node',
            output='screen'
        ),

        # Ejecutar camera_vff_node
        Node(
            package='pibotj_rr',
            executable='camera_vff_node',
            name='camera_vff_node',
            output='screen'
        ),

        # Ejecutar motors_vffv2_node
        Node(
            package='pibotj_rr',
            executable='motors_vffv2_node',
            name='motors_vffv2_node',
            output='screen'
        )

    ])
