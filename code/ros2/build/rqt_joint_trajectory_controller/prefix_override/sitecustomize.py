import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/install/rqt_joint_trajectory_controller'
