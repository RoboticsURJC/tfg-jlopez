# CMake generated Testfile for 
# Source directory: /home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7
# Build directory: /home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(example_7_urdf_xacro "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/example_7_urdf_xacro.xunit.xml" "--package-name" "ros2_control_demo_example_7" "--output-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/example_7_urdf_xacro.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/test/test_urdf_xacro.py" "-o" "cache_dir=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/example_7_urdf_xacro/.cache" "--junit-xml=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/example_7_urdf_xacro.xunit.xml" "--junit-prefix=ros2_control_demo_example_7")
set_tests_properties(example_7_urdf_xacro PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;103;ament_add_pytest_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;0;")
add_test(view_example_7_launch "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/view_example_7_launch.xunit.xml" "--package-name" "ros2_control_demo_example_7" "--output-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/view_example_7_launch.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/test/test_view_robot_launch.py" "-o" "cache_dir=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/view_example_7_launch/.cache" "--junit-xml=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/view_example_7_launch.xunit.xml" "--junit-prefix=ros2_control_demo_example_7")
set_tests_properties(view_example_7_launch PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;104;ament_add_pytest_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;0;")
add_test(run_example_7_launch "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/run_example_7_launch.xunit.xml" "--package-name" "ros2_control_demo_example_7" "--output-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/run_example_7_launch.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/test/test_r6bot_controller_launch.py" "-o" "cache_dir=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/ament_cmake_pytest/run_example_7_launch/.cache" "--junit-xml=/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7/test_results/ros2_control_demo_example_7/run_example_7_launch.xunit.xml" "--junit-prefix=ros2_control_demo_example_7")
set_tests_properties(run_example_7_launch PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/ros2_control_demo_example_7" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;105;ament_add_pytest_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_control_demos/example_7/CMakeLists.txt;0;")
