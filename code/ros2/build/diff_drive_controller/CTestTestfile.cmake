# CMake generated Testfile for 
# Source directory: /home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller
# Build directory: /home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_diff_drive_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_results/diff_drive_controller/test_diff_drive_controller.gtest.xml" "--package-name" "diff_drive_controller" "--output-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/ament_cmake_gmock/test_diff_drive_controller.txt" "--command" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_diff_drive_controller" "--gtest_output=xml:/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_results/diff_drive_controller/test_diff_drive_controller.gtest.xml")
set_tests_properties(test_diff_drive_controller PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_diff_drive_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;106;ament_add_test;/opt/ros/humble/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;52;_ament_add_gmock;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller/CMakeLists.txt;55;ament_add_gmock;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller/CMakeLists.txt;0;")
add_test(test_load_diff_drive_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_results/diff_drive_controller/test_load_diff_drive_controller.gtest.xml" "--package-name" "diff_drive_controller" "--output-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_results/diff_drive_controller/test_load_diff_drive_controller.txt" "--command" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_load_diff_drive_controller" "--ros-args" "--params-file" "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller/test/config/test_diff_drive_controller.yaml" "--" "--gtest_output=xml:/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller/test_results/diff_drive_controller/test_load_diff_drive_controller.gtest.xml")
set_tests_properties(test_load_diff_drive_controller PROPERTIES  TIMEOUT "60" WORKING_DIRECTORY "/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/build/diff_drive_controller" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/generate_parameter_library/cmake/generate_parameter_library.cmake;160;ament_add_test;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller/CMakeLists.txt;71;add_rostest_with_parameters_gmock;/home/juloau/Desktop/TFG/tfg-jlopez/code/ros2/src/ros-controls/ros2_controllers/diff_drive_controller/CMakeLists.txt;0;")
subdirs("gmock")
subdirs("gtest")
