cmake_minimum_required(VERSION 3.8)
project(rqt_joint_trajectory_controller)

# Built with ament_cmake instead of upstream's ament_python (setup.py) so that
# the plugin runs with the same Python environment as the rest of the workspace.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS ${PROJECT_NAME}/rqt_joint_trajectory_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rqt_joint_trajectory_controller
)

file(GLOB RESOURCE_FILES resource/*.*)
install(FILES ${RESOURCE_FILES} DESTINATION share/${PROJECT_NAME}/resource)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_joint_limits_urdf test/test_joint_limits_urdf.py)
  ament_add_pytest_test(test_command_trajectory test/test_command_trajectory.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_run_launch.py)
endif()

ament_package()
