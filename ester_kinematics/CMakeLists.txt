cmake_minimum_required(VERSION 3.0.2)
project(ester_kinematics)

add_compile_options(-std=c++17 -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    roslib
    ester_common
    sensor_msgs
    visualization_msgs
    eigen_conversions
    urdf
)

find_package(DART 6.11 REQUIRED COMPONENTS
    dart
    utils
#    utils-urdf
    gui-osg
    CONFIG)
find_package(Eigen3 REQUIRED)

set(ESTER_KINEMATICS_LIBRARIES
    dart
    dart-utils
    dart-utils-urdf
)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        rospy
        roscpp
        roslib
        ester_common
        sensor_msgs
        visualization_msgs
        eigen_conversions
        urdf
    LIBRARIES ester_kinematics
    DEPENDS dart Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(ester_kinematics_ikfast_front_left SHARED
    src/ester_kinematics/ikfast/ikfast_front_left_translation.cpp
)
add_library(ester_kinematics_ikfast_front_right SHARED
    src/ester_kinematics/ikfast/ikfast_front_right_translation.cpp
)
add_library(ester_kinematics_ikfast_rear_left SHARED
    src/ester_kinematics/ikfast/ikfast_rear_left_translation.cpp
)
add_library(ester_kinematics_ikfast_rear_right SHARED
    src/ester_kinematics/ikfast/ikfast_rear_right_translation.cpp
)

add_compile_options(-DESTER_KINEMATICS_LIBRARY_PATH="$<TARGET_FILE_DIR:ester_kinematics_ikfast_front_left>")

add_library(ester_kinematics
    src/ester_kinematics/spine_kinematics.cpp
    src/ester_kinematics/leg_dynamics.cpp
    src/ester_kinematics/leg_controller.cpp
    src/ester_kinematics/robot_model.cpp
    src/ester_kinematics/robot_joint_controller.cpp
    src/ester_kinematics/force_visualiser.cpp
    src/ester_kinematics/inertia_calculator.cpp
)

add_dependencies(ester_kinematics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
)

add_executable(test_dynamics
    src/test_dynamics.cpp
)

add_dependencies(test_dynamics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_dynamics
    ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
    dart-gui-osg
)

add_executable(force_applied_urdf_minimum_test
    src/force_applied_urdf_minimum_test.cpp
)

target_link_libraries(force_applied_urdf_minimum_test
    ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
    dart-gui-osg
)


add_executable(full_robot_balance_test
    src/full_robot_balance_test.cpp
)

target_link_libraries(full_robot_balance_test
    ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
    dart-gui-osg
)

add_executable(read_robot_urdf_inertia
    src/read_robot_urdf_inertia.cpp
)

add_dependencies(read_robot_urdf_inertia
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(read_robot_urdf_inertia
    ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
)

catkin_add_gtest(kinematics_test
    tests/test_robot_model.cpp
    tests/test_dynamics.cpp
    tests/test_leg_controller.cpp
    tests/test_spine_kinematics.cpp
    tests/test_robot_joint_controller.cpp
    tests/test_inertia_calculations.cpp
    tests/run_all_tests.cpp
)

target_link_libraries(kinematics_test
    ester_kinematics
    ${catkin_LIBRARIES}
    ${ESTER_KINEMATICS_LIBRARIES}
)

install(TARGETS ester_kinematics
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/ester_kinematics
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
