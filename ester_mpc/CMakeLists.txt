cmake_minimum_required(VERSION 3.0.2)
project(ester_mpc)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    ester_common
    ester_kinematics
    eigen_conversions
    xmlrpcpp
)

find_package(Sophus REQUIRED)

find_package(Eigen3 REQUIRED COMPONENTS unsupported)

# From https://github.com/qpSWIFT/qpSWIFT
find_package(qpSWIFT REQUIRED)

find_package(DART 6.11 REQUIRED COMPONENTS
    dart
    utils
    gui-osg
    CONFIG
)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ester_mpc_controllers
    DEPENDS sophus Eigen3 qpSWIFT
    CATKIN_DEPENDS 
        rospy
        roscpp
        ester_common
        ester_kinematics
        eigen_conversions
        xmlrpcpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(ester_mpc_controllers
    src/ester_mpc/spine_trajectory_generator.cpp
    src/ester_mpc/trajectory_planner.cpp
    src/ester_mpc/trajectory_visualiser.cpp
    src/ester_mpc/gait_manager.cpp
    src/ester_mpc/grf_visualiser.cpp
    src/ester_mpc/mpc_manager.cpp
    src/ester_mpc/mpc_base.cpp
    src/ester_mpc/srbd_mpc.cpp
)

add_dependencies(ester_mpc_controllers
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ester_mpc_controllers
    qpSWIFT::qpSWIFT-static
    ${catkin_LIBRARIES}
)

add_library(ester_mpc_simulators
    src/ester_mpc/world_nodes/replay_world_osg.cpp
    src/ester_mpc/world_nodes/spine_trajectory_cot_world.cpp
    src/ester_mpc/world_nodes/spine_trajectory_optimise_world.cpp
)

add_dependencies(ester_mpc_simulators
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ester_mpc_simulators
    ester_mpc_controllers
    dart
    dart-utils
    dart-utils-urdf
    dart-gui-osg
    ${catkin_LIBRARIES}
)

add_executable(optimise_spine_trajectory
    src/optimise_spine_trajectory.cpp
)

target_link_libraries(optimise_spine_trajectory
    ester_mpc_controllers
    ester_mpc_simulators
)

add_executable(find_cots
    src/find_cots.cpp
)

target_link_libraries(find_cots
    ester_mpc_controllers
    ester_mpc_simulators
)

install(DIRECTORY include/ester_mpc
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)
