cmake_minimum_required(VERSION 3.0.2)
project(ester_common)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS)

find_package(Eigen3 REQUIRED)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ester_common
#    CATKIN_DEPENDS
    DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(ester_common
    src/ester_common/logging/file_logger.cpp
)

add_dependencies(ester_common
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ester_common
    ${catkin_LIBRARIES}
)

install(TARGETS ester_common
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/ester_common/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)
