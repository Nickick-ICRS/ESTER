#include <gtest/gtest.h>

#include <ros/ros.h>

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    return RUN_ALL_TESTS();
}
