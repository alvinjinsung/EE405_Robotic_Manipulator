cmake_minimum_required(VERSION 3.0.0)
project(gripper_test VERSION 0.1.0)


include_directories(include)
add_executable(gripper_test src/gripper_test.cpp src/gripper_control.cpp src/gpio_control.cpp)

