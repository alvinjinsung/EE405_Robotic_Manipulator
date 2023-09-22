cmake_minimum_required(VERSION 3.0.0)
project(loopLight VERSION 0.1.0)


include_directories(include)
add_executable(loopLight src/loop_light_control.cpp src/gpio_control.cpp )

