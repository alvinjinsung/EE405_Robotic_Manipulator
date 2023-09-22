cmake_minimum_required(VERSION 3.0)
#set(CMAKE_TOOLCHAIN_FILE "toolchain.arm.cmake")
project(lab4 LANGUAGES CXX)

add_compile_options(-std=c++17)
# set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
enable_language(C)
enable_language(CXX)

# find_package(Eigen3 REQUIRED)
# find_package(Curses REQUIRED)s
# find_package(glfw3 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(Threads REQUIRED)

# include_directories (${EIGEN3_INCLUDE_DIRS}
#                     include
#                     include/dynamixel_sdk
#                     src)
  
# link_directories(
#     lib
#     lib/dynamixel_sdk
# )
                    
add_executable(capture src/capture.c)
#add_executable(client src/client.c)
#add_executable(server src/server.c)
#add_executable(talker src/talker.c)
#add_executable(listener src/listener.c)



# create_executable(PoseMatlabTest) 


