cmake_minimum_required(VERSION 3.10)
project(EE405_SystemIntegration LANGUAGES CXX)

add_compile_options(-std=c++17)

### find package: pre-installed library The CMake can automatically find following installed package.
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

### include_directories: We can set include directories, The CMake can look for the header file now in this path.
include_directories (${EIGEN3_INCLUDE_DIRS}
                    include
                    include/dynamixel_sdk
                    src)

### link_directories: We can set link directories, The CMake can look for the library file now in this path in order to link library and executable file.
link_directories(
    lib/dynamixel_sdk
    lib
)

# add_library: How to make a static library file using the following code. 
# If you want to create a shared library, change 'STATIC' to 'SHARED'. 
# However, for cross-compiling, it is not recommended to change to 'SHARED'.

##for Problem1
add_library(EE405_SI_P1 STATIC include/Controller/ArticulatedSystem.cpp
                                            include/Common/Kinematics.cpp
                                            include/Common/gpio_control.cpp
                                            include/FileIO/MatrixFileIO.cpp
                                            include/Common/DirectTeaching.cpp
                                            )
add_executable(modified_direct_teaching src/modified_direct_teaching.cpp)
target_link_libraries(modified_direct_teaching Threads::Threads EE405_SI_P1 dxl_sbc_cpp)

##for Problem2
add_library(EE405_SI_P2 STATIC include/Controller/ArticulatedSystem.cpp
                                            include/Common/Kinematics.cpp
                                            include/Common/gpio_control.cpp
                                            )

add_executable(teleoperation_udp src/teleoperation_udp.cpp)
target_link_libraries(teleoperation_udp Threads::Threads EE405_SI_P2 dxl_sbc_cpp)



#for Problem3
add_library(FSM STATIC include/FiniteStateMachine/fsm.cpp)
target_link_libraries(FSM EE405_SI_P1)

add_executable(system_integration src/system_integration.cpp)
target_link_libraries(system_integration Threads::Threads EE405_SI_P1 FSM dxl_sbc_cpp)

#for Problem4
add_library(FSM_OWN STATIC include/FiniteStateMachine/fsm_own.cpp)
target_link_libraries(FSM_OWN EE405_SI_P1)

add_executable(system_integration_own src/system_integration_own.cpp)
target_link_libraries(system_integration_own Threads::Threads EE405_SI_P1 FSM_OWN dxl_sbc_cpp)
