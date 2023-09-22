cmake_minimum_required(VERSION 3.10)
project(EE405_MotorControl LANGUAGES CXX)

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
    lib
    lib/dynamixel_sdk
)

### add_library: How to make a static library file using the following code. 
### If you want to create a shared library, change 'STATIC' to 'SHARED'. 
### However, for cross-compiling, it is not recommended to change to 'SHARED'.
add_library(EE405_robot_manipulator STATIC include/Controller/ArticulatedSystem.cpp
                                            include/FileIO/MatrixFileIO.cpp
                                            include/Common/LowPassFilter.cpp
                                            include/Common/NumDiff.cpp
                                            )

### add_executable: Make executable file using following motor_control.cpp code.
add_executable(motor_control src/motor_control.cpp )

### target_link_libraries: link between exe file and library file.
### EE405_robot_manipulator: The static library which we make. please refer to add_library().
### dxl_sbc_cpp: The dxl_x64_cpp shared library is from the Dynamixel SDK and enables motor rotation in the program.
target_link_libraries(motor_control EE405_robot_manipulator dxl_sbc_cpp  )





