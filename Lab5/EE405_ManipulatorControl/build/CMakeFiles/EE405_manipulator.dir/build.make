# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaist/DesignLab/20170699/EE405_ManipulatorControl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build

# Include any dependencies generated for this target.
include CMakeFiles/EE405_manipulator.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/EE405_manipulator.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/EE405_manipulator.dir/flags.make

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o: CMakeFiles/EE405_manipulator.dir/flags.make
CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o: ../include/Controller/ArticulatedSystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o -c /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Controller/ArticulatedSystem.cpp

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Controller/ArticulatedSystem.cpp > CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.i

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Controller/ArticulatedSystem.cpp -o CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.s

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.requires:

.PHONY : CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.requires

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.provides: CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_manipulator.dir/build.make CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.provides

CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.provides.build: CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o


CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o: CMakeFiles/EE405_manipulator.dir/flags.make
CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o: ../include/FileIO/MatrixFileIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o -c /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/FileIO/MatrixFileIO.cpp

CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/FileIO/MatrixFileIO.cpp > CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.i

CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/FileIO/MatrixFileIO.cpp -o CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.s

CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.requires:

.PHONY : CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.requires

CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.provides: CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_manipulator.dir/build.make CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.provides

CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.provides.build: CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o


CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o: CMakeFiles/EE405_manipulator.dir/flags.make
CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o: ../include/Common/DirectTeaching.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o -c /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/DirectTeaching.cpp

CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/DirectTeaching.cpp > CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.i

CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/DirectTeaching.cpp -o CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.s

CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.requires:

.PHONY : CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.requires

CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.provides: CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_manipulator.dir/build.make CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.provides

CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.provides.build: CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o


CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o: CMakeFiles/EE405_manipulator.dir/flags.make
CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o: ../include/Common/Kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o -c /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/Kinematics.cpp

CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/Kinematics.cpp > CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.i

CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/Common/Kinematics.cpp -o CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.s

CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.requires:

.PHONY : CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.requires

CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.provides: CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_manipulator.dir/build.make CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.provides

CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.provides.build: CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o


CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o: CMakeFiles/EE405_manipulator.dir/flags.make
CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o: ../include/KeyInput/getche.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o -c /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/KeyInput/getche.cpp

CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/KeyInput/getche.cpp > CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.i

CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/include/KeyInput/getche.cpp -o CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.s

CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.requires:

.PHONY : CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.requires

CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.provides: CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_manipulator.dir/build.make CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.provides

CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.provides.build: CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o


# Object files for target EE405_manipulator
EE405_manipulator_OBJECTS = \
"CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o" \
"CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o" \
"CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o" \
"CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o" \
"CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o"

# External object files for target EE405_manipulator
EE405_manipulator_EXTERNAL_OBJECTS =

libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/build.make
libEE405_manipulator.a: CMakeFiles/EE405_manipulator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libEE405_manipulator.a"
	$(CMAKE_COMMAND) -P CMakeFiles/EE405_manipulator.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EE405_manipulator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/EE405_manipulator.dir/build: libEE405_manipulator.a

.PHONY : CMakeFiles/EE405_manipulator.dir/build

CMakeFiles/EE405_manipulator.dir/requires: CMakeFiles/EE405_manipulator.dir/include/Controller/ArticulatedSystem.cpp.o.requires
CMakeFiles/EE405_manipulator.dir/requires: CMakeFiles/EE405_manipulator.dir/include/FileIO/MatrixFileIO.cpp.o.requires
CMakeFiles/EE405_manipulator.dir/requires: CMakeFiles/EE405_manipulator.dir/include/Common/DirectTeaching.cpp.o.requires
CMakeFiles/EE405_manipulator.dir/requires: CMakeFiles/EE405_manipulator.dir/include/Common/Kinematics.cpp.o.requires
CMakeFiles/EE405_manipulator.dir/requires: CMakeFiles/EE405_manipulator.dir/include/KeyInput/getche.cpp.o.requires

.PHONY : CMakeFiles/EE405_manipulator.dir/requires

CMakeFiles/EE405_manipulator.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/EE405_manipulator.dir/cmake_clean.cmake
.PHONY : CMakeFiles/EE405_manipulator.dir/clean

CMakeFiles/EE405_manipulator.dir/depend:
	cd /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaist/DesignLab/20170699/EE405_ManipulatorControl /home/kaist/DesignLab/20170699/EE405_ManipulatorControl /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build /home/kaist/DesignLab/20170699/EE405_ManipulatorControl/build/CMakeFiles/EE405_manipulator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/EE405_manipulator.dir/depend
