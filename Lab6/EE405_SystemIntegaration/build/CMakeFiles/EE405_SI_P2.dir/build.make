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
CMAKE_SOURCE_DIR = /home/kaist/DesignLab/20170699/EE405_SystemIntegaration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build

# Include any dependencies generated for this target.
include CMakeFiles/EE405_SI_P2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/EE405_SI_P2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/EE405_SI_P2.dir/flags.make

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o: CMakeFiles/EE405_SI_P2.dir/flags.make
CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o: ../include/Controller/ArticulatedSystem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o -c /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Controller/ArticulatedSystem.cpp

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Controller/ArticulatedSystem.cpp > CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.i

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Controller/ArticulatedSystem.cpp -o CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.s

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.requires:

.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.requires

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.provides: CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_SI_P2.dir/build.make CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.provides

CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.provides.build: CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o


CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o: CMakeFiles/EE405_SI_P2.dir/flags.make
CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o: ../include/Common/Kinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o -c /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/Kinematics.cpp

CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/Kinematics.cpp > CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.i

CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/Kinematics.cpp -o CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.s

CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.requires:

.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.requires

CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.provides: CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_SI_P2.dir/build.make CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.provides

CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.provides.build: CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o


CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o: CMakeFiles/EE405_SI_P2.dir/flags.make
CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o: ../include/Common/gpio_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o -c /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/gpio_control.cpp

CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/gpio_control.cpp > CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.i

CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/Common/gpio_control.cpp -o CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.s

CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.requires:

.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.requires

CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.provides: CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.requires
	$(MAKE) -f CMakeFiles/EE405_SI_P2.dir/build.make CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.provides.build
.PHONY : CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.provides

CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.provides.build: CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o


# Object files for target EE405_SI_P2
EE405_SI_P2_OBJECTS = \
"CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o" \
"CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o" \
"CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o"

# External object files for target EE405_SI_P2
EE405_SI_P2_EXTERNAL_OBJECTS =

libEE405_SI_P2.a: CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o
libEE405_SI_P2.a: CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o
libEE405_SI_P2.a: CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o
libEE405_SI_P2.a: CMakeFiles/EE405_SI_P2.dir/build.make
libEE405_SI_P2.a: CMakeFiles/EE405_SI_P2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libEE405_SI_P2.a"
	$(CMAKE_COMMAND) -P CMakeFiles/EE405_SI_P2.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/EE405_SI_P2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/EE405_SI_P2.dir/build: libEE405_SI_P2.a

.PHONY : CMakeFiles/EE405_SI_P2.dir/build

CMakeFiles/EE405_SI_P2.dir/requires: CMakeFiles/EE405_SI_P2.dir/include/Controller/ArticulatedSystem.cpp.o.requires
CMakeFiles/EE405_SI_P2.dir/requires: CMakeFiles/EE405_SI_P2.dir/include/Common/Kinematics.cpp.o.requires
CMakeFiles/EE405_SI_P2.dir/requires: CMakeFiles/EE405_SI_P2.dir/include/Common/gpio_control.cpp.o.requires

.PHONY : CMakeFiles/EE405_SI_P2.dir/requires

CMakeFiles/EE405_SI_P2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/EE405_SI_P2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/EE405_SI_P2.dir/clean

CMakeFiles/EE405_SI_P2.dir/depend:
	cd /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaist/DesignLab/20170699/EE405_SystemIntegaration /home/kaist/DesignLab/20170699/EE405_SystemIntegaration /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles/EE405_SI_P2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/EE405_SI_P2.dir/depend
