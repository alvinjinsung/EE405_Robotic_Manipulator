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
CMAKE_SOURCE_DIR = /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build

# Include any dependencies generated for this target.
include CMakeFiles/loopLight.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/loopLight.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/loopLight.dir/flags.make

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o: CMakeFiles/loopLight.dir/flags.make
CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o: ../src/loop_light_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o -c /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/loop_light_control.cpp

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/loopLight.dir/src/loop_light_control.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/loop_light_control.cpp > CMakeFiles/loopLight.dir/src/loop_light_control.cpp.i

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/loopLight.dir/src/loop_light_control.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/loop_light_control.cpp -o CMakeFiles/loopLight.dir/src/loop_light_control.cpp.s

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.requires:

.PHONY : CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.requires

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.provides: CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.requires
	$(MAKE) -f CMakeFiles/loopLight.dir/build.make CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.provides.build
.PHONY : CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.provides

CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.provides.build: CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o


CMakeFiles/loopLight.dir/src/gpio_control.cpp.o: CMakeFiles/loopLight.dir/flags.make
CMakeFiles/loopLight.dir/src/gpio_control.cpp.o: ../src/gpio_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/loopLight.dir/src/gpio_control.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/loopLight.dir/src/gpio_control.cpp.o -c /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/gpio_control.cpp

CMakeFiles/loopLight.dir/src/gpio_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/loopLight.dir/src/gpio_control.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/gpio_control.cpp > CMakeFiles/loopLight.dir/src/gpio_control.cpp.i

CMakeFiles/loopLight.dir/src/gpio_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/loopLight.dir/src/gpio_control.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/src/gpio_control.cpp -o CMakeFiles/loopLight.dir/src/gpio_control.cpp.s

CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.requires:

.PHONY : CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.requires

CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.provides: CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.requires
	$(MAKE) -f CMakeFiles/loopLight.dir/build.make CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.provides.build
.PHONY : CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.provides

CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.provides.build: CMakeFiles/loopLight.dir/src/gpio_control.cpp.o


# Object files for target loopLight
loopLight_OBJECTS = \
"CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o" \
"CMakeFiles/loopLight.dir/src/gpio_control.cpp.o"

# External object files for target loopLight
loopLight_EXTERNAL_OBJECTS =

loopLight: CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o
loopLight: CMakeFiles/loopLight.dir/src/gpio_control.cpp.o
loopLight: CMakeFiles/loopLight.dir/build.make
loopLight: CMakeFiles/loopLight.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable loopLight"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/loopLight.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/loopLight.dir/build: loopLight

.PHONY : CMakeFiles/loopLight.dir/build

CMakeFiles/loopLight.dir/requires: CMakeFiles/loopLight.dir/src/loop_light_control.cpp.o.requires
CMakeFiles/loopLight.dir/requires: CMakeFiles/loopLight.dir/src/gpio_control.cpp.o.requires

.PHONY : CMakeFiles/loopLight.dir/requires

CMakeFiles/loopLight.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/loopLight.dir/cmake_clean.cmake
.PHONY : CMakeFiles/loopLight.dir/clean

CMakeFiles/loopLight.dir/depend:
	cd /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build /home/kaist/DesignLab/20170699/2_SwitchControl/c_LED_CppProgram/build/CMakeFiles/loopLight.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/loopLight.dir/depend

