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
include CMakeFiles/FSM.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FSM.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FSM.dir/flags.make

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o: CMakeFiles/FSM.dir/flags.make
CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o: ../include/FiniteStateMachine/fsm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o -c /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/FiniteStateMachine/fsm.cpp

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/FiniteStateMachine/fsm.cpp > CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.i

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/include/FiniteStateMachine/fsm.cpp -o CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.s

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.requires:

.PHONY : CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.requires

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.provides: CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.requires
	$(MAKE) -f CMakeFiles/FSM.dir/build.make CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.provides.build
.PHONY : CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.provides

CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.provides.build: CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o


# Object files for target FSM
FSM_OBJECTS = \
"CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o"

# External object files for target FSM
FSM_EXTERNAL_OBJECTS =

libFSM.a: CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o
libFSM.a: CMakeFiles/FSM.dir/build.make
libFSM.a: CMakeFiles/FSM.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libFSM.a"
	$(CMAKE_COMMAND) -P CMakeFiles/FSM.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FSM.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FSM.dir/build: libFSM.a

.PHONY : CMakeFiles/FSM.dir/build

CMakeFiles/FSM.dir/requires: CMakeFiles/FSM.dir/include/FiniteStateMachine/fsm.cpp.o.requires

.PHONY : CMakeFiles/FSM.dir/requires

CMakeFiles/FSM.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FSM.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FSM.dir/clean

CMakeFiles/FSM.dir/depend:
	cd /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaist/DesignLab/20170699/EE405_SystemIntegaration /home/kaist/DesignLab/20170699/EE405_SystemIntegaration /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build /home/kaist/DesignLab/20170699/EE405_SystemIntegaration/build/CMakeFiles/FSM.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FSM.dir/depend
