# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/crta/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/crta/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/crta/aaron_zavrsni/graph_velocities

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crta/aaron_zavrsni/graph_velocities/build

# Include any dependencies generated for this target.
include CMakeFiles/write_velocity_states.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/write_velocity_states.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/write_velocity_states.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/write_velocity_states.dir/flags.make

CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o: CMakeFiles/write_velocity_states.dir/flags.make
CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o: /home/crta/aaron_zavrsni/graph_velocities/write_velocity_states.cpp
CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o: CMakeFiles/write_velocity_states.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/graph_velocities/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o -MF CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o.d -o CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o -c /home/crta/aaron_zavrsni/graph_velocities/write_velocity_states.cpp

CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/graph_velocities/write_velocity_states.cpp > CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.i

CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/graph_velocities/write_velocity_states.cpp -o CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.s

CMakeFiles/write_velocity_states.dir/examples_common.cpp.o: CMakeFiles/write_velocity_states.dir/flags.make
CMakeFiles/write_velocity_states.dir/examples_common.cpp.o: /home/crta/aaron_zavrsni/graph_velocities/examples_common.cpp
CMakeFiles/write_velocity_states.dir/examples_common.cpp.o: CMakeFiles/write_velocity_states.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/graph_velocities/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/write_velocity_states.dir/examples_common.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/write_velocity_states.dir/examples_common.cpp.o -MF CMakeFiles/write_velocity_states.dir/examples_common.cpp.o.d -o CMakeFiles/write_velocity_states.dir/examples_common.cpp.o -c /home/crta/aaron_zavrsni/graph_velocities/examples_common.cpp

CMakeFiles/write_velocity_states.dir/examples_common.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/write_velocity_states.dir/examples_common.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/graph_velocities/examples_common.cpp > CMakeFiles/write_velocity_states.dir/examples_common.cpp.i

CMakeFiles/write_velocity_states.dir/examples_common.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/write_velocity_states.dir/examples_common.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/graph_velocities/examples_common.cpp -o CMakeFiles/write_velocity_states.dir/examples_common.cpp.s

# Object files for target write_velocity_states
write_velocity_states_OBJECTS = \
"CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o" \
"CMakeFiles/write_velocity_states.dir/examples_common.cpp.o"

# External object files for target write_velocity_states
write_velocity_states_EXTERNAL_OBJECTS =

write_velocity_states: CMakeFiles/write_velocity_states.dir/write_velocity_states.cpp.o
write_velocity_states: CMakeFiles/write_velocity_states.dir/examples_common.cpp.o
write_velocity_states: CMakeFiles/write_velocity_states.dir/build.make
write_velocity_states: /usr/local/lib/libfranka.so.0.8.0
write_velocity_states: CMakeFiles/write_velocity_states.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/crta/aaron_zavrsni/graph_velocities/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable write_velocity_states"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/write_velocity_states.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/write_velocity_states.dir/build: write_velocity_states
.PHONY : CMakeFiles/write_velocity_states.dir/build

CMakeFiles/write_velocity_states.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/write_velocity_states.dir/cmake_clean.cmake
.PHONY : CMakeFiles/write_velocity_states.dir/clean

CMakeFiles/write_velocity_states.dir/depend:
	cd /home/crta/aaron_zavrsni/graph_velocities/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crta/aaron_zavrsni/graph_velocities /home/crta/aaron_zavrsni/graph_velocities /home/crta/aaron_zavrsni/graph_velocities/build /home/crta/aaron_zavrsni/graph_velocities/build /home/crta/aaron_zavrsni/graph_velocities/build/CMakeFiles/write_velocity_states.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/write_velocity_states.dir/depend

