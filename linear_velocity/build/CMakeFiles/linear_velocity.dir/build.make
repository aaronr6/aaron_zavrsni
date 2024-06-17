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
CMAKE_SOURCE_DIR = /home/crta/aaron_zavrsni/linear_velocity

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crta/aaron_zavrsni/linear_velocity/build

# Include any dependencies generated for this target.
include CMakeFiles/linear_velocity.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/linear_velocity.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/linear_velocity.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/linear_velocity.dir/flags.make

CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o: CMakeFiles/linear_velocity.dir/flags.make
CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o: /home/crta/aaron_zavrsni/linear_velocity/linear_velocity.cpp
CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o: CMakeFiles/linear_velocity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/linear_velocity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o -MF CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o.d -o CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o -c /home/crta/aaron_zavrsni/linear_velocity/linear_velocity.cpp

CMakeFiles/linear_velocity.dir/linear_velocity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linear_velocity.dir/linear_velocity.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/linear_velocity/linear_velocity.cpp > CMakeFiles/linear_velocity.dir/linear_velocity.cpp.i

CMakeFiles/linear_velocity.dir/linear_velocity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linear_velocity.dir/linear_velocity.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/linear_velocity/linear_velocity.cpp -o CMakeFiles/linear_velocity.dir/linear_velocity.cpp.s

CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o: CMakeFiles/linear_velocity.dir/flags.make
CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o: /home/crta/aaron_zavrsni/linear_velocity/tcp_mg.cpp
CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o: CMakeFiles/linear_velocity.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/linear_velocity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o -MF CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o.d -o CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o -c /home/crta/aaron_zavrsni/linear_velocity/tcp_mg.cpp

CMakeFiles/linear_velocity.dir/tcp_mg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linear_velocity.dir/tcp_mg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/linear_velocity/tcp_mg.cpp > CMakeFiles/linear_velocity.dir/tcp_mg.cpp.i

CMakeFiles/linear_velocity.dir/tcp_mg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linear_velocity.dir/tcp_mg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/linear_velocity/tcp_mg.cpp -o CMakeFiles/linear_velocity.dir/tcp_mg.cpp.s

# Object files for target linear_velocity
linear_velocity_OBJECTS = \
"CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o" \
"CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o"

# External object files for target linear_velocity
linear_velocity_EXTERNAL_OBJECTS =

linear_velocity: CMakeFiles/linear_velocity.dir/linear_velocity.cpp.o
linear_velocity: CMakeFiles/linear_velocity.dir/tcp_mg.cpp.o
linear_velocity: CMakeFiles/linear_velocity.dir/build.make
linear_velocity: /usr/local/lib/libfranka.so.0.8.0
linear_velocity: CMakeFiles/linear_velocity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/crta/aaron_zavrsni/linear_velocity/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable linear_velocity"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linear_velocity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/linear_velocity.dir/build: linear_velocity
.PHONY : CMakeFiles/linear_velocity.dir/build

CMakeFiles/linear_velocity.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/linear_velocity.dir/cmake_clean.cmake
.PHONY : CMakeFiles/linear_velocity.dir/clean

CMakeFiles/linear_velocity.dir/depend:
	cd /home/crta/aaron_zavrsni/linear_velocity/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crta/aaron_zavrsni/linear_velocity /home/crta/aaron_zavrsni/linear_velocity /home/crta/aaron_zavrsni/linear_velocity/build /home/crta/aaron_zavrsni/linear_velocity/build /home/crta/aaron_zavrsni/linear_velocity/build/CMakeFiles/linear_velocity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/linear_velocity.dir/depend

