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
CMAKE_SOURCE_DIR = /home/crta/aaron_zavrsni/tcp_mg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/crta/aaron_zavrsni/tcp_mg/build

# Include any dependencies generated for this target.
include CMakeFiles/tcp_mg_tester.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/tcp_mg_tester.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tcp_mg_tester.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tcp_mg_tester.dir/flags.make

CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o: CMakeFiles/tcp_mg_tester.dir/flags.make
CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o: /home/crta/aaron_zavrsni/tcp_mg/tcp_mg_tester.cpp
CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o: CMakeFiles/tcp_mg_tester.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/tcp_mg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o -MF CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o.d -o CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o -c /home/crta/aaron_zavrsni/tcp_mg/tcp_mg_tester.cpp

CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/tcp_mg/tcp_mg_tester.cpp > CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.i

CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/tcp_mg/tcp_mg_tester.cpp -o CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.s

CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o: CMakeFiles/tcp_mg_tester.dir/flags.make
CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o: /home/crta/aaron_zavrsni/tcp_mg/tcp_mg.cpp
CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o: CMakeFiles/tcp_mg_tester.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/crta/aaron_zavrsni/tcp_mg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o -MF CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o.d -o CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o -c /home/crta/aaron_zavrsni/tcp_mg/tcp_mg.cpp

CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/crta/aaron_zavrsni/tcp_mg/tcp_mg.cpp > CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.i

CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/crta/aaron_zavrsni/tcp_mg/tcp_mg.cpp -o CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.s

# Object files for target tcp_mg_tester
tcp_mg_tester_OBJECTS = \
"CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o" \
"CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o"

# External object files for target tcp_mg_tester
tcp_mg_tester_EXTERNAL_OBJECTS =

tcp_mg_tester: CMakeFiles/tcp_mg_tester.dir/tcp_mg_tester.cpp.o
tcp_mg_tester: CMakeFiles/tcp_mg_tester.dir/tcp_mg.cpp.o
tcp_mg_tester: CMakeFiles/tcp_mg_tester.dir/build.make
tcp_mg_tester: /usr/local/lib/libfranka.so.0.8.0
tcp_mg_tester: CMakeFiles/tcp_mg_tester.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/crta/aaron_zavrsni/tcp_mg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable tcp_mg_tester"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tcp_mg_tester.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tcp_mg_tester.dir/build: tcp_mg_tester
.PHONY : CMakeFiles/tcp_mg_tester.dir/build

CMakeFiles/tcp_mg_tester.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tcp_mg_tester.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tcp_mg_tester.dir/clean

CMakeFiles/tcp_mg_tester.dir/depend:
	cd /home/crta/aaron_zavrsni/tcp_mg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/crta/aaron_zavrsni/tcp_mg /home/crta/aaron_zavrsni/tcp_mg /home/crta/aaron_zavrsni/tcp_mg/build /home/crta/aaron_zavrsni/tcp_mg/build /home/crta/aaron_zavrsni/tcp_mg/build/CMakeFiles/tcp_mg_tester.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tcp_mg_tester.dir/depend

