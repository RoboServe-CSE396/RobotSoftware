# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /home/reveha/.local/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/reveha/.local/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/reveha/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reveha/YDLidar-SDK/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/lidar_c_api_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/lidar_c_api_test.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/lidar_c_api_test.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/lidar_c_api_test.dir/flags.make

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o: examples/CMakeFiles/lidar_c_api_test.dir/flags.make
examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o: /home/reveha/YDLidar-SDK/examples/lidar_c_api_test.c
examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o: examples/CMakeFiles/lidar_c_api_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/reveha/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o"
	cd /home/reveha/YDLidar-SDK/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o -MF CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o.d -o CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o -c /home/reveha/YDLidar-SDK/examples/lidar_c_api_test.c

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i"
	cd /home/reveha/YDLidar-SDK/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/reveha/YDLidar-SDK/examples/lidar_c_api_test.c > CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.i

examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s"
	cd /home/reveha/YDLidar-SDK/build/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/reveha/YDLidar-SDK/examples/lidar_c_api_test.c -o CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.s

# Object files for target lidar_c_api_test
lidar_c_api_test_OBJECTS = \
"CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o"

# External object files for target lidar_c_api_test
lidar_c_api_test_EXTERNAL_OBJECTS =

lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/lidar_c_api_test.c.o
lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/build.make
lidar_c_api_test: libydlidar_sdk.a
lidar_c_api_test: examples/CMakeFiles/lidar_c_api_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/reveha/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../lidar_c_api_test"
	cd /home/reveha/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar_c_api_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/lidar_c_api_test.dir/build: lidar_c_api_test
.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/build

examples/CMakeFiles/lidar_c_api_test.dir/clean:
	cd /home/reveha/YDLidar-SDK/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/lidar_c_api_test.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/clean

examples/CMakeFiles/lidar_c_api_test.dir/depend:
	cd /home/reveha/YDLidar-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reveha/YDLidar-SDK /home/reveha/YDLidar-SDK/examples /home/reveha/YDLidar-SDK/build /home/reveha/YDLidar-SDK/build/examples /home/reveha/YDLidar-SDK/build/examples/CMakeFiles/lidar_c_api_test.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/lidar_c_api_test.dir/depend

