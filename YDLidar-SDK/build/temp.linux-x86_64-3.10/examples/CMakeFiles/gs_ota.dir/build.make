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
CMAKE_BINARY_DIR = /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10

# Include any dependencies generated for this target.
include examples/CMakeFiles/gs_ota.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/gs_ota.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/gs_ota.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/gs_ota.dir/flags.make

examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o: examples/CMakeFiles/gs_ota.dir/flags.make
examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o: /home/reveha/YDLidar-SDK/examples/gs_ota.cpp
examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o: examples/CMakeFiles/gs_ota.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o"
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o -MF CMakeFiles/gs_ota.dir/gs_ota.cpp.o.d -o CMakeFiles/gs_ota.dir/gs_ota.cpp.o -c /home/reveha/YDLidar-SDK/examples/gs_ota.cpp

examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/gs_ota.dir/gs_ota.cpp.i"
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/reveha/YDLidar-SDK/examples/gs_ota.cpp > CMakeFiles/gs_ota.dir/gs_ota.cpp.i

examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/gs_ota.dir/gs_ota.cpp.s"
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/reveha/YDLidar-SDK/examples/gs_ota.cpp -o CMakeFiles/gs_ota.dir/gs_ota.cpp.s

# Object files for target gs_ota
gs_ota_OBJECTS = \
"CMakeFiles/gs_ota.dir/gs_ota.cpp.o"

# External object files for target gs_ota
gs_ota_EXTERNAL_OBJECTS =

gs_ota: examples/CMakeFiles/gs_ota.dir/gs_ota.cpp.o
gs_ota: examples/CMakeFiles/gs_ota.dir/build.make
gs_ota: libydlidar_sdk.a
gs_ota: examples/CMakeFiles/gs_ota.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../gs_ota"
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gs_ota.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/gs_ota.dir/build: gs_ota
.PHONY : examples/CMakeFiles/gs_ota.dir/build

examples/CMakeFiles/gs_ota.dir/clean:
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples && $(CMAKE_COMMAND) -P CMakeFiles/gs_ota.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/gs_ota.dir/clean

examples/CMakeFiles/gs_ota.dir/depend:
	cd /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reveha/YDLidar-SDK /home/reveha/YDLidar-SDK/examples /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10 /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples /home/reveha/YDLidar-SDK/build/temp.linux-x86_64-3.10/examples/CMakeFiles/gs_ota.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/gs_ota.dir/depend

