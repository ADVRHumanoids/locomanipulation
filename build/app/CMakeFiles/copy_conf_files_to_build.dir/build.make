# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lucamuratore/walkman/drc/locomanipulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lucamuratore/walkman/drc/locomanipulation/build

# Utility rule file for copy_conf_files_to_build.

# Include the progress variables for this target.
include app/CMakeFiles/copy_conf_files_to_build.dir/progress.make

app/CMakeFiles/copy_conf_files_to_build:
	cd /home/lucamuratore/walkman/drc/locomanipulation/build/app && /usr/bin/cmake -E copy_directory /home/lucamuratore/walkman/drc/locomanipulation/app/conf /home/lucamuratore/walkman/drc/locomanipulation/build/bin

copy_conf_files_to_build: app/CMakeFiles/copy_conf_files_to_build
copy_conf_files_to_build: app/CMakeFiles/copy_conf_files_to_build.dir/build.make
.PHONY : copy_conf_files_to_build

# Rule to build all files generated by this target.
app/CMakeFiles/copy_conf_files_to_build.dir/build: copy_conf_files_to_build
.PHONY : app/CMakeFiles/copy_conf_files_to_build.dir/build

app/CMakeFiles/copy_conf_files_to_build.dir/clean:
	cd /home/lucamuratore/walkman/drc/locomanipulation/build/app && $(CMAKE_COMMAND) -P CMakeFiles/copy_conf_files_to_build.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/copy_conf_files_to_build.dir/clean

app/CMakeFiles/copy_conf_files_to_build.dir/depend:
	cd /home/lucamuratore/walkman/drc/locomanipulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lucamuratore/walkman/drc/locomanipulation /home/lucamuratore/walkman/drc/locomanipulation/app /home/lucamuratore/walkman/drc/locomanipulation/build /home/lucamuratore/walkman/drc/locomanipulation/build/app /home/lucamuratore/walkman/drc/locomanipulation/build/app/CMakeFiles/copy_conf_files_to_build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/copy_conf_files_to_build.dir/depend

