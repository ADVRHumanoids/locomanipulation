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

# Include any dependencies generated for this target.
include CMakeFiles/locomanipulation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/locomanipulation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/locomanipulation.dir/flags.make

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o: CMakeFiles/locomanipulation.dir/flags.make
CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o: ../src/locoman_main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lucamuratore/walkman/drc/locomanipulation/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o -c /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_main.cpp

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_main.cpp > CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.i

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_main.cpp -o CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.s

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.requires:
.PHONY : CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.requires

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.provides: CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.requires
	$(MAKE) -f CMakeFiles/locomanipulation.dir/build.make CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.provides.build
.PHONY : CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.provides

CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.provides.build: CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o: CMakeFiles/locomanipulation.dir/flags.make
CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o: ../src/locoman_control_thread.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/lucamuratore/walkman/drc/locomanipulation/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o -c /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_control_thread.cpp

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_control_thread.cpp > CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.i

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/lucamuratore/walkman/drc/locomanipulation/src/locoman_control_thread.cpp -o CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.s

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.requires:
.PHONY : CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.requires

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.provides: CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/locomanipulation.dir/build.make CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.provides.build
.PHONY : CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.provides

CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.provides.build: CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o

# Object files for target locomanipulation
locomanipulation_OBJECTS = \
"CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o" \
"CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o"

# External object files for target locomanipulation
locomanipulation_EXTERNAL_OBJECTS =

locomanipulation: CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o
locomanipulation: CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o
locomanipulation: CMakeFiles/locomanipulation.dir/build.make
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libidynutils.so.0.0.1
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_OS.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_sig.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_math.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_dev.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_name.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_init.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libidyntree.so.0.0.1
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libGYM.so.0.0.1
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libidyntree.so.0.0.1
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libkdl-codyco.so.0.1.2
locomanipulation: /opt/ros/indigo/lib/liborocos-kdl.so.1.3.0
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libparamhelp.so.0.0.2
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_dev.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_name.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_init.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_math.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_sig.so.2.3.63
locomanipulation: /home/lucamuratore/walkman/build/install/lib/libYARP_OS.so.2.3.63
locomanipulation: /usr/lib/x86_64-linux-gnu/libboost_system.so
locomanipulation: /usr/lib/x86_64-linux-gnu/libboost_thread.so
locomanipulation: /usr/lib/x86_64-linux-gnu/libpthread.so
locomanipulation: CMakeFiles/locomanipulation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable locomanipulation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/locomanipulation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/locomanipulation.dir/build: locomanipulation
.PHONY : CMakeFiles/locomanipulation.dir/build

CMakeFiles/locomanipulation.dir/requires: CMakeFiles/locomanipulation.dir/src/locoman_main.cpp.o.requires
CMakeFiles/locomanipulation.dir/requires: CMakeFiles/locomanipulation.dir/src/locoman_control_thread.cpp.o.requires
.PHONY : CMakeFiles/locomanipulation.dir/requires

CMakeFiles/locomanipulation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/locomanipulation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/locomanipulation.dir/clean

CMakeFiles/locomanipulation.dir/depend:
	cd /home/lucamuratore/walkman/drc/locomanipulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lucamuratore/walkman/drc/locomanipulation /home/lucamuratore/walkman/drc/locomanipulation /home/lucamuratore/walkman/drc/locomanipulation/build /home/lucamuratore/walkman/drc/locomanipulation/build /home/lucamuratore/walkman/drc/locomanipulation/build/CMakeFiles/locomanipulation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/locomanipulation.dir/depend
