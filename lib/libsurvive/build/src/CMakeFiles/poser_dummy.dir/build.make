# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build

# Include any dependencies generated for this target.
include src/CMakeFiles/poser_dummy.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/poser_dummy.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/poser_dummy.dir/flags.make

src/CMakeFiles/poser_dummy.dir/poser_dummy.c.o: src/CMakeFiles/poser_dummy.dir/flags.make
src/CMakeFiles/poser_dummy.dir/poser_dummy.c.o: ../src/poser_dummy.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/CMakeFiles/poser_dummy.dir/poser_dummy.c.o"
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/poser_dummy.dir/poser_dummy.c.o   -c /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/src/poser_dummy.c

src/CMakeFiles/poser_dummy.dir/poser_dummy.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/poser_dummy.dir/poser_dummy.c.i"
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/src/poser_dummy.c > CMakeFiles/poser_dummy.dir/poser_dummy.c.i

src/CMakeFiles/poser_dummy.dir/poser_dummy.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/poser_dummy.dir/poser_dummy.c.s"
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/src/poser_dummy.c -o CMakeFiles/poser_dummy.dir/poser_dummy.c.s

# Object files for target poser_dummy
poser_dummy_OBJECTS = \
"CMakeFiles/poser_dummy.dir/poser_dummy.c.o"

# External object files for target poser_dummy
poser_dummy_EXTERNAL_OBJECTS =

plugins/poser_dummy.so: src/CMakeFiles/poser_dummy.dir/poser_dummy.c.o
plugins/poser_dummy.so: src/CMakeFiles/poser_dummy.dir/build.make
plugins/poser_dummy.so: libsurvive.so.0.3
plugins/poser_dummy.so: redist/libmpfit.a
plugins/poser_dummy.so: libs/cnkalman/src/libcnkalman.a
plugins/poser_dummy.so: libs/cnkalman/libs/cnmatrix/src/libcnmatrix.a
plugins/poser_dummy.so: src/CMakeFiles/poser_dummy.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../plugins/poser_dummy.so"
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/poser_dummy.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/poser_dummy.dir/build: plugins/poser_dummy.so

.PHONY : src/CMakeFiles/poser_dummy.dir/build

src/CMakeFiles/poser_dummy.dir/clean:
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src && $(CMAKE_COMMAND) -P CMakeFiles/poser_dummy.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/poser_dummy.dir/clean

src/CMakeFiles/poser_dummy.dir/depend:
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/src /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/src/CMakeFiles/poser_dummy.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/poser_dummy.dir/depend

