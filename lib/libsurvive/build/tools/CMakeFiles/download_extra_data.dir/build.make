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

# Utility rule file for download_extra_data.

# Include the progress variables for this target.
include tools/CMakeFiles/download_extra_data.dir/progress.make

download_extra_data: tools/CMakeFiles/download_extra_data.dir/build.make

.PHONY : download_extra_data

# Rule to build all files generated by this target.
tools/CMakeFiles/download_extra_data.dir/build: download_extra_data

.PHONY : tools/CMakeFiles/download_extra_data.dir/build

tools/CMakeFiles/download_extra_data.dir/clean:
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/tools && $(CMAKE_COMMAND) -P CMakeFiles/download_extra_data.dir/cmake_clean.cmake
.PHONY : tools/CMakeFiles/download_extra_data.dir/clean

tools/CMakeFiles/download_extra_data.dir/depend:
	cd /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/tools /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/tools /home/agilex/oculus_ws/src/cobot_remote/lib/libsurvive/build/tools/CMakeFiles/download_extra_data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/CMakeFiles/download_extra_data.dir/depend
