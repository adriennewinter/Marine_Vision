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
CMAKE_SOURCE_DIR = /home/user/Documents/ROS_Workspace/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/Documents/ROS_Workspace/build

# Include any dependencies generated for this target.
include gscam/CMakeFiles/gscam.dir/depend.make

# Include the progress variables for this target.
include gscam/CMakeFiles/gscam.dir/progress.make

# Include the compile flags for this target's objects.
include gscam/CMakeFiles/gscam.dir/flags.make

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o: gscam/CMakeFiles/gscam.dir/flags.make
gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o: /home/user/Documents/ROS_Workspace/src/gscam/src/gscam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/Documents/ROS_Workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o"
	cd /home/user/Documents/ROS_Workspace/build/gscam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gscam.dir/src/gscam.cpp.o -c /home/user/Documents/ROS_Workspace/src/gscam/src/gscam.cpp

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gscam.dir/src/gscam.cpp.i"
	cd /home/user/Documents/ROS_Workspace/build/gscam && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/Documents/ROS_Workspace/src/gscam/src/gscam.cpp > CMakeFiles/gscam.dir/src/gscam.cpp.i

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gscam.dir/src/gscam.cpp.s"
	cd /home/user/Documents/ROS_Workspace/build/gscam && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/Documents/ROS_Workspace/src/gscam/src/gscam.cpp -o CMakeFiles/gscam.dir/src/gscam.cpp.s

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.requires:

.PHONY : gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.requires

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.provides: gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.requires
	$(MAKE) -f gscam/CMakeFiles/gscam.dir/build.make gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.provides.build
.PHONY : gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.provides

gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.provides.build: gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o


# Object files for target gscam
gscam_OBJECTS = \
"CMakeFiles/gscam.dir/src/gscam.cpp.o"

# External object files for target gscam
gscam_EXTERNAL_OBJECTS =

/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: gscam/CMakeFiles/gscam.dir/build.make
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libimage_transport.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libnodeletlib.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libbondcpp.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libuuid.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libclass_loader.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/libPocoFoundation.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libdl.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libroslib.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/librospack.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libcamera_info_manager.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libcamera_calibration_parsers.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libroscpp.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/librosconsole.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/librostime.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /opt/ros/melodic/lib/libcpp_common.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/user/Documents/ROS_Workspace/devel/lib/libgscam.so: gscam/CMakeFiles/gscam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/Documents/ROS_Workspace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/user/Documents/ROS_Workspace/devel/lib/libgscam.so"
	cd /home/user/Documents/ROS_Workspace/build/gscam && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gscam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gscam/CMakeFiles/gscam.dir/build: /home/user/Documents/ROS_Workspace/devel/lib/libgscam.so

.PHONY : gscam/CMakeFiles/gscam.dir/build

gscam/CMakeFiles/gscam.dir/requires: gscam/CMakeFiles/gscam.dir/src/gscam.cpp.o.requires

.PHONY : gscam/CMakeFiles/gscam.dir/requires

gscam/CMakeFiles/gscam.dir/clean:
	cd /home/user/Documents/ROS_Workspace/build/gscam && $(CMAKE_COMMAND) -P CMakeFiles/gscam.dir/cmake_clean.cmake
.PHONY : gscam/CMakeFiles/gscam.dir/clean

gscam/CMakeFiles/gscam.dir/depend:
	cd /home/user/Documents/ROS_Workspace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/Documents/ROS_Workspace/src /home/user/Documents/ROS_Workspace/src/gscam /home/user/Documents/ROS_Workspace/build /home/user/Documents/ROS_Workspace/build/gscam /home/user/Documents/ROS_Workspace/build/gscam/CMakeFiles/gscam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gscam/CMakeFiles/gscam.dir/depend

