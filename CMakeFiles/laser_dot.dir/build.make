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
CMAKE_SOURCE_DIR = /home/lingbo/Documents/GitHub/AssemblyGuidanceTool

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lingbo/Documents/GitHub/AssemblyGuidanceTool

# Include any dependencies generated for this target.
include CMakeFiles/laser_dot.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/laser_dot.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/laser_dot.dir/flags.make

CMakeFiles/laser_dot.dir/laser_dot.cpp.o: CMakeFiles/laser_dot.dir/flags.make
CMakeFiles/laser_dot.dir/laser_dot.cpp.o: laser_dot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/laser_dot.dir/laser_dot.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laser_dot.dir/laser_dot.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laser_dot.cpp

CMakeFiles/laser_dot.dir/laser_dot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laser_dot.dir/laser_dot.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laser_dot.cpp > CMakeFiles/laser_dot.dir/laser_dot.cpp.i

CMakeFiles/laser_dot.dir/laser_dot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laser_dot.dir/laser_dot.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laser_dot.cpp -o CMakeFiles/laser_dot.dir/laser_dot.cpp.s

CMakeFiles/laser_dot.dir/laser_dot.cpp.o.requires:

.PHONY : CMakeFiles/laser_dot.dir/laser_dot.cpp.o.requires

CMakeFiles/laser_dot.dir/laser_dot.cpp.o.provides: CMakeFiles/laser_dot.dir/laser_dot.cpp.o.requires
	$(MAKE) -f CMakeFiles/laser_dot.dir/build.make CMakeFiles/laser_dot.dir/laser_dot.cpp.o.provides.build
.PHONY : CMakeFiles/laser_dot.dir/laser_dot.cpp.o.provides

CMakeFiles/laser_dot.dir/laser_dot.cpp.o.provides.build: CMakeFiles/laser_dot.dir/laser_dot.cpp.o


# Object files for target laser_dot
laser_dot_OBJECTS = \
"CMakeFiles/laser_dot.dir/laser_dot.cpp.o"

# External object files for target laser_dot
laser_dot_EXTERNAL_OBJECTS =

laser_dot: CMakeFiles/laser_dot.dir/laser_dot.cpp.o
laser_dot: CMakeFiles/laser_dot.dir/build.make
laser_dot: /usr/local/lib/libopencv_gapi.so.4.7.0
laser_dot: /usr/local/lib/libopencv_highgui.so.4.7.0
laser_dot: /usr/local/lib/libopencv_ml.so.4.7.0
laser_dot: /usr/local/lib/libopencv_objdetect.so.4.7.0
laser_dot: /usr/local/lib/libopencv_photo.so.4.7.0
laser_dot: /usr/local/lib/libopencv_stitching.so.4.7.0
laser_dot: /usr/local/lib/libopencv_video.so.4.7.0
laser_dot: /usr/local/lib/libopencv_videoio.so.4.7.0
laser_dot: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
laser_dot: /usr/local/lib/libopencv_dnn.so.4.7.0
laser_dot: /usr/local/lib/libopencv_calib3d.so.4.7.0
laser_dot: /usr/local/lib/libopencv_features2d.so.4.7.0
laser_dot: /usr/local/lib/libopencv_flann.so.4.7.0
laser_dot: /usr/local/lib/libopencv_imgproc.so.4.7.0
laser_dot: /usr/local/lib/libopencv_core.so.4.7.0
laser_dot: CMakeFiles/laser_dot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable laser_dot"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laser_dot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/laser_dot.dir/build: laser_dot

.PHONY : CMakeFiles/laser_dot.dir/build

CMakeFiles/laser_dot.dir/requires: CMakeFiles/laser_dot.dir/laser_dot.cpp.o.requires

.PHONY : CMakeFiles/laser_dot.dir/requires

CMakeFiles/laser_dot.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/laser_dot.dir/cmake_clean.cmake
.PHONY : CMakeFiles/laser_dot.dir/clean

CMakeFiles/laser_dot.dir/depend:
	cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles/laser_dot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/laser_dot.dir/depend
