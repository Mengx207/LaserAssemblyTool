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
include CMakeFiles/laserorigin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/laserorigin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/laserorigin.dir/flags.make

CMakeFiles/laserorigin.dir/laserorigin.cpp.o: CMakeFiles/laserorigin.dir/flags.make
CMakeFiles/laserorigin.dir/laserorigin.cpp.o: laserorigin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/laserorigin.dir/laserorigin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserorigin.dir/laserorigin.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laserorigin.cpp

CMakeFiles/laserorigin.dir/laserorigin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserorigin.dir/laserorigin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laserorigin.cpp > CMakeFiles/laserorigin.dir/laserorigin.cpp.i

CMakeFiles/laserorigin.dir/laserorigin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserorigin.dir/laserorigin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/laserorigin.cpp -o CMakeFiles/laserorigin.dir/laserorigin.cpp.s

CMakeFiles/laserorigin.dir/laserorigin.cpp.o.requires:

.PHONY : CMakeFiles/laserorigin.dir/laserorigin.cpp.o.requires

CMakeFiles/laserorigin.dir/laserorigin.cpp.o.provides: CMakeFiles/laserorigin.dir/laserorigin.cpp.o.requires
	$(MAKE) -f CMakeFiles/laserorigin.dir/build.make CMakeFiles/laserorigin.dir/laserorigin.cpp.o.provides.build
.PHONY : CMakeFiles/laserorigin.dir/laserorigin.cpp.o.provides

CMakeFiles/laserorigin.dir/laserorigin.cpp.o.provides.build: CMakeFiles/laserorigin.dir/laserorigin.cpp.o


# Object files for target laserorigin
laserorigin_OBJECTS = \
"CMakeFiles/laserorigin.dir/laserorigin.cpp.o"

# External object files for target laserorigin
laserorigin_EXTERNAL_OBJECTS =

laserorigin: CMakeFiles/laserorigin.dir/laserorigin.cpp.o
laserorigin: CMakeFiles/laserorigin.dir/build.make
laserorigin: /usr/local/lib/libopencv_gapi.so.4.7.0
laserorigin: /usr/local/lib/libopencv_highgui.so.4.7.0
laserorigin: /usr/local/lib/libopencv_ml.so.4.7.0
laserorigin: /usr/local/lib/libopencv_objdetect.so.4.7.0
laserorigin: /usr/local/lib/libopencv_photo.so.4.7.0
laserorigin: /usr/local/lib/libopencv_stitching.so.4.7.0
laserorigin: /usr/local/lib/libopencv_video.so.4.7.0
laserorigin: /usr/local/lib/libopencv_videoio.so.4.7.0
laserorigin: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
laserorigin: /usr/local/lib/libopencv_dnn.so.4.7.0
laserorigin: /usr/local/lib/libopencv_calib3d.so.4.7.0
laserorigin: /usr/local/lib/libopencv_features2d.so.4.7.0
laserorigin: /usr/local/lib/libopencv_flann.so.4.7.0
laserorigin: /usr/local/lib/libopencv_imgproc.so.4.7.0
laserorigin: /usr/local/lib/libopencv_core.so.4.7.0
laserorigin: CMakeFiles/laserorigin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable laserorigin"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laserorigin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/laserorigin.dir/build: laserorigin

.PHONY : CMakeFiles/laserorigin.dir/build

CMakeFiles/laserorigin.dir/requires: CMakeFiles/laserorigin.dir/laserorigin.cpp.o.requires

.PHONY : CMakeFiles/laserorigin.dir/requires

CMakeFiles/laserorigin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/laserorigin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/laserorigin.dir/clean

CMakeFiles/laserorigin.dir/depend:
	cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles/laserorigin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/laserorigin.dir/depend

