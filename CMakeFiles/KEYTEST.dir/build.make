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
include CMakeFiles/KEYTEST.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/KEYTEST.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/KEYTEST.dir/flags.make

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o: CMakeFiles/KEYTEST.dir/flags.make
CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o: KEYTEST.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/KEYTEST.cpp

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KEYTEST.dir/KEYTEST.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/KEYTEST.cpp > CMakeFiles/KEYTEST.dir/KEYTEST.cpp.i

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KEYTEST.dir/KEYTEST.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/KEYTEST.cpp -o CMakeFiles/KEYTEST.dir/KEYTEST.cpp.s

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.requires:

.PHONY : CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.requires

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.provides: CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.requires
	$(MAKE) -f CMakeFiles/KEYTEST.dir/build.make CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.provides.build
.PHONY : CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.provides

CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.provides.build: CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o


# Object files for target KEYTEST
KEYTEST_OBJECTS = \
"CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o"

# External object files for target KEYTEST
KEYTEST_EXTERNAL_OBJECTS =

KEYTEST: CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o
KEYTEST: CMakeFiles/KEYTEST.dir/build.make
KEYTEST: /usr/local/lib/libopencv_gapi.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_highgui.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_ml.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_objdetect.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_photo.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_stitching.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_video.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_videoio.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_imgcodecs.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_dnn.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_calib3d.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_features2d.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_flann.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_imgproc.so.4.7.0
KEYTEST: /usr/local/lib/libopencv_core.so.4.7.0
KEYTEST: CMakeFiles/KEYTEST.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable KEYTEST"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KEYTEST.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/KEYTEST.dir/build: KEYTEST

.PHONY : CMakeFiles/KEYTEST.dir/build

CMakeFiles/KEYTEST.dir/requires: CMakeFiles/KEYTEST.dir/KEYTEST.cpp.o.requires

.PHONY : CMakeFiles/KEYTEST.dir/requires

CMakeFiles/KEYTEST.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/KEYTEST.dir/cmake_clean.cmake
.PHONY : CMakeFiles/KEYTEST.dir/clean

CMakeFiles/KEYTEST.dir/depend:
	cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/CMakeFiles/KEYTEST.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/KEYTEST.dir/depend

