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
CMAKE_BINARY_DIR = /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build

# Include any dependencies generated for this target.
include CMakeFiles/laserplaneguide.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/laserplaneguide.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/laserplaneguide.dir/flags.make

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o: CMakeFiles/laserplaneguide.dir/flags.make
CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o: ../src/laserplaneguide.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/laserplaneguide.cpp

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/laserplaneguide.cpp > CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.i

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/laserplaneguide.cpp -o CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.s

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.requires:

.PHONY : CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.requires

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.provides: CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.requires
	$(MAKE) -f CMakeFiles/laserplaneguide.dir/build.make CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.provides.build
.PHONY : CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.provides

CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.provides.build: CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o


CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o: CMakeFiles/laserplaneguide.dir/flags.make
CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o: ../src/imgpro.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/imgpro.cpp

CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/imgpro.cpp > CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.i

CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/imgpro.cpp -o CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.s

CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.requires:

.PHONY : CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.requires

CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.provides: CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.requires
	$(MAKE) -f CMakeFiles/laserplaneguide.dir/build.make CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.provides.build
.PHONY : CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.provides

CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.provides.build: CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o


CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o: CMakeFiles/laserplaneguide.dir/flags.make
CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o: ../src/gencal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o -c /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/gencal.cpp

CMakeFiles/laserplaneguide.dir/src/gencal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laserplaneguide.dir/src/gencal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/gencal.cpp > CMakeFiles/laserplaneguide.dir/src/gencal.cpp.i

CMakeFiles/laserplaneguide.dir/src/gencal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laserplaneguide.dir/src/gencal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/src/gencal.cpp -o CMakeFiles/laserplaneguide.dir/src/gencal.cpp.s

CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.requires:

.PHONY : CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.requires

CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.provides: CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.requires
	$(MAKE) -f CMakeFiles/laserplaneguide.dir/build.make CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.provides.build
.PHONY : CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.provides

CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.provides.build: CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o


# Object files for target laserplaneguide
laserplaneguide_OBJECTS = \
"CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o" \
"CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o" \
"CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o"

# External object files for target laserplaneguide
laserplaneguide_EXTERNAL_OBJECTS =

../laserplaneguide: CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o
../laserplaneguide: CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o
../laserplaneguide: CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o
../laserplaneguide: CMakeFiles/laserplaneguide.dir/build.make
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
../laserplaneguide: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
../laserplaneguide: CMakeFiles/laserplaneguide.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../laserplaneguide"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laserplaneguide.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/laserplaneguide.dir/build: ../laserplaneguide

.PHONY : CMakeFiles/laserplaneguide.dir/build

CMakeFiles/laserplaneguide.dir/requires: CMakeFiles/laserplaneguide.dir/src/laserplaneguide.cpp.o.requires
CMakeFiles/laserplaneguide.dir/requires: CMakeFiles/laserplaneguide.dir/src/imgpro.cpp.o.requires
CMakeFiles/laserplaneguide.dir/requires: CMakeFiles/laserplaneguide.dir/src/gencal.cpp.o.requires

.PHONY : CMakeFiles/laserplaneguide.dir/requires

CMakeFiles/laserplaneguide.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/laserplaneguide.dir/cmake_clean.cmake
.PHONY : CMakeFiles/laserplaneguide.dir/clean

CMakeFiles/laserplaneguide.dir/depend:
	cd /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build /home/lingbo/Documents/GitHub/AssemblyGuidanceTool/build/CMakeFiles/laserplaneguide.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/laserplaneguide.dir/depend

