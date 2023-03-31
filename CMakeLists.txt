cmake_minimum_required(VERSION 2.8)
project( LaserDotMeasure )
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package( OpenCV REQUIRED )
find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
include_directories( include
		     		${OpenCV_INCLUDE_DIRS}
		     		${Pylon_INCLUDE_DIRS}
	             	${pylon_camera_INCLUDE_DIRS}
 )



add_executable( calculated_laser_line calculated_laser_line.cpp )
target_link_libraries( calculated_laser_line
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( LaserDotAlignment LaserDotAlignment.cpp )
target_link_libraries( LaserDotAlignment
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( LaserLineAlignment LaserLineAlignment.cpp )
target_link_libraries( LaserLineAlignment
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)



