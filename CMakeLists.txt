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



add_executable( get_rvec_tvec get_rvec_tvec.cpp )
target_link_libraries( get_rvec_tvec
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}
                      
)

add_executable( planes_intersection planes_intersection.cpp )
target_link_libraries( planes_intersection
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( calculated_laser_line calculated_laser_line.cpp )
target_link_libraries( calculated_laser_line
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)

add_executable( AG_DEMO AG_DEMO.cpp )
target_link_libraries( AG_DEMO
		       ${OpenCV_LIBS}
	               ${Pylon_LIBRARIES}
                       ${pylon_camera_INCLUDE_DIRS}

)
