cmake_minimum_required(VERSION 2.8.3)
project(diff_elevation_mapping)

set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

find_package(catkin REQUIRED
             COMPONENTS
             roscpp
             tf
             nav_msgs
             sensor_msgs
             pcl_ros
             pcl_conversions
             pcl_msgs
             laser_geometry)

find_package(Boost REQUIRED)


catkin_package(
    CATKIN_DEPENDS
        roscpp
        tf
        pcl_ros
        pcl_conversions
        pcl_msgs
        laser_geometry
  INCLUDE_DIRS include
  LIBRARIES diff_elevation_mapping_map
)

include_directories(include/diff_elevation_mapping include/diff_elevation_mapping/map)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_library(diff_elevation_mapping_map
                    src/diff_elevation_mapping/map/map.cpp)
target_link_libraries(diff_elevation_mapping_map)


add_executable(diff_elevation_mapping
                       src/diff_elevation_mapping_node.cpp)

target_link_libraries(diff_elevation_mapping
    diff_elevation_mapping_map
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    diff_elevation_mapping diff_elevation_mapping_map
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/diff_elevation_mapping/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
