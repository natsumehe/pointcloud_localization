cmake_minimum_required(VERSION 3.10)
project(PointCloudRegistration)

find_package(PCL REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main
    src/main.cpp
    src/io_utils.cpp
    src/voxelizer.cpp
    src/score_table.cpp
    src/csm_matcher.cpp
)

target_link_libraries(main ${PCL_LIBRARIES})

add_executable(simulate_data src/simulate_data.cpp)
target_link_libraries(simulate_data ${PCL_LIBRARIES})
