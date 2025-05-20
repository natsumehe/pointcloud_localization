cmake_minimum_required(VERSION 3.10)
project(PointCloudLocalization)

set(CMAKE_CXX_STANDARD 17)

# 如果使用 PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(include)

add_executable(main
    src/main.cpp
    src/voxelizer.cpp
    src/score_table.cpp
    src/csm_matcher.cpp
    src/io_utils.cpp
)
target_link_libraries(main ${PCL_LIBRARIES})
