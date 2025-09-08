cmake_minimum_required(VERSION 3.16)
project(pathlab CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(pathlab_core
  src/core/grid_map.cpp
  src/io/scen_loader.cpp
)

target_include_directories(pathlab_core PUBLIC include)

add_executable(bench_single apps/bench_single/main.cpp)
target_link_libraries(bench_single PRIVATE pathlab_core)
