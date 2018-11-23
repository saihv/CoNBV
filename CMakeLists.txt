cmake_minimum_required(VERSION 2.8.3)
project(nbvplanner)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(LCMAES_PATH "/home/sai/code/nbvplanner/lib/libcmaes/install/lib")
set(LCMAES_INCLUDES "/home/sai/code/nbvplanner/lib/libcmaes/install/include")

#find_library(libcmaes NAMES libcmaes.so PATHS ${LCMAES_PATH})
add_library(libcmaes SHARED IMPORTED)
set_property(TARGET libcmaes PROPERTY IMPORTED_LOCATION "/home/sai/code/nbvplanner/lib/libcmaes/install/lib/libcmaes.so")

include_directories(include ${LCMAES_INCLUDES} ${EIGEN3_INCLUDE_DIRS})
add_executable(nbvplanner src/main.cpp)
target_link_libraries(nbvplanner libcmaes)
