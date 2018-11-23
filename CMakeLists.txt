cmake_minimum_required(VERSION 2.8.3)
project(conbv)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(LCMAES_PATH "~/rsrch/plan/CoNBV/lib/cmaes/install/lib")
set(LCMAES_INCLUDES "~/rsrch/plan/CoNBV/lib/cmaes/install/include")

#find_library(libcmaes NAMES libcmaes.so PATHS ${LCMAES_PATH})
add_library(libcmaes SHARED IMPORTED)
set_property(TARGET libcmaes PROPERTY IMPORTED_LOCATION "~/rsrch/plan/CoNBV/lib/cmaes/install/lib/libcmaes.so")

include_directories(include ${LCMAES_INCLUDES} ${EIGEN3_INCLUDE_DIRS})
add_executable(conbv src/main.cpp)
target_link_libraries(conbv libcmaes)
