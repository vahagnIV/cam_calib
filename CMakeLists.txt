cmake_minimum_required(VERSION 3.13)
project(g2o_ba_test)
set(CMAKE_CXX_STANDARD 11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
find_package(g2o REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

find_package(Eigen3 REQUIRED)
find_package(Cholmod)
#CHOLMOD
message(${CHOLMOD_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})
include_directories(g2o ${Eigen3_INCLUDE_DIRECTORIES})
add_executable(g2o_ba_test main.cpp)

target_link_libraries(g2o_ba_test g2o::core g2o::solver_cholmod g2o::types_sba Eigen3::Eigen)