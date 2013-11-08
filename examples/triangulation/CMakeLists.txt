cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(triangulation-example)

add_executable(delaunay delaunay.cpp)
target_link_libraries(delaunay visualization ${GMP_LIBRARIES})