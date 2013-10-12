cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(visibility-example)

add_executable(visibility visibility.cpp)
target_link_libraries(visibility visualization ${GMP_LIBRARIES})