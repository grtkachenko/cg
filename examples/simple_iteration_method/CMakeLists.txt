cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(simple_iteration_method)

add_executable(si_method si_method.cpp)
target_link_libraries(si_method visualization ${GMP_LIBRARIES})