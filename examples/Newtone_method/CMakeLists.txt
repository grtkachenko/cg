cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Newtone_method)

add_executable(Newtone_method Newtone_method.cpp)
target_link_libraries(Newtone_method visualization ${GMP_LIBRARIES})