cmake_minimum_required(VERSION 3.10)
project(AbstractClassExample)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_library(A SHARED A.cpp)
add_library(B SHARED B.cpp)

target_link_libraries(B A)

add_executable(MyApp main.cpp)
target_link_libraries(MyApp B)
