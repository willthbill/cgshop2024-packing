cmake_minimum_required(VERSION 3.1...3.15)

project( cgshop-lib )

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()  
endif()

# Boost and its components
find_package( Boost REQUIRED COMPONENTS system filesystem) 
if ( NOT Boost_FOUND )
    message(STATUS "This project requires the Boost library, and will not be compiled.")
    return()  
endif()

find_package(OpenMP)
if( NOT OpenMP_CXX_FOUND)
    message(STATUS "This project requires OMP library, and will not be compiled.")
    return()
endif()

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories(${CMAKE_SOURCE_DIR}/lib)

include_directories(../cgshop-lib)
link_directories(../cgshop-lib/out/build/lib)

get_directory_property(_link_dirs LINK_DIRECTORIES)
message(STATUS "Link directories: ${_link_dirs}")

# for LSP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I$ENV{CGSHOP_LIB_PATH_HOST}")
# include_directories(${CGSHOP_LIB_PATH_HOST})

set(CMAKE_SHARED_LIBRARY_PREFIX "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -D__local__")
set(CMAKE_CXX_FLAGS_DEBUG "")

message("FLAGS:")
message("- all: "${CMAKE_CXX_FLAGS})
message("- release: "${CMAKE_CXX_FLAGS_RELEASE})
message("- debug: "${CMAKE_CXX_FLAGS_DEBUG})

# for pybind11 libraries
execute_process(COMMAND ./bin/getpybindincludes.sh OUTPUT_VARIABLE pybind11_paths)
include_directories(${pybind11_paths})
execute_process(COMMAND ./bin/getpybindsuffix.sh OUTPUT_VARIABLE pybind11_suffix OUTPUT_STRIP_TRAILING_WHITESPACE)

# TEMPLATES
# lib for cpp
# add_library(lib SHARED lib/lib.cpp)
# target_link_libraries(lib CGAL::CGAL)
# lib for py
# add_library(lib${pybind11_suffix} SHARED src/cpp/algorithms/lib.cpp)
# target_link_libraries(lib${pybind11_suffix} CGAL::CGAL lib)

# CPP
add_library(dummy_algorithm SHARED src/cpp/algorithms/dummy.cpp)
target_link_libraries(dummy_algorithm CGAL::CGAL)

# PY
add_library(main${pybind11_suffix} SHARED src/cpp/main.cpp)
target_link_libraries(
    main${pybind11_suffix}
    dummy_algorithm
    com CGAL::CGAL
)

