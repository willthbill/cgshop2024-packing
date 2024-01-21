cmake_minimum_required(VERSION 3.1...3.15)

project( cgshop-lib )

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()  
endif()

message(STATUS "CXX ${CXX}")

find_package(SCIP QUIET)
include(CGAL_SCIP_support)
if(NOT TARGET CGAL::SCIP_support)
    message(STATUS "NOTICE: This project requires SCIP and will not be compiled.")
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

set(CXX 1) # C++ libs for gurobi
find_package(GUROBI REQUIRED)
if (NOT GUROBI_CXX_LIBRARY OR NOT GUROBI_LIBRARY)
    message(STATUS "This project requires Gurobi, and will not be compiled")
endif()


include_directories(${Boost_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(src/cpp)
include_directories(../cgshop-lib)
include_directories(${GUROBI_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})
link_directories(../cgshop-lib/out/build/lib)

get_directory_property(_include_dirs INCLUDE_DIRECTORIES)
message(STATUS "Include directories: ${_include_dirs}")
get_directory_property(_link_dirs LINK_DIRECTORIES)
message(STATUS "Link directories: ${_link_dirs}")

# for LSP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I$ENV{CGSHOP_LIB_PATH_HOST}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I$ENV{SRC_PATH_HOST}")
# include_directories(${CGSHOP_LIB_PATH_HOST})

set(CMAKE_SHARED_LIBRARY_PREFIX "")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "cpp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__local__")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wfatal-errors")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -Wshadow -Wformat=2 -Wfloat-equal -Wconversion -Wlogical-op -Wshift-overflow=2 -Wduplicated-cond -Wcast-qual -Wcast-align -Wfatal-errors")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fsanitize=undefined -fno-sanitize-recover -fstack-protector")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC -D_FORTIFY_SOURCE=2")

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

set(CGALLIB CGAL::CGAL CGAL::SCIP_support)

set(PRECOMPILE_HEADERS "src/cpp/pch.h")

###### CPP ######
### lib2 ###
add_library(util SHARED src/cpp/lib2/util.cpp)
target_precompile_headers(util PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(util ${CGALLIB})

add_library(io SHARED src/cpp/io.cpp)
target_precompile_headers(io PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(io ${CGALLIB} util geometry_utils intersection_predicates)

add_library(configuration_space SHARED src/cpp/lib2/configuration_space.cpp)
target_precompile_headers(configuration_space PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(configuration_space ${CGALLIB} geometry_utils util)

add_library(mip SHARED src/cpp/lib2/mip/mip.cpp)
target_link_libraries(mip ${CGALLIB})

add_library(scip_mip SHARED src/cpp/lib2/mip/scip.cpp)
target_link_libraries(scip_mip ${CGALLIB} geometry_utils mip)

add_library(gurobi_mip SHARED src/cpp/lib2/mip/gurobi.cpp)
target_link_libraries(gurobi_mip ${CGALLIB} mip ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

add_library(snap SHARED src/cpp/lib2/snap.cpp)
target_precompile_headers(snap PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(snap ${CGALLIB} intersection_predicates geometry_utils util)

add_library(convex_cover SHARED src/cpp/lib2/convex_cover.cpp)
target_precompile_headers(convex_cover PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(convex_cover ${CGALLIB} partition_constructor visgraph cliquecover geometry_utils util)

add_library(simplification SHARED src/cpp/lib2/simplification.cpp)
target_precompile_headers(simplification PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(simplification ${CGALLIB} geometry_utils util)

# add_library(optimal_packing SHARED src/cpp/lib2/optimal_packing.cpp)
# target_precompile_headers(optimal_packing PUBLIC ${PRECOMPILE_HEADERS})
# target_link_libraries(optimal_packing ${CGALLIB} partition_constructor gurobi_mip configuration_space geometry_utils io snap convex_cover simplification util)

add_library(heuristic_packing SHARED src/cpp/lib2/heuristic_packing.cpp)
target_precompile_headers(heuristic_packing PUBLIC ${PRECOMPILE_HEADERS})
target_link_libraries(heuristic_packing ${CGALLIB} 
    # optimal_packing
    partition_constructor gurobi_mip configuration_space geometry_utils io snap convex_cover simplification util
)

# add_library(repacking SHARED src/cpp/lib2/repacking.cpp)
# target_precompile_headers(repacking PUBLIC ${PRECOMPILE_HEADERS})
# target_link_libraries(repacking ${CGALLIB} heuristic_packing optimal_packing partition_constructor gurobi_mip configuration_space geometry_utils io snap convex_cover simplification util tree)

### algorithms ###
#add_library(dummy_algorithm SHARED src/cpp/algorithms/dummy.cpp)
#target_link_libraries(dummy_algorithm CGAL::CGAL util)

# add_library(optimal_algorithm SHARED src/cpp/algorithms/optimal.cpp)
# target_precompile_headers(optimal_algorithm PUBLIC ${PRECOMPILE_HEADERS})
# target_link_libraries(optimal_algorithm optimal_packing)

#add_executable(mip_example src/cpp/lib2/mip_example.cpp)
#target_link_libraries(mip_example CGAL::CGAL CGAL::SCIP_support)

# add_executable(gurobi_example src/cpp/lib2/mip/gurobi_example.cpp)
# target_link_libraries(gurobi_example ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

###### PY ######
add_library(main${pybind11_suffix} SHARED src/cpp/main.cpp)
target_link_libraries(
    main${pybind11_suffix}
    #repacking
    #optimal_packing
    heuristic_packing
    com CGAL::CGAL
    util
    io
)
