cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(tuto-pairwise)

find_package(PCL 1.4 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})

## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-march=core2 -O3 -pipe -openmp -msse -msse2 -msse3 -mmmx")        ## Optimize
    #set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
endif()
