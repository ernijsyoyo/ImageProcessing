#Project Setup
cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17")

project(Sample)

# Source and Header files
file(GLOB SRC_FILES "src/*.cpp")
set(INCL_FILES_DIR "include/")

include_directories(
    ${INCL_FILES_DIR}   
)

add_executable(SampleProject
    ${SRC_FILES}
)

