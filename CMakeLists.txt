#Project Setup
cmake_minimum_required(VERSION 3.0)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17")
project(Sample)

#Dependencies
find_package( OpenCV REQUIRED )

# Source and Header files
file(GLOB SRC_FILES "src/*.cpp")
set(INCL_FILES_DIR "include/")

# Local include/ folder
include_directories(
    ${INCL_FILES_DIR}   
)
# OpenCV lib
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Make executable
add_executable(
    imgProc
    ${SRC_FILES}
)

target_link_libraries( imgProc ${OpenCV_LIBS} )