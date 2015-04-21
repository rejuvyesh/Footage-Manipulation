cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

project(drone)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++1y")

# Find PCL, OpenCV and Boost
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED program_options system filesystem)


include_directories("include" ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Recurse into subdirectories
#add_subdirectory(lib)
add_subdirectory(tools)
