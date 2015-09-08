add_library(Ert ert.cpp)

# Make sure the compiler can find include files for our ert library
# when other libraries or executables link to ert

target_include_directories(Ert PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})