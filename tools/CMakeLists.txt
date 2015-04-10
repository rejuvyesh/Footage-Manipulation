# Create exectuable with target name "calibrate"
add_executable(calibrate calibrate.cpp)
add_executable(undistort undistort.cpp)

# Link target with libraries
target_link_libraries(calibrate undistort ${OpenCV_LIBS} ${Boost_LIBRARIES})
