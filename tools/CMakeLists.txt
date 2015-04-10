# Create exectuable with target name "calibrate"
add_executable(correct_distort correct_distort.cpp)

# Link target with libraries
target_link_libraries(correct_distort ${OpenCV_LIBS} ${Boost_LIBRARIES})
