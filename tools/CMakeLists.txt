# Create exectuable with target name "calibrate"
add_executable(calibrate calibrate.cpp)
add_executable(undistort undistort.cpp)
add_executable(stabilize stabilize.cpp)
add_executable(split_vid split_vid.cpp)

add_executable(stabilizedev stabilizedev.cpp)
add_executable(videostab videostab.cpp)



# Link target with libraries
target_link_libraries(calibrate ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(undistort ${OpenCV_LIBS} ${Boost_LIBRARIES})
target_link_libraries(stabilize LINK_PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} Ert)
target_link_libraries(split_vid ${OpenCV_LIBS} ${Boost_LIBRARIES})

target_link_libraries(stabilizedev LINK_PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} Ert)
target_link_libraries(videostab LINK_PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} Ert)

# Link the executable to the ERT library. Since the ERT library has
# public include directories we will use those link directories when stabilize
