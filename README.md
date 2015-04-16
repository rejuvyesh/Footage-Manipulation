# Footage-Manipulation
## Introduction
This project provides tools to help with the manipulation of the footage. The currently available tools are 

* the **calibration** of the camera using a checkerboard _(tools/calibrate.cpp)_, 
 f* the **undistortion** of the footage _(tools/undistort.cpp)_ and
* the **stabilization** of the footage _(tools/undistort.cpp)_

## How to Compile

### Dependencies
1. C++ compiler
2. OpenCV2
3. BOOST libraries
4. CMake

### Steps
1. Install dependencies if not already done.
2. Make a new directory to compile the code into _> mkdir manip_bin_
3. Change into that directory _> cd manip_bin_
4. CMake the project _> cmake .._
5. Make the project _> make_
6. Run the tool you want to use _> tools/[Your tool of choice]_ 