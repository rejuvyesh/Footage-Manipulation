# Footage-Manipulation
## Introduction
This project provides tools to help with the manipulation of the footage. The currently available tools are 

* the **calibration** of the camera using a checkerboard _(tools/calibrate.cpp)_, 
* the **undistortion** of the footage _(tools/undistort.cpp)_,
* the **stabilization** of the footage _(tools/undistort.cpp)_ and
* the **splitting** of the footage into sub rectangles. _(tools/split_vid)_

## How to Build and Run

### Dependencies
1. C++ compiler
2. OpenCV2
3. BOOST libraries
4. CMake

### Steps
1. Install dependencies if not already done.
2. Make a new directory to compile the code into 
```
$> mkdir manip_bin
```
3. Change into that directory
```
$> cd manip_bin
```
4. CMake the project
```
$> cmake ..
```
5. Make the project 
```
$> make
```
6. Run the tool you want to use
```
$> tools/[Your tool of choice]
```

## What each tool does
### Calibrate
This is the calibration software for the camera.

#### How do you use it?
```
Usage: tools/calibrate [options] <horiz> <vert> <input>

Options:
  -h [ --help ]	Print help messages
  --horiz arg                       Number of horizontal corners
  --vert arg                        Number of vertical corners
  -i [ --input ] arg                Input directory
  -c [ --calibfn ] arg (=calib.yml) calibration filename
```
#### What does it do?
Takes in a directory that contains all the calibration images that you have with the checkerboard and writes the corresponding calibration output file.

### Undistort
This is the undistortion software for the camera.

#### How do you use it?
```
Usage: tools/undistort [options] <calibfn> <footage>

Options:
  -h [ --help ]         Print help messages
  -c [ --calibfn ] arg  calibration filename
  -f [ --footage ] arg  footage file
```
#### What does it do?
Takes in the footage that you have recorded as well as the calibration file that was created from the **calibration** tool and undistorts the footage.

N.B. For now we do not save the video.

### Stabilize
This is the stabilization software for the camera.

#### How do you use it?
```
Usage: tools/stabilize [options] <footage>

Options:
  -h [ --help ]                     Print help messages
  -b [ --boxsize ] arg (=20)        The size of the box that you search for the
                                    best point to track in
  -c [ --hcrop ] arg (=30)          Horizontal Border Crop, crops the border to
                                    reduce the black borders from stabilization
                                    being too noticeable.
  -s [ --startframe ] arg (=0)      Frame to do manual capturing on.
  -f [ --footage ] arg              footage file
  -o [ --output ] arg (=output.avi) output file

```
#### What does it do?
Takes in the footage that you have recorded and creates the corresponding stabilized footage. When run manually select the points that you want to track.

### Splitting
This is the splitting software for the footage.

#### How do you use it?
```
Usage: tools/split_vid [options] <footage> <numx> <numy>

Options:
  -h [ --help ]            Print help messages
  -f [ --footage ] arg     footage file
  -x [ --numx ] arg        number of x splits
  -y [ --numy ] arg        number of y splits
  -o [ --output ] arg (=.) output directory
```
#### What does it do?
Takes in the stabilized footage and splits it into the number of rectangles you would like.