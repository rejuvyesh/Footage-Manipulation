// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"

// Boost includes
#include <boost/program_options.hpp>

// General C++ includes
#include <iostream>
#include <string>
#include <vector>

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;

int main( int argc, char **argv ) {

  // Argument variables
  int cam; // Buff numbers
  int num_boards; // num of diff poses
  int board_w; // num of horizontal corners
  int board_h; // num of vertical corners
  string calib_fn; // calibration file name
  string footage_fn; // footage file name

  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("poses", po::value<int>(&num_boards)->required(), "Number of different poses")
      ("horiz", po::value<int>(&board_w)->required(), "Number of horizontal corners")
      ("vert", po::value<int>(&board_h)->required(), "Number of vertical corners")
      ("footage,f", po::value<string>(&footage_fn)->required(), "Footage file name")
      ("calibfn,f", po::value<string>(&calib_fn)->default_value("stereocalib.yml"), "calibration filename");

    po::positional_options_description positionalOptions; 
    positionalOptions.add("poses", 1);
    positionalOptions.add("horiz", 1);
    positionalOptions.add("vert", 1);
    positionalOptions,add("footage", 1);

    po::variables_map vm;

    // Parse command line arguments
    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

      if ( vm.count( "help" ) )
      {
        cout << "This is the calibration software for the gopro camera. " << endl << "To exit the program press <esc>." << endl << endl << desc << endl;

        return 0;
      }
      
      po::notify(vm);
    }
    catch ( po::error& e )
    {
      cerr << "ERROR: " << e.what() << endl << endl;
      cerr << desc << endl;
      return 1;
    }

    // Simplification of variables
    Size board_sz = Size(board_w, board_h);
    int board_n = board_w * board_h;

    // location of the corners in 3D.
    vector<vector<Point3f> > objectPoints; 
    // Default filler for object points
    vector<Point3f> obj;
    for ( int j = 0; j < board_n; ++j )
    {
      obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

    // location of the detected corners in the image.
    vector<vector<Point2f> > imagePoints; 
    
    // corners found from chessboard calibration
    vector<Point2f> corners;

    Mat src, calib;

    char key; // Escape key
    int success = 0; // number of successful chessboard captures

    // Initialize the cameras
    VideoCapture capture(footage_fn);

    // Get calibration images
    while( success < num_boards ) 
    {
      // Get camera data
      capture >> src;

      // resize image
      resize(src, src, Size(), 0.4, 0.4);
      
      // Convert to b/w
      cvtColor(src, calib, CV_BGR2GRAY);

      // Default OpenCV chessboard corner finders
      bool found = findChessboardCorners(calib, board_sz, corners,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

      if ( found)
      {
        cornerSubPix(calib, corners, Size(11,11), Size(-1,-1),
            TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(calib, board_sz, corners, found);
      }

      // Display images
      imshow("calib", calib);

      key = waitKey(10);
      // Pause for potential calibration image
      if ( found )
      {
        key = waitKey(0);
      }
      
      // Accept calibration image
      if ( key == ' ' && found != 0 )
      {
        imagePoints.push_back(corners);
        objectPoints.push_back(obj);
        success++;
        cout << success << " poses with corners stored." << endl;
      }
      // If something went wrong with calibration
      if ( char(key) == 27 )
      {
        return 0;
      }
    }

    destroyAllWindows();

    cout << "Starting Calibration..." << endl;

    // Intrinsic camera matrices
    Mat intrinsic = Mat(3, 3, CV_64FC1);

    // Distortion coefficient matrices
    Mat distcoeffs;

    // rotation and translation matrices
    vector<Mat> rvecs, tvecs;

    intrinsic.at<float>(0,0) = 1;
    intrinsic.at<float>(1,1) = 1;

    calibrateCamera(objectPoints, imagePoints, src.size(), intrinsic, distcoeffs, rvecs, tvecs);

    FileStorage fs(calib_fn, FileStorage::WRITE);
    fs << "intrinsic" << intrinsic;
    fs << "distcoeffs" << distcoeffs;

    cout << "Done Calibration." << endl;

    cout << "Starting Rectification..." << endl;

    // Undistortion matrix
    Mat undistort_mat;
    while ( true )
    {
      // Get camera data
      capture >> src;

      // resize image
      resize(src, src, Size(), 0.4, 0.4);
      
      // undistort image
      undistort(src, undistort_mat, intrinsic, distcoeffs);

      // Display undistorted image
      imshow("undistort", undistort_mat);

      // Wait for <esc> key to exit
      key = waitKey(10);
      if ( char(key) == 27 )
      {
        break;
      }

    }

    capture.release();
  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }

  return 0;
} // main
