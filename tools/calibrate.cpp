// OpenCV includes
#include "opencv2/features2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// #include "opencv2/contrib.hpp"

// Boost includes
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// General C++ includes
#include <iostream>
#include <string>
#include <vector>

// Defines
#define SCALE 0.2

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main( int argc, char **argv ) {

  // Argument variables
  int board_w; // num of horizontal corners
  int board_h; // num of vertical corners
  string calib_fn; // calibration file name
  string input_dir; // input directory

  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("horiz", po::value<int>(&board_w)->required(), "Number of horizontal corners")
      ("vert", po::value<int>(&board_h)->required(), "Number of vertical corners")
      ("input,i", po::value<string>(&input_dir)->required(), "Input directory")
      ("calibfn,c", po::value<string>(&calib_fn)->default_value("calib.yml"), "calibration filename");

    po::positional_options_description positionalOptions; 
    positionalOptions.add("horiz", 1);
    positionalOptions.add("vert", 1);
    positionalOptions.add("input", 1);

    po::variables_map vm;

    // Parse command line arguments
    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

      if ( vm.count( "help" ) )
      {
        cout << "This is the calibration software for the gopro camera. " << endl << endl; 
        cout << "Usage: " << argv[0] << " [options] <horiz> <vert> <input> " << endl  << endl << desc << endl;

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

    Mat src;

    char key; // Escape key
    int success = 0; // number of successful chessboard captures

    fs::directory_iterator end_iter;

    namedWindow( "Calibration", WINDOW_AUTOSIZE );
    for ( fs::directory_iterator iter(input_dir); iter != end_iter; ++iter )
    {
      // If it's not a directory, use it.
      if ( is_regular_file(iter->path()) )
      {
        string curr_img = iter->path().string();
        cout << curr_img << endl;
        src = imread(curr_img, CV_LOAD_IMAGE_GRAYSCALE);

        bool found = findChessboardCorners(src, board_sz, corners,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if ( found )
        {
          cornerSubPix(src, corners, Size(11,11), Size(-1,-1),
              TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

          drawChessboardCorners(src, board_sz, corners, found);
          imagePoints.push_back(corners);
          objectPoints.push_back(obj);
          success++;
          cout << success << " poses with corners stored." << endl;
        }
        // Resize for display
        Mat vis;
        resize(src, vis, Size(), SCALE, SCALE);
        imshow("Calibration", vis);

        // <esc> key to exit
        key = waitKey(10);
        if ( char(key) == 27 )
        {
          cout << "Exiting program. " << endl;
          return 1;
        }
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
  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }

  return 0;
} // main
