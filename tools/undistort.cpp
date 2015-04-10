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

// Defines
#define SCALE 0.4

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;

int main( int argc, char **argv ) {

  string footage, calib_fn;
  int key = 0;

  try
  {
    
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("calibfn,c", po::value<string>(&calib_fn)->required(), "calibration filename")
      ("footage,f", po::value<string>(&footage)->required(), "footage file");

    po::positional_options_description positionalOptions; 
    positionalOptions.add("calibfn", 1);
    positionalOptions.add("footage", 1);

    po::variables_map vm;

    // Parse command line arguments
    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

      if ( vm.count( "help" ) )
      {
        cout << "This is the undistortion software for the gopro camera. " << endl << endl; 
        cout << "Usage: " << argv[0] << " [options] <calibfn> <footage>" << endl  << endl << desc << endl;

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

    FileStorage fs(calib_fn, FileStorage::READ);

    Mat intrinsic, distcoeffs;

    fs["intrinsic"] >> intrinsic;
    fs["distcoeffs"] >> distcoeffs;

    VideoCapture capture(footage);
    Mat src, undistorted_img;

    if ( !capture.isOpened() )
    {
      throw "Error when reading footage";
    }

    namedWindow("footage", WINDOW_AUTOSIZE);
    while ( true )
    {
      capture >> src;
      undistort(src, undistorted_img, intrinsic, distcoeffs);
      resize(undistorted_img, undistorted_img, Size(), SCALE, SCALE);
      imshow("footage", undistorted_img);
      key = waitKey(1);
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
