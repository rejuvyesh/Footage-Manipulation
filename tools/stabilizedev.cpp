// OpenCV
#include <opencv2/opencv.hpp>

// Boost includes
#include <boost/program_options.hpp>

// General C++ includes
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

//relative files
#include "ert.h"

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;

struct TransformParam
{
  TransformParam() {}
  TransformParam( double _dx, double _dy, double _da) {
    dx = _dx;
    dy = _dy;
    da = _da;
  }

  double dx;
  double dy;
  double da; // angle
};

struct Trajectory
{
  Trajectory() {}
  Trajectory( double _x, double _y, double _a ) {
    x = _x;
    y = _x;
    a = _a;
  }
  
  double x;
  double y;
  double a; // angle
};

struct UserData
{
  UserData() {}
  UserData( Mat _img, vector<Point2f> *_corners, int _rect_size, float _scale_factor) {
    img = _img;
    corners = _corners;
    rect_size = _rect_size;
    scale_factor = _scale_factor;
  }
  Mat img;
  vector<Point2f> *corners;
  int rect_size;
  float scale_factor;
};

void disp_progress(float progress, int bar_width)
{
  cout << "[";
  int pos = bar_width * progress;
  for ( int i = 0; i < bar_width; ++i )
  {
    if ( i < pos )
    {
      cout << "=";
    }
    else if ( i == pos )
    {
      cout << ">";
    }
    else
    {
      cout << " ";
    }
  }
  cout << "] " << int(progress * 100.0) << "%\r" ;
  cout.flush();
}

void select_features_callback( int event, int x, int y, int flags, void* userdata ) {
  struct UserData ud = *static_cast<struct UserData*>(userdata);
  int rect_size = ud.rect_size;
  Mat first_grey = ud.img;
  if ( event == EVENT_LBUTTONDOWN )
  {
    vector<Point2f> corners;
    goodFeaturesToTrack(Mat( first_grey, Rect(x-(rect_size/2), y-(rect_size/2), rect_size, rect_size)), corners, 1, 0.1, 30);
    ud.corners->push_back((corners[0]+Point2f(x-(rect_size/2),y-(rect_size/2)))*(1.0/ud.scale_factor));
    circle ( first_grey, ud.corners->back()*ud.scale_factor, 4, Scalar(0, 255, 0), -1, 8, 0 );
    imshow("first", first_grey);
  }
}

int main( int argc, char **argv ) {
  string fn, output_fn;
  int box_size, horizon_crop;
  int start_frame;
  float scale_factor;
  int end_frame;
  int ransac_max_iters;
  float ransac_good_ratio;
  int reset_start;
  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("boxsize,b", po::value<int>(&box_size)->default_value(20), "The size of the box that you search for the best point to track in")
      ("hcrop,c", po::value<int>(&horizon_crop)->default_value(30), "Horizontal Border Crop, crops the border to reduce the black borders from stabilization being too noticeable.")
      ("manualframe,m", po::value<int>(&start_frame)->default_value(0), "Frame to do manual capturing on.")
      ("endframe,e", po::value<int>(&end_frame)->default_value(0), "Frame to stop stabilization at.")
      ("scalefactor,s", po::value<float>(&scale_factor)->default_value(0.25), "Scaling Factor for manual marking.")
      ("ransac_max_iters,i", po::value<int>(&ransac_max_iters)->default_value(500), "Maximum number of iterations for RANSAC.")
      ("ransac_good_ratio,g", po::value<float>(&ransac_good_ratio)->default_value(0.9), "Inlier Ratio used for RANSAC.")
      //A higher inlier ratio will force model to only estimate the affine transform using that percentage of inlier points.
      ("reset_start,r", po::value<int>(&reset_start)->default_value(2000), "Frame to reset the point selection at.")
      ("footage,f", po::value<string>(&fn)->required(), "footage file")
      ("output,o", po::value<string>(&output_fn)->default_value("output.avi"), "output file");

    po::positional_options_description positionalOptions; 
    positionalOptions.add("footage", 1);

    po::variables_map vm;

    // Parse command line arguments
    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

      if ( vm.count( "help" ) )
      {
        cout << "This is the stabilization software for the gopro camera. " << endl << endl; 
        cout << "Usage: " << argv[0] << " [options] <footage>" << endl  << endl << desc << endl;

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
    if (end_frame > 0 && end_frame < start_frame)
    {
      throw invalid_argument( "selected end frame is before start frame");
    }
    VideoCapture capturefirst(fn);
    VideoWriter writer;
    writer.open(output_fn, CV_FOURCC('m','p','4','v'), capturefirst.get(CV_CAP_PROP_FPS), Size((int) capturefirst.get(CV_CAP_PROP_FRAME_WIDTH), (int) capturefirst.get(CV_CAP_PROP_FRAME_HEIGHT)), true);
    Mat curr, curr_grey;
    Mat first, first_grey, first_grey_disp;
    int max_frames = capturefirst.get(CV_CAP_PROP_FRAME_COUNT);
    printf("Number of frames in video: %d\n",max_frames);
    printf("Reset point selection after %d frames\n",reset_start);
    if ( start_frame > max_frames )
    {
      throw invalid_argument( "start_frame larger than max frames" );
    }
    if (end_frame == 0)
    {
      end_frame = max_frames;
    }
    capturefirst.set(CV_CAP_PROP_POS_FRAMES, start_frame);
    do
    {
      capturefirst >> first;
    }
    while ( first.data == NULL );
    cvtColor(first, first_grey, COLOR_BGR2GRAY);
    resize(first_grey, first_grey_disp, Size(), scale_factor, scale_factor);
    vector <Point2f> first_corners, first_corners2;
    struct UserData ud(first_grey_disp, &first_corners, box_size, scale_factor);

    namedWindow("first", CV_WINDOW_AUTOSIZE);
    setMouseCallback("first", select_features_callback, &ud);
    imshow("first", first_grey_disp);
    waitKey(0);
    cout << first_corners.size() << " corners detected." << endl;
    destroyAllWindows();
    writer << first;

    Mat last_T;
    capturefirst.release();

    VideoCapture capture(fn);
    cout << "Analyzing" << endl;
    int k = 0;
    while ( k < max_frames - 1 )
    {
      capture >> curr; 
      if ( curr.data == NULL )
      {
        break;
      }

      cvtColor(curr, curr_grey, COLOR_BGR2GRAY);
      vector <Point2f> curr_corners, curr_corners2;
      vector <uchar> status;
      vector <float> err;

      calcOpticalFlowPyrLK(first_grey, curr_grey, first_corners, curr_corners, status, err);
      
      // weed out bad matches
      first_corners2.clear();
      for ( int i = 0; i < status.size(); ++i )
      {
        if ( status[i] )
        {
          first_corners2.push_back(first_corners[i]);
          curr_corners2.push_back(curr_corners[i]);
        }
      }

      //Mat T = estimateRigidTransform(curr_corners2, first_corners2, true);
      //Mat T = findHomography(curr_corners2, first_corners2, CV_RANSAC);
      Mat T = estimateRigidTransformRansac(curr_corners2, first_corners2, true, ransac_max_iters, ransac_good_ratio);
      if ( T.data == NULL )
      {
        last_T.copyTo(T);
      }

      T.copyTo(last_T);

      Mat currT;
      warpAffine( curr, currT, T, curr.size() );
      //warpPerspective(curr, currT, T, curr.size());

      int vert_border = horizon_crop * first.rows / first.cols;
      currT = currT( Range(vert_border, currT.rows-vert_border), Range(horizon_crop, currT.cols-horizon_crop) );
      resize(currT, currT, curr.size());
      writer << currT;


      if (k > 0 && k % reset_start == 0)
      {
        printf("\nreseting point selection at frame %d\n",k);
        Mat first, first_grey, first_grey_disp;
        cvtColor(currT, first_grey, COLOR_BGR2GRAY);
        resize(first_grey, first_grey_disp, Size(), scale_factor, scale_factor);
        vector <Point2f> first_corners, first_corners2;
        struct UserData ud(first_grey_disp, &first_corners, box_size, scale_factor);
        namedWindow("first", CV_WINDOW_AUTOSIZE);
        setMouseCallback("first", select_features_callback, &ud);
        imshow("first", first_grey_disp);
        waitKey(0);
        cout << first_corners.size() << " corners detected." << endl;
        destroyAllWindows();
      }
      disp_progress((float)k/(max_frames-1), 50);
      k++;
    }
    cout << endl;
    capture.release();

  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }

  return 0;
} // main
