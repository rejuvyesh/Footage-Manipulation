// OpenCV
#include <opencv2/opencv.hpp>

// Boost includes
#include <boost/program_options.hpp>

// General C++ includes
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

// Defines
#define SCALE 0.4

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

int main( int argc, char **argv ) {
  string fn, output_fn;
  int smoothing_rad, horizon_crop;
  int key = 1;
  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("srad,s", po::value<int>(&smoothing_rad)->default_value(20), "Smoothing Radius, the larger the more stable the video, but less reactive to sudden panning")
      ("hcrop,c", po::value<int>(&horizon_crop)->default_value(30), "Horizontal Border Crop, crops the border to reduce the black borders from stabilisation being too noticeable.")
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

    VideoCapture capture(fn);
    Mat curr, curr_grey;
    Mat prev, prev_grey;

    do
    {
      capture >> prev;
    }
    while ( prev.data == NULL );
    cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

    // Get previous to current frame transformation (dx, dy, da), for all frames
    vector<TransformParam> prev_to_curr_transform;
    int max_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);
    Mat last_T;

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
      
      // vector from prev to curr
      vector <Point2f> prev_corner, curr_corner;
      vector <Point2f> prev_corner2, curr_corner2;
      vector <uchar> status;
      vector <float> err;

      goodFeaturesToTrack(prev_grey, prev_corner, 200, 0.01, 30);
      calcOpticalFlowPyrLK(prev_grey, curr_grey, prev_corner, curr_corner, status, err);
      
      // weed out bad matches
      for ( int i = 0; i < status.size(); ++i )
      {
        if ( status[i] )
        {
          prev_corner2.push_back(prev_corner[i]);
          curr_corner2.push_back(curr_corner[i]);
        }
      }

      Mat T = estimateRigidTransform(prev_corner2, curr_corner2, false);

      if ( T.data == NULL )
      {
        last_T.copyTo(T);
      }

      T.copyTo(last_T);

      // decompose T
      double dx = T.at<double>(0,2);
      double dy = T.at<double>(1,2);
      double da = atan2(T.at<double>(1,0), T.at<double>(0,0));

      prev_to_curr_transform.push_back(TransformParam(dx, dy, da));

      curr.copyTo(prev);
      curr_grey.copyTo(prev_grey);
      disp_progress((float)k/(max_frames-1), 50);
      k++;
    }

    double a = 0;
    double x = 0;
    double y = 0;

    vector <Trajectory> traj;

    for ( int i = 0; i < prev_to_curr_transform.size(); ++i )
    {
      x += prev_to_curr_transform[i].dx;
      y += prev_to_curr_transform[i].dy;
      a += prev_to_curr_transform[i].da;

      traj.push_back(Trajectory(x,y,a));
    }

    vector <Trajectory> smoothed_traj;

    for ( int i = 0; i < traj.size(); ++i )
    {
      double sum_x = 0;
      double sum_y = 0;
      double sum_a = 0;
      int count = 0;

      for ( int j = -smoothing_rad; j <= smoothing_rad; ++j )
      {
        if ( i+j >= 0 && i+j < traj.size() )
        {
          sum_x += traj[i+j].x;
          sum_y += traj[i+j].y;
          sum_a += traj[i+j].a;

          count++;
        }
      }

      double avg_a = sum_a / count;
      double avg_x = sum_x / count;
      double avg_y = sum_y / count;

      smoothed_traj.push_back(Trajectory(avg_x, avg_y, avg_a));
    }
    
    // Generate a new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
    vector <TransformParam> new_prev_to_curr_transform;

    a = 0;
    x = 0;
    y = 0;

    for ( int i = 0; i < prev_to_curr_transform.size(); ++i )
    {
      x += prev_to_curr_transform[i].dx;
      y += prev_to_curr_transform[i].dy;
      a += prev_to_curr_transform[i].da;

      double diff_x = smoothed_traj[i].x - x;
      double diff_y = smoothed_traj[i].y - y;
      double diff_a = smoothed_traj[i].a - a;

      double dx = prev_to_curr_transform[i].dx + diff_x;
      double dy = prev_to_curr_transform[i].dy + diff_y;
      double da = prev_to_curr_transform[i].da + diff_a;
      
      new_prev_to_curr_transform.push_back(TransformParam(dx, dy, da));
    }

    capture.release();

    // Apply the new transform to the vid
    VideoCapture capture2(fn);
    //capture.set(CV_CAP_PROP_POS_FRAMES, 0);
    VideoWriter writer;
    writer.open(output_fn, CV_FOURCC('m','p','4','v'), capture2.get(CV_CAP_PROP_FPS), Size((int) capture2.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture2.get(CV_CAP_PROP_FRAME_HEIGHT)), true);
    Mat T(2,3,CV_64F);

    int vert_border = horizon_crop * prev.rows / prev.cols;

    k = 0;
    cout << endl << "Writing" << endl;
    while ( k < max_frames - 1 )
    {
      capture2 >> curr;

      if ( curr.data == NULL )
      {
        cout << "Corrupt data captured." << endl;
        break;
      }

      T.at<double>(0,0) = cos( new_prev_to_curr_transform[k].da );
      T.at<double>(0,1) = -sin( new_prev_to_curr_transform[k].da );
      T.at<double>(1,0) = sin( new_prev_to_curr_transform[k].da );
      T.at<double>(1,1) = cos( new_prev_to_curr_transform[k].da );

      T.at<double>(0,2) = new_prev_to_curr_transform[k].dx;
      T.at<double>(1,2) = new_prev_to_curr_transform[k].dy;

      Mat curr2;
      warpAffine( curr, curr2, T, curr.size() );

      curr2 = curr2( Range(vert_border, curr2.rows-vert_border), Range(horizon_crop, curr2.cols-horizon_crop) );

      resize( curr2, curr2, curr.size() );

      writer << curr2;

      //Uncomment to show.
      /*imshow("Stabilized", curr2);
      key = waitKey(10);
      if ( char(key) == 27 )
      {
        break;
      }*/

      disp_progress((float)k/(max_frames-1), 50);
      ++k;
    }
    cout << endl;
    capture2.release();

  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }

  return 0;
} // main
