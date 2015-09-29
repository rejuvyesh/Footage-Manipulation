// OpenCV
#include <opencv2/opencv.hpp>

// Boost includes
#include <boost/program_options.hpp>

// General C++ includes
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <thread>
#include <mutex>

// Defines
#define SCALE 0.4

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;

// global variables
mutex mtx;

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

void split_img( int split_num, int x, int y, int num_x, int num_y, int time_split, string fn, string out_dir, int overlap) {
  mtx.lock();
  cout << "worker " << x << "-" << y << "-" << split_num << " running." << endl;
  mtx.unlock();
  VideoCapture capture(fn);
  int max_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);

  int rect_width = (int) capture.get(CV_CAP_PROP_FRAME_WIDTH)/num_x;
  int rect_height = (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT)/num_y;

  int start_frame = split_num * (max_frames / time_split);

  capture.set(CV_CAP_PROP_POS_FRAMES, start_frame);

  int this_x = x*rect_width;
  int this_y = y*rect_height;
  int this_width = rect_width;
  int this_height = rect_height;

  //Determine whether to add overlap to the quadrant.
  if (x > 0){
    this_x -= overlap;
    this_width += overlap;
  }
  if (y > 0) {
    this_y -= overlap;
    this_height += overlap;
  }
  if (x != num_x-1) {
    this_width += overlap;
  }
  if (y != num_y-1) {
    this_height += overlap;
  }

  Size sz = Size(this_width, this_height);

  stringstream out_key;
  out_key << y << "-" << x << "-" << split_num;
  VideoWriter writer;
  string out_fn = out_dir + "/" + out_key.str() + ".avi"; 
  writer.open(out_fn, CV_FOURCC('m','p','4','v'), capture.get(CV_CAP_PROP_FPS), sz, true);

  Mat src;
  Rect rect = Rect( this_x, this_y, this_width, this_height );

  int k = 0;
  while ( k < (max_frames / time_split)-1 )
  {
    capture >> src;
    if ( src.data == NULL )
      break;

    writer << Mat(src, rect);

    int percent = int(100 * ((float)k/(max_frames/time_split)));
    if ( percent % 5 == 0 )
    {
      mtx.lock();
      cout << "worker " << x << "-" << y << "-" << split_num << ": " << percent << "%" << endl;
      mtx.unlock();
    }
    k++;
  }
}

int main( int argc, char **argv ) {
  string fn, out_dir;
  int num_x, num_y, time_split, kthreads, overlap;
  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("footage,f", po::value<string>(&fn)->required(), "footage file")
      ("numx,x", po::value<int>(&num_x)->required(), "number of x splits")
      ("numy,y", po::value<int>(&num_y)->required(), "number of y splits")
      ("timesplit,t", po::value<int>(&time_split)->default_value(1), "number of time splits")
      ("output,o", po::value<string>(&out_dir)->default_value("."), "output directory")
      ("threads,s", po::value<int>(&kthreads)->default_value(4), "number of threads")
      ("overlap,l", po::value<int>(&overlap)->default_value(0), "number of pixels to overlap spatially between splits");

    po::positional_options_description positionalOptions; 
    positionalOptions.add("footage", 1);
    positionalOptions.add("numx", 1);
    positionalOptions.add("numy", 1);

    po::variables_map vm;

    // Parse command line arguments
    try
    {
      po::store(po::command_line_parser(argc, argv).options(desc).positional(positionalOptions).run(), vm);

      if ( vm.count( "help" ) )
      {
        cout << "This is the video splitting software for the gopro camera. " << endl << endl; 
        cout << "Usage: " << argv[0] << " [options] <footage> <numx> <numy>" << endl  << endl << desc << endl;

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

    vector <thread> workers;
    int kspawned = 0;
    for ( int time_num = 0; time_num < time_split; ++time_num )
    {
      for ( int ny = 0; ny < num_y; ++ny )
      {
        for ( int nx = 0; nx < num_x; ++nx )
        {
          if ( kspawned > 0 && kspawned % kthreads == 0 )
          {
            workers.back().join();
            workers.pop_back();
          }
          workers.push_back(thread(split_img, time_num, nx, ny, num_x, num_y, time_split, fn, out_dir, overlap));
          kspawned++;
        }
      }
    }

    while ( workers.size() > 0 )
    {
      workers.back().join();
      workers.pop_back();
    }
    cout << endl;
    return 0;
  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }
  return 0;
}
