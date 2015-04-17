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

// Defines
#define SCALE 0.4

// namespaces
using namespace std;
using namespace cv;
namespace po = boost::program_options;

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
  string fn, out_dir;
  int num_x, num_y;
  int key = 0;
  try
  {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "Print help messages")
      ("footage,f", po::value<string>(&fn)->required(), "footage file")
      ("numx,x", po::value<int>(&num_x)->required(), "number of x splits")
      ("numy,y", po::value<int>(&num_y)->required(), "number of y splits")
      ("output,o", po::value<string>(&out_dir)->default_value("."), "output directory");

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

    VideoCapture capture(fn);
    Mat src;
    map <string, VideoWriter> writers;
    Size sz = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH)/num_x, (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT)/num_y);

    for ( int ny = 0; ny < num_y; ++ny )
    {
      for ( int nx = 0; nx < num_x; ++nx )
      {
        stringstream out_key;
        out_key << ny << "-" << nx ;
        VideoWriter writer;
        string out_fn = out_dir + "/" + out_key.str() + ".avi"; 
        writer.open(out_fn, CV_FOURCC('m','p','4','v'), capture.get(CV_CAP_PROP_FPS), sz, true);
        writers[out_key.str()] = writer;
      }
    }
    VideoWriter tmp;
    tmp.open("temp.avi", CV_FOURCC('m','p','4','v'), capture.get(CV_CAP_PROP_FPS), sz, true);


    int max_frames = capture.get(CV_CAP_PROP_FRAME_COUNT);
    int k = 0;

    cout << "Splitting:" << endl;
    while ( k < max_frames - 1 )
    {
      capture >> src;
      if ( src.data == NULL )
      {
        break;
      }
      int rect_width = src.cols / num_x;
      int rect_height = src.rows / num_y;
      map <string, Mat> splits;

      int nx = 0;
      int ny = 0;
      for ( int y = 0; y < src.rows; y += rect_height )
      {
        for ( int x = 0; x < src.cols; x += rect_width )
        {
          Rect rect = Rect( x, y, rect_width, rect_height);
          stringstream key;
          key << ny << "-" << nx ;
          Mat sub_mat = Mat(src, rect);
          writers[key.str()] << sub_mat;
          nx++;
        }
        ny++;
        nx = 0;
      }

      // hit <esc> to exit
      key = waitKey(1);
      if ( char(key) == 27 )
      {
        break;
      }
      disp_progress((float)k/(max_frames-1), 50);
      k++;
    }
    cout << endl;
    
  }
  catch ( exception& e )
  {
    cerr << "Unhandled Exception reached the top of main: " << e.what() << ", application will now exit." << endl;
    return 2;
  }
  return 0;
}
