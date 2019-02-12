#include <opencv2/opencv.hpp>
#include "utils/keycode.hpp"
#include "utils/dir.cpp"

using namespace std;
using namespace cv;

int main (int argc, char *argv[]) {
  vector<string> args;
  for (int i = 1; i < argc; i++) args.push_back(argv[i]);
  if (args.size() < 1) {
    return 1;
  }

  vector<string> files = searchDir(args[0]);
  sort(files.begin(), files.end());

  int id = 0;
  Mat img = imread(args[0] + files[id]);
  namedWindow("window", 0);
  while (true) {
    imshow("window", img);
    bool ok = false;
    switch(waitKey(0)) {
      case KEYCODE_V:
        id = min((int)files.size() - 1, id + 1);
        img = imread(args[0] + files[id]);
        break;
      case KEYCODE_C:
        id = max(0, id - 1);
        img = imread(args[0] + files[id]);
        break;
      case KEYCODE_ESC:
        ok = true;
        break;
	  default:
	    break;
    }
    if (ok) break;
  }
  return 0;
}