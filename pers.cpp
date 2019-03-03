#include <opencv2/opencv.hpp>
#include "utils/keycode.hpp"
#include "utils/dir.cpp"

using namespace std;
using namespace cv;

function<void(int x, int y)> f = [](int x, int y) { cout << x << " " << y << endl; };
int mouseX = 0, mouseY = 0;

void cb(int e, int x, int y, int flags, void *params) {
  if (e == EVENT_LBUTTONDOWN) {
    f(x, y);
  } else if (e == EVENT_MOUSEMOVE) {
    mouseX = x;
    mouseY = y;
  }
}

int main (int argc, char *argv[]) {
  vector<string> args;
  for (int i = 1; i < argc; i++) args.push_back(argv[i]);
  if (args.size() < 1) {
    return 1;
  }

  vector<string> files = searchDir(args[0]);
  sort(files.begin(), files.end());

  int id = 0;
  Mat img, view;
  img = imread(args[0] + files[id]);
  view = img.clone();

  vector<Point2f> pos, out;
  out.push_back(Point2f(0, 0));
  out.push_back(Point2f(704, 0));
  out.push_back(Point2f(0, 960));
  out.push_back(Point2f(704, 960));
  f = [&pos, &img, &view](int x, int y) {
    if (pos.size() > 3) {
      pos.erase(pos.begin());
    }
    pos.push_back(Point2f(x, y));
    view = img.clone();
    Scalar color(255., 255., 255.);
    for (auto &p : pos) {
      circle(view, p, 30., color);
    }
    imshow("window", view);
  };
  namedWindow("window", 1);
  imshow("window", view);
  setMouseCallback("window", cb);

  auto conv = [&]() {
    Mat rmat = getPerspectiveTransform(pos, out);
    Mat result(960, 704, img.type());
    warpPerspective(img, result, rmat, Size(704, 960));
    imwrite(args[0] + "_" + files[id], result);
  };

  while (true) {
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
      case KEYCODE_T:
        conv();
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