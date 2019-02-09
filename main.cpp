#include <opencv2/opencv.hpp>
#include "utils/keycode.hpp"

using namespace std;
using namespace cv;

class Pic {
  Mat raw, mask, corner, binary, view;
  int blockSize, ksize, w, h;
  double k, thd;
  bool mode;
  vector<Point2d> pending;
  vector<vector<Point2d>> points;
  void drawCircle() {
    view = binary.clone();
    Scalar color(255., 255., 255.);
    for (auto &v : points) {
      for (auto &p : v) {
        circle(view, p, 30, color);
      }
    }
  }
  void binarize() {
    threshold(corner, binary, thd, 1., THRESH_BINARY);
    drawCircle();
  }
  void findCorner() {
    cornerHarris(mask, corner, blockSize, ksize, k);
    binarize();
  }
public:
  Pic (string filename, int _blockSize = 4, int _ksize = 5, double _k = 0.04, double _thd = 0.2) : blockSize(_blockSize), ksize(_ksize), k(_k), thd(_thd) {
    mode = false;
    string rawfile = "raws/" + filename, maskfile = "masks/" + filename;
    raw = imread(rawfile);
    mask = imread(maskfile, 0);
    w = raw.cols;
    h = raw.rows;
    cout << w << " " << h << endl;
    thd = min(1., max(thd, 0.));
    findCorner();
  }
  void show(string winname) {
    if (mode) {
      imshow(winname, raw);
    } else {
      imshow(winname, view);
    }
  }
  void addThd(double e) {
    thd = min(1., max(thd + e, 0.));
    binarize();
  }
  void addBlockSize(int e) {
    blockSize = max(blockSize + e, 1);
    findCorner();
  }
  void addKSize(int e) {
    if (e % 2 != 0) return;
    ksize = min(max(ksize + e, 1), 31);
    findCorner();
  }
  void addK(double e) {
    k = max(k + e, 0.);
    findCorner();
  }
  void printParam() {
    cout << "blockSize: " << blockSize << endl;
    cout << "ksize: " << ksize << endl;
    cout << "k: " << k << endl;
    cout << "thd: " << thd << endl;
  }
  void changeMode() {
    mode = !mode;
  }
  void addPoint(int x, int y) {
    if (pending.size() == 2) {
      pending.erase(pending.begin());
    }
    pending.push_back(Point2d(x, y));
  }

  Point2d findPoint(Point2d p, double r = 30.) {
    int ox = p.x, oy = p.y;
    int l = r;
    int sumx = 0, sumy = 0, cnt = 0;
    for (int i = -l; i <= l; i++) for (int j = -l; j <= l; j++) {
      int x = ox + i, y = oy + j;
      if (x < 1 || x > w || y < 1 || y > h) continue;
      if (i * i + j * j > r * r) continue;
      cout << binary.data[y * binary.step + x * binary.elemSize()] << endl;
      break;
    }
    return p;
  }
  void calcPoints() {
    cout << pending.size() << endl;
    if (pending.size() < 2) {
      return;
    }
    vector<Point2d> v;
    v.push_back(findPoint(pending[0]));
  }
};

function<void(int x, int y)> f;

void cb(int e, int x, int y, int flags, void *params) {
  if (e == EVENT_LBUTTONDOWN) {
    f(x, y);
  }
}

int main () {
  namedWindow("window", 1);
  Pic p("0.jpg");
  f = [&p](int x, int y) {
    p.addPoint(x, y);
  };
  setMouseCallback("window", cb);
  bool loop = true;
  while (loop) {
    p.show("window");
    switch (waitKey(0)) {
      case KEYCODE_Q:
        p.addThd(0.001);
        break;
      case KEYCODE_A:
        p.addThd(-0.001);
        break;
      case KEYCODE_W:
        p.addBlockSize(1);
        break;
      case KEYCODE_S:
        p.addBlockSize(-1);
        break;
      case KEYCODE_E:
        p.addKSize(2);
        break;
      case KEYCODE_D:
        p.addKSize(-2);
        break;
      case KEYCODE_R:
        p.addK(0.001);
        break;
      case KEYCODE_F:
        p.addK(-0.001);
        break;
      case KEYCODE_SPACE:
        p.changeMode();
        break;
      case KEYCODE_ENTER:
        p.printParam();
        p.calcPoints();
        break;
      case KEYCODE_ESC:
        loop = false;
        break;
      default:
        break;
    }
  }
  return 0;
}