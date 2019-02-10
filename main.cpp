#include <opencv2/opencv.hpp>
#include "utils/keycode.hpp"
#include "utils/lsm.cpp"

using namespace std;
using namespace cv;

const int chessW = 10, chessH = 15, chessRectLeng = 64;

class Pic {
  Mat raw, mask, corner, binary, view, result;
  int blockSize, ksize, w, h;
  double k, thd;
  bool mode;
  vector<Point2d> pending, lines;
  vector<double> surfX, surfY;
  vector<vector<Point2d>> points;
  void drawLine() {
    Scalar color(255., 255., 255.);
    for (auto &l : lines) {
      line(view, Point2d(l.y, 0), Point2d(l.y + h * l.x, h), color);
    }
  }
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
    Mat floatMat;
    threshold(corner, floatMat, thd, 1., THRESH_BINARY);
    floatMat.convertTo(binary, CV_8UC1, 255);
    drawCircle();
  }
  void findCorner() {
    cornerHarris(mask, corner, blockSize, ksize, k);
    binarize();
  }
  void convSurf() {
    if (surfX.size() == 0 || surfY.size() == 0) {
      return;
    }
    double sx_c = surfX[0], sx_x = surfX[1], sx_y = surfX[2], sx_xx = surfX[3], sx_xy = surfX[4], sx_yy = surfX[5], sx_xxx = surfX[6], sx_xxy = surfX[7], sx_xyy = surfX[8], sx_yyy = surfX[9];
    double sy_c = surfY[0], sy_x = surfY[1], sy_y = surfY[2], sy_xx = surfY[3], sy_xy = surfY[4], sy_yy = surfY[5], sy_xxx = surfY[6], sy_xxy = surfY[7], sy_xyy = surfY[8], sy_yyy = surfY[9];

    result = Mat(chessH * chessRectLeng, chessW * chessRectLeng, raw.type());
    for (int i = 0; i < result.rows; i++) for (int j = 0; j < result.cols; j++) {
      double x = (double)j / chessRectLeng, y = (double)i / chessRectLeng;
      double nx = sx_c + sx_x * x + sx_y * y + sx_xx * x * x + sx_xy * x * y + sx_yy * y * y + sx_xxx * x * x * x + sx_xxy * x * x * y + sx_xyy * x * y * y + sx_yyy * y * y * y;
      double ny = sy_c + sy_x * x + sy_y * y + sy_xx * x * x + sy_xy * x * y + sy_yy * y * y + sy_xxx * x * x * x + sy_xxy * x * x * y + sy_xyy * x * y * y + sy_yyy * y * y * y;
      int lx = nx, rx = lx + 1;
      int ly = ny, ry = ly + 1;
      double dx = nx - lx, dy = ny - ly;

      for (int c = 0; c < result.channels(); c++) {
        result.data[i * result.step + j * result.elemSize() + c] =
          dx * (dy * raw.data[ly * raw.step + lx * raw.elemSize() + c] + (1 - dy) * raw.data[ry * raw.step + lx * raw.elemSize() + c]) +
          (1 - dx) * (dy * dy * raw.data[ly * raw.step + rx * raw.elemSize() + c] + (1 - dy) * raw.data[ry * raw.step + rx * raw.elemSize() + c]);
      }
    }
    imshow("result", result);
  }
public:
  Pic (string filename, int _blockSize = 10, int _ksize = 5, double _k = 0.04, double _thd = 0.03) : blockSize(_blockSize), ksize(_ksize), k(_k), thd(_thd) {
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
    cout << "lines: " << endl;
    for (auto &l : lines) {
      cout << l.x << " " << l.y << endl;
    }
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

  Point2d findPoint(Point2d p, double r = 30., double eps = 1.) {
    int ox = p.x, oy = p.y;
    int l = r;
    int sumx = 0, sumy = 0, cnt = 0;
    for (int i = -l; i <= l; i++) for (int j = -l; j <= l; j++) {
      int x = ox + i, y = oy + j;
      if (x < 1 || x > w || y < 1 || y > h) continue;
      if (i * i + j * j > r * r) continue;
      if (binary.at<uchar>(y, x) == 0) continue;
      if (binary.at<uchar>(y - 1, x) == 0) continue;
      if (binary.at<uchar>(y, x - 1) == 0) continue;
      if (binary.at<uchar>(y - 1, x - 1) == 0) continue;
      sumx += x;
      sumy += y;
      cnt++;
    }
    if (cnt > 0) {
      Point2d next((double)sumx / cnt, (double)sumy / cnt);
      if ((next.x - p.x) * (next.x - p.x) + (next.y - p.y) * (next.y - p.y) <= eps) return next;
      return findPoint(next, r, eps);
    }
    return p;
  }
  void estimateLine(vector<Point2d> &v) {
    double XY = 0, X = 0, Y = 0, YY = 0;
    for (auto &p : v) {
      XY += p.x * p.y;
      X += p.x;
      Y += p.y;
      YY += p.y * p.y;
    }
    double n = v.size();
    lines.push_back(Point2d((n * XY - X * Y) / (n * YY - Y * Y), (YY * X - XY * Y) / (n * YY - Y * Y)));
  }
  void calcPoints() {
    if (pending.size() < 2) {
      return;
    }
    vector<Point2d> v;
    Point2d st = findPoint(pending[0]), nd = findPoint(pending[1]);
    if (st == pending[0] || nd == pending[1] || st == nd) {
      pending.clear();
      return;
    }
    Point2d d = nd - st;
	d /= sqrt(d.dot(d));
	v.push_back(st);
	Point2d next = st + d * 60;
	while((next - nd).dot(next - nd) > 35 * 35) {
	  Point2d find = findPoint(next);
	  if (find.x == next.x && find.y == next.y) {
	    next += d * 10;
	    continue;
	  }
	  v.push_back(find);
	  next = st + (d.dot(find - st) + 60) * d;
	}
	v.push_back(nd);
    points.push_back(v);

    pending.clear();
    drawCircle();
  }
  void estimateSurface() {
    if (points.size() < 1) return;
    vector<vector<double>> A, At, AtA, AtAinv, AtAinvAt;
    vector<double> bx, by;
    for (int i = 0; i < points.size(); i++) {
      for (int j = 0; j < points[i].size(); j++) {
        double x = i, y = j;
        A.push_back(vector<double>({1., x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y}));
        bx.push_back(points[i][j].x);
        by.push_back(points[i][j].y);
      }
    }
    cout << "trans" << endl;
    mat::trans(A, At);
    cout << "mult" << endl;
    mat::mult(At, A, AtA);
    cout << "gauss" << endl;
    mat::gaussJordan(AtA, AtAinv);
    cout << "mult" << endl;
    mat::mult(AtAinv, At, AtAinvAt);
    cout << "mult" << endl;
    mat::mult(AtAinvAt, bx, surfX);
    mat::mult(AtAinvAt, by, surfY);

    convSurf();
  }
  void reset() {
    points.clear();
    surfX.clear();
    surfY.clear();
    lines.clear();
    findCorner();
  }
  void refitting() {
    for (auto &v : points) {
      for (auto &p : v) {
        p = findPoint(p);
      }
    }
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
  namedWindow("result", 2);
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
        p.addThd(0.005);
        break;
      case KEYCODE_A:
        p.addThd(-0.005);
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
      case KEYCODE_T:
        p.estimateSurface();
        break;
      case KEYCODE_Z:
        p.reset();
        break;
      case KEYCODE_G:
        p.refitting();
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