#include <vector>

namespace mat {

using namespace std;

// b = a^T
void trans (vector<vector<double>> &a, vector<vector<double>> &b) {
  int n = a.size(), m = a[0].size();
  b.clear();
  b.resize(m);
  for (int i = 0; i < m; i++) {
    b[i].resize(n);
    for (int j = 0; j < n; j++) {
      b[i][j] = a[j][i];
    }
  }
}

void mult (vector<vector<double>> &a, vector<vector<double>> &b, vector<vector<double>> &c) {
  int n = a.size(), m = b[0].size();
  assert(a[0].size() == b.size());
  int l = a[0].size();
  c.clear();
  c.resize(n);
  for (int i = 0; i < n; i++) {
    c[i].resize(m);
    for (int j = 0; j < m; j++) for (int k = 0; k < l; k++) c[i][j] += a[i][k] * b[k][j];
  }
}

void mult (vector<vector<double>> &a, vector<double> &b, vector<double> &c) {
  int n = a.size(), m = a[0].size();
  assert(a[0].size() == b.size());
  c.clear();
  c.resize(n);
  for (int i = 0; i < n; i++) for (int j = 0; j < m; j++) c[i] += a[i][j] * b[j];
}

// b = a^-1
void gaussJordan (vector<vector<double>> &a, vector<vector<double>> &b) {
  assert(a.size() == a[0].size());
  vector<vector<double>> c = a;
  int n = a.size();
  for (int i = 0; i < n; i++) {
    c[i].resize(2 * n);
    c[i][n + i] = 1;
  }
  for (int i = 0; i < n; i++) {
    double cii = c[i][i];
    for (int j = 0; j < 2 * n; j++) {
      c[i][j] /= cii;
    }
    for (int j = 0; j < n; j++) {
      if (i == j) continue;
      double cji = c[j][i];
      for (int k = 0; k < 2 * n; k++) {
        c[j][k] -= c[i][k] * cji;
      }
    }
  }
  b.clear();
  b.resize(n);
  for (int i = 0; i < n; i++) {
    b[i].resize(n);
    for (int j = 0; j < n; j++) b[i][j] = c[i][j + n];
  }
}

}