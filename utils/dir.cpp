#include <vector>
#include <string>
#include <sys/stat.h>
#include <dirent.h>

using namespace std;

vector<string> searchDir (string path) {
  struct dirent **namelist = NULL;
  int dirNum = scandir(path.c_str(), &namelist, NULL, NULL);
  if (dirNum == -1) {
    return vector<string>();
  }
  struct stat stat_buf;
  vector<string> ret;
  for (int i = 0; i < dirNum; i++) {
    string search_path = path + namelist[i]->d_name;
    if (stat(search_path.c_str(), &stat_buf) == 0) {
      if ((stat_buf.st_mode & S_IFMT) == S_IFDIR) continue;
      ret.push_back(string(namelist[i]->d_name));
    }
  }
  return ret;
}