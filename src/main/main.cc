#include <iostream>
#include "src/lib/trojanmap.h"

int main() {
  TrojanMap x;
  x.CreateGraphFromCSVFile();
  x.PrintMenu();
  return 0;
  //bazel test tests:trojanmap_test
}