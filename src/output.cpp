#include "output.h"

#include <cstdio>
#include <filesystem>

void Output::writeMesh(std::string filename, Polygon_mesh &polygon)
{
  if (filename.empty())
  {
    std::cout << "new file\n";
    std::ofstream{filename};
  }
  CGAL::IO::write_OBJ(filename, polygon);

}

