#include "mesh.h"

bool Mesh::loadMesh(std::string filename, Polygon_mesh &polygon)
{
    //load mesh from file using CGAL
  if(!PMP::IO::read_polygon_mesh(filename, polygon))
  {
    std::cerr << "Invalid input." << std::endl;
    return 0;
  }
 

    return 1;
}	





