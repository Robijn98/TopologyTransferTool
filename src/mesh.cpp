#include "mesh.h"

#include <fstream>
#include <iostream>
#include <string>


#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;

// https://doc.cgal.org/latest/BGL/Polygon_mesh_processing_2face_filtered_graph_example_8cpp-example.html used as reference
bool Mesh::loadMesh(std::string filename)
{
    //load mesh from file using CGAL
    //const std::string filename = CGAL::data_file_path(filename);

  Polygon_mesh polygon;
  if(!PMP::IO::read_polygon_mesh(filename, polygon))
  {
    std::cerr << "Invalid input." << std::endl;
    return 0;
  }
 

    return 1;
}	




