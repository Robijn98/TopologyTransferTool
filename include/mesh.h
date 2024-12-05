#ifndef MESH_H_
#define MESH_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>


#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;
typedef boost::graph_traits<Polygon_mesh>::vertex_descriptor vertex_descriptor;

class Mesh
{
public:
    bool loadMesh(std::string filename, Polygon_mesh &polygon);
    bool validateMesh(Polygon_mesh &polygon);
private:
    std::string filename;
};

#endif 

