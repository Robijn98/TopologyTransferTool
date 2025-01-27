/// header file encapsulates the struct Output, which is used to write the mesh to an obj file and update the obj file with the wrapped points.
/// author: Robin van den Eerenbeemd
/// version: 1.0
/// date: 27 jan 2025 updated to NCCA coding standards

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;

struct Output
{
    //writes mesh to an obj file
    void writeMesh(std::string filename, Polygon_mesh &polygon);
    //updates the obj file with the wrapped points
    void updateFileWithWrap(std::string filename, std::map<int, std::vector<Point>> WrappedPoints);
};  


#endif //_OUTPUT_H_