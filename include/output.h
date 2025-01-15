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
    void writeMesh(std::string filename, Polygon_mesh &polygon);
    void updateFileWithWrap(std::string filename, std::map<int, std::vector<Point>> WrappedPoints);
};  


#endif