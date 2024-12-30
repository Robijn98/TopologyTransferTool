#ifndef _MESHUTILITY_H_
#define _MESHUTILITY_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;

class meshUtility
{
    public:
        void computeBarycentric_coordinates(Polygon_mesh &polygon, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates);
        //void laplacianSmoothing(Polygon_mesh &polygon, int iterations);
        //void wrapMesh(Polygon_mesh &polygon, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates);
    private:

};









#endif
