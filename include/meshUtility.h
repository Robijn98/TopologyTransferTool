#ifndef _MESHUTILITY_H_
#define _MESHUTILITY_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>


#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;
typedef boost::graph_traits<Polygon_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Polygon_mesh>::halfedge_descriptor               halfedge_descriptor;

class meshUtility
{
    public:
        std::map<std::string, std::array<Point, 3>> divideMeshForBarycentricComputing(Polygon_mesh &polygon, Polygon_mesh &debugMesh, double z_threshold, double y_threshold);
        
        // void clampBarycentricCoordinates(double &u, double &v, double &w, const Point &P, 
        // const Point &P0, const Point &P1, const Point &P2);
        
        void computeBarycentric_coordinates(Polygon_mesh &polygon, Polygon_mesh &octahedron ,std::map<std::string, std::array<Point, 3>> triangles, 
        std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentric_coordinates);

        std::map<int, std::vector<Point>> initialWrapping(Polygon_mesh octahedronSource, Polygon_mesh octahedronTarget, Polygon_mesh &sourceMesh,
        std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentric_coordinatesSource);

    private:

};









#endif
