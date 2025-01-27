/// header file encapsulates the class meshUtility, which is used to divide the mesh into an octohedron, 
/// calculate barycentric coordinates, wrap the mesh and relax the mesh.
/// author: Robin van den Eerenbeemd
/// version: 1.0
/// date: 27 jan 2025 updated to NCCA coding standards
/// to do : add laplacian smoother to relax mesh


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
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;
typedef boost::graph_traits<Polygon_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Polygon_mesh>::halfedge_descriptor halfedge_descriptor;

class meshUtility
{
    public:
        //creates a map of triangles, triangles are determined by dividing the mesh into an octohedron
        std::map<std::string, std::array<Point, 3>> divideMeshForBarycentricComputing(Polygon_mesh &polygon, Polygon_mesh &debugMesh, double zThreshold, double yThreshold);
        //calculates the barycentric coordinates of the mesh based on an octohedron
        void computeBarycentricCoordinates(Polygon_mesh &polygon, Polygon_mesh &octahedron ,std::map<std::string, std::array<Point, 3>> triangles, 
        std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentricCoordinates);
        //wraps the source mesh to the target mesh based on the ocothedrons for both meshes and the barycentric coordinates
        std::map<int, std::vector<Point>> initialWrapping(Polygon_mesh octahedronSource, Polygon_mesh octahedronTarget, Polygon_mesh &sourceMesh,
        std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentricCoordinatesSource);
        //relaxes the mesh 
        void relaxMesh(Polygon_mesh &polygon);     

    private:

};



#endif // _MESHUTILITY_H_
