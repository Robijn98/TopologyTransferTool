#ifndef MESH_H_
#define MESH_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <ngl/Transformation.h>
#include <ngl/NGLInit.h>

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
typedef Kernel::Vector_3 Vector_3;


class Mesh
{
public:
    //load mesh from obj file
    bool loadMesh(std::string filename, Polygon_mesh &polygon);
    //validate mesh, makes sure it is a manifold mesh with either quads or triangles
    bool validateMesh(Polygon_mesh &polygon);
    //checks if mesh if triangulated if not triangulates it
    void triangulateMesh(Polygon_mesh &polygon);
    //assigns colors to the vertex of the mesh and saves to a txt file
    void assignColors(Polygon_mesh &polygon, std::string outputFile);
    //reads the colors from the txt file
    std::vector<std::array<float, 3>> getVertexColors(const std::string &filename);
    //interleaves the position and color of the vertices and returns them into a format that the viewer accepts
    std::vector<ngl::Vec3> interleavePosAndColor(Polygon_mesh &polygon, std::vector<std::array<float, 3>> &vertexColors);
private:
    std::string filename;
};

#endif 

