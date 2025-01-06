#include <gtest/gtest.h>
#include "meshUtility.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Polygon_mesh;

//asked chatgpt to write test for code by feeding it code and saying 
//'can you write me a test for this code'
// and then adjusted to actually work

// TEST(MeshUtility, ComputeBarycentricCoordinates)
// {
//     // Create a simple triangle mesh (for testing)
//     Polygon_mesh polygon;
    
//     // Add vertices (triangle)
//     auto v0 = polygon.add_vertex(Point(0, 0, 0));
//     auto v1 = polygon.add_vertex(Point(1, 0, 0));
//     auto v2 = polygon.add_vertex(Point(0, 1, 0));

//     // Add face (the triangle made from v0, v1, v2)
//     std::vector<Polygon_mesh::Vertex_index> face_vertices = {v0, v1, v2};
//     polygon.add_face(face_vertices);

//     // Create a reference curve (points where we want to compute barycentric coordinates)
//     std::vector<Point> curveRef = {Point(0.25, 0.25, 0), Point(0.75, 0.25, 0)};

//     // Vector to store the barycentric coordinates
//     std::vector<double> barycentric_coordinates;

//     // Create meshUtility object and call computeBarycentric_coordinates
//     meshUtility meshUtil;
//     meshUtil.computeBarycentric_coordinates(polygon, barycentric_coordinates);

//     // For the test, just print the coordinates to verify
//     std::cout << "Barycentric Coordinates for the first point (0.25, 0.25, 0): ";
//     std::cout << barycentric_coordinates[0] << ", " << barycentric_coordinates[1] << ", " << barycentric_coordinates[2] << std::endl;

//     std::cout << "Barycentric Coordinates for the second point (0.75, 0.25, 0): ";
//     std::cout << barycentric_coordinates[3] << ", " << barycentric_coordinates[4] << ", " << barycentric_coordinates[5] << std::endl;
// }

