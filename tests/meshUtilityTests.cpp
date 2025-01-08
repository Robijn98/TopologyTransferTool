#include <gtest/gtest.h>
#include "meshUtility.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Polygon_mesh;

//asked chatgpt to write test for code by feeding it code and saying 
//'can you write me a test for this code'
// and then adjusted to actually work

TEST(MeshUtility, ComputeBarycentricCoordinates)
{
    // Create a simple triangle mesh (for testing)
    Polygon_mesh polygon;
    
    // Add vertices (triangle)
    auto v0 = polygon.add_vertex(Point(0, 0, 0));
    auto v1 = polygon.add_vertex(Point(1, 0, 0));
    auto v2 = polygon.add_vertex(Point(0, 1, 0));

    // Add face (the triangle made from v0, v1, v2)
    std::vector<Polygon_mesh::Vertex_index> face_vertices = {v0, v1, v2};
    polygon.add_face(face_vertices);

    // Create a map of triangles (triangle name -> triangle points)
    std::map<std::string, std::array<Point, 3>> triangles;
    triangles["Triangle1"] = {Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0)};

    // Add additional points inside the triangle for testing
    polygon.add_vertex(Point(0.25, 0.25, 0)); // Inside Triangle1
    polygon.add_vertex(Point(0.75, 0.25, 0)); // Inside Triangle1

    // Vector to store the barycentric coordinates
    std::vector<std::array<double, 3>> barycentric_coordinates;

    // Create meshUtility object and call computeBarycentric_coordinates
    meshUtility meshUtil;
    meshUtil.computeBarycentric_coordinates(polygon, triangles, barycentric_coordinates);

    // Verify the barycentric coordinates
    ASSERT_EQ(barycentric_coordinates.size(), 5);

    // Barycentric coordinates for the first point (0.25, 0.25, 0)
    EXPECT_NEAR(barycentric_coordinates[3][0], 0.5, 1e-6);
    EXPECT_NEAR(barycentric_coordinates[3][1], 0.25, 1e-6);
    EXPECT_NEAR(barycentric_coordinates[3][2], 0.25, 1e-6);

    // Barycentric coordinates for the second point (0.75, 0.25, 0)
    EXPECT_NEAR(barycentric_coordinates[4][0], 0.0, 1e-6);
    EXPECT_NEAR(barycentric_coordinates[4][1], 0.75, 1e-6);
    EXPECT_NEAR(barycentric_coordinates[4][2], 0.25, 1e-6);

}

