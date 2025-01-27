#include <gtest/gtest.h>
#include "meshUtility.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include "mesh.h"
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Polygon_mesh;


TEST(MeshUtility, DivideMeshForBarycentricComputing)
{
    Mesh mesh;

    Polygon_mesh polygonSource;
    Polygon_mesh ocatohedron;
    
    std::string filenameSource = "files/sphere.obj";

    mesh.loadMesh(filenameSource, polygonSource);

    std::map<std::string, std::array<Point, 3>> trianglesSource;

    meshUtility meshUtil;
    trianglesSource = meshUtil.divideMeshForBarycentricComputing(polygonSource, ocatohedron, 1e-3, 1e-3);

    EXPECT_EQ(trianglesSource.size(), 8);

}


TEST(MeshUtility, ComputeBarycentricCoordinates)
{
    // Create a simple triangle mesh
    Polygon_mesh polygon;
    Polygon_mesh octahedron;

    // Add vertices for the triangle
    auto v0 = polygon.add_vertex(Point(0, 0, 0));
    auto v1 = polygon.add_vertex(Point(1, 0, 0));
    auto v2 = polygon.add_vertex(Point(0, 1, 0));

    // Add the face (triangle made from v0, v1, v2)
    std::vector<Polygon_mesh::Vertex_index> faceVertices = {v0, v1, v2};
    polygon.add_face(faceVertices);

    // Add additional test points inside the triangle
    auto testPoint1 = polygon.add_vertex(Point(0.25, 0.25, 0));
    auto testPoint2 = polygon.add_vertex(Point(0.75, 0.25, 0));

    // Create a simple mesh to test against
    auto v0_octo = octahedron.add_vertex(Point(0, 0, 0));
    auto v1_octo = octahedron.add_vertex(Point(1, 0, 0));
    auto v2_octo = octahedron.add_vertex(Point(0, 1, 0));

    std::vector<Polygon_mesh::Vertex_index> faceOctaVertices = {v0_octo, v1_octo, v2_octo};
    octahedron.add_face(faceOctaVertices);

    //empty triangles
    std::map<std::string, std::array<Point, 3>> triangles;

    // Map to store the barycentric coordinates
    std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> barycentricCoordinates;

    // Create a meshUtility object and compute barycentric coordinates
    meshUtility meshUtil;
    meshUtil.computeBarycentricCoordinates(polygon, octahedron, triangles, barycentricCoordinates);


    // Verify the barycentric coordinates map has an entry for "f0"
    ASSERT_EQ(barycentricCoordinates.size(), 1);

    const auto &coordsVector = barycentricCoordinates["0"];
    ASSERT_EQ(coordsVector.size(), 5); 

    std::array<double, 3> baryCoords1, baryCoords2;
    double distance1, distance2;


    // Find the barycentric coordinates for the test points
    for (const auto &[vertexID, baryCoords, distance] : coordsVector) {
        if (vertexID == 3) {
            baryCoords1 = baryCoords;
            distance1 = distance;
        } else if (vertexID == 4) {
            baryCoords2 = baryCoords;
            distance2 = distance;
        }
    }

    // Validate barycentric coordinates for the first point (0.25, 0.25, 0)
    EXPECT_NEAR(baryCoords1[0], 0.5, 1e-6);
    EXPECT_NEAR(baryCoords1[1], 0.25, 1e-6);
    EXPECT_NEAR(baryCoords1[2], 0.25, 1e-6);
    EXPECT_NEAR(distance1, 0.0, 1e-6);

    // Validate barycentric coordinates for the second point (0.75, 0.25, 0)
    EXPECT_NEAR(baryCoords2[0], 0.0, 1e-6);
    EXPECT_NEAR(baryCoords2[1], 0.75, 1e-6);
    EXPECT_NEAR(baryCoords2[2], 0.25, 1e-6);
    EXPECT_NEAR(distance2, 0.0, 1e-6); 
}

