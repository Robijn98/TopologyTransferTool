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
    std::string filenameSource = "files/normal_sphere_export.obj";

    mesh.loadMesh(filenameSource, polygonSource);

    std::map<std::string, std::array<Point, 3>> trianglesSource;

    meshUtility meshUtil;
    trianglesSource = meshUtil.divideMeshForBarycentricComputing(polygonSource, 1e-3, 1e-3);

    EXPECT_EQ(trianglesSource.size(), 8);

}



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
    polygon.add_vertex(Point(0.25, 0.25, 0)); 
    polygon.add_vertex(Point(0.75, 0.25, 0)); 

    // Vector to store the barycentric coordinates
    std::vector<std::tuple<std::string, std::array<double, 3>, double>> barycentric_coordinates;

    // Create meshUtility object and call computeBarycentric_coordinates
    meshUtility meshUtil;
    meshUtil.computeBarycentric_coordinates(polygon, triangles, barycentric_coordinates);

    // Verify the barycentric coordinates
    ASSERT_EQ(barycentric_coordinates.size(), 5);

    // Barycentric coordinates for the first point (0.25, 0.25, 0)
    std::array<double, 3> coords_3 = std::get<1>(barycentric_coordinates[3]);

    EXPECT_NEAR(coords_3[0], 0.5, 1e-6);
    EXPECT_NEAR(coords_3[1], 0.25, 1e-6);
    EXPECT_NEAR(coords_3[2], 0.25, 1e-6);

    // Barycentric coordinates for the second point (0.75, 0.25, 0)
    std::array<double, 3> coords_4 = std::get<1>(barycentric_coordinates[4]);
    EXPECT_NEAR(coords_4[0], 0.0, 1e-6);
    EXPECT_NEAR(coords_4[1], 0.75, 1e-6);
    EXPECT_NEAR(coords_4[2], 0.25, 1e-6);

}

TEST(MeshUtility, projectPoints)
{
    Mesh mesh;

    Polygon_mesh polygonSource;
    Polygon_mesh polygonTarget;
    std::string filenameSource = "files/normal_sphere_export.obj";
    std::string filenameTarget = "files/deformed_sphere_line.obj";

    mesh.loadMesh(filenameSource, polygonSource);
    mesh.loadMesh(filenameTarget, polygonTarget);

    std::map<std::string, std::array<Point, 3>> trianglesSource;
    std::map<std::string, std::array<Point, 3>> trianglesTarget;

    meshUtility meshUtil;
    trianglesSource = meshUtil.divideMeshForBarycentricComputing(polygonSource, 1e-3, 1e-3);
    trianglesTarget = meshUtil.divideMeshForBarycentricComputing(polygonTarget, 0.1, 0.1);

    std::map<std::string, std::array<Point, 3>> projected_points;

    meshUtil.projectTrianglePoints(trianglesSource, trianglesTarget, projected_points);
    EXPECT_EQ(projected_points.size(), 8);

}


// TEST(MeshUtility, wrapping)
// {
//     // Create a simple triangle mesh (for testing)
//     Polygon_mesh polygonSource;
//     Polygon_mesh polygonTarget;
    
//     // Add vertices (triangle)
//     auto v0 = polygonSource.add_vertex(Point(0, 0, 0));
//     auto v1 = polygonSource.add_vertex(Point(1, 0, 0));
//     auto v2 = polygonSource.add_vertex(Point(0, 1, 0));

//     // Add face (the triangle made from v0, v1, v2)
//     std::vector<Polygon_mesh::Vertex_index> face_vertices = {v0, v1, v2};
//     polygonSource.add_face(face_vertices);

//     // Create a map of triangles (triangle name -> triangle points)
//     std::map<std::string, std::array<Point, 3>> triangles;
//     triangles["Triangle1"] = {Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0)};


//     // Add additional points inside the triangle for testing
//     polygonSource.add_vertex(Point(0.25, 0.25, 0)); 
//     polygonSource.add_vertex(Point(0.75, 0.25, 0)); 

//     // Vector to store the barycentric coordinates
//     std::vector<std::tuple<std::string, std::array<double, 3>, double>> barycentric_coordinates;

//     // Create meshUtility object and call computeBarycentric_coordinates
//     meshUtility meshUtil;
//     meshUtil.computeBarycentric_coordinates(polygonSource, triangles, barycentric_coordinates);

//     // Verify the barycentric coordinates
//     ASSERT_EQ(barycentric_coordinates.size(), 5);

//     // Barycentric coordinates for the first point (0.25, 0.25, 0)
//     std::array<double, 3> coords_3 = std::get<1>(barycentric_coordinates[3]);

//     EXPECT_NEAR(coords_3[0], 0.5, 1e-6);
//     EXPECT_NEAR(coords_3[1], 0.25, 1e-6);
//     EXPECT_NEAR(coords_3[2], 0.25, 1e-6);

//     //create a map of triangles for the target mesh
//     std::map<std::string, std::array<Point, 3>> trianglesTarget;
//     trianglesTarget["Triangle1"] = {Point(1, 0, 0), Point(1, 1, 0), Point(0, 1, 0)};
    
// }