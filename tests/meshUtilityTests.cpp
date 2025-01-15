// #include <gtest/gtest.h>
// #include "meshUtility.h"
// #include <CGAL/Simple_cartesian.h>
// #include <CGAL/Surface_mesh.h>
// #include "mesh.h"
// typedef Kernel::Point_3 Point;
// typedef CGAL::Surface_mesh<Point> Polygon_mesh;


// TEST(MeshUtility, DivideMeshForBarycentricComputing)
// {
//     Mesh mesh;

//     Polygon_mesh polygonSource;
    
//     std::string filenameSource = "files/normal_sphere_export.obj";

//     mesh.loadMesh(filenameSource, polygonSource);

//     std::map<std::string, std::array<Point, 3>> trianglesSource;

//     meshUtility meshUtil;
//     trianglesSource = meshUtil.divideMeshForBarycentricComputing(polygonSource, 1e-3, 1e-3);

//     EXPECT_EQ(trianglesSource.size(), 8);

// }


// TEST(MeshUtility, ComputeBarycentricCoordinates)
// {
//     // Create a simple triangle mesh
//     Polygon_mesh polygon;

//     // Add vertices for the triangle
//     auto v0 = polygon.add_vertex(Point(0, 0, 0));
//     auto v1 = polygon.add_vertex(Point(1, 0, 0));
//     auto v2 = polygon.add_vertex(Point(0, 1, 0));

//     // Add the face (triangle made from v0, v1, v2)
//     std::vector<Polygon_mesh::Vertex_index> face_vertices = {v0, v1, v2};
//     polygon.add_face(face_vertices);

//     // Add additional test points inside the triangle
//     auto test_point_1 = polygon.add_vertex(Point(0.25, 0.25, 0));
//     auto test_point_2 = polygon.add_vertex(Point(0.75, 0.25, 0));

//     // Create a map of triangles (triangle name -> triangle vertices)
//     std::map<std::string, std::array<Point, 3>> triangles;
//     triangles["Triangle1"] = {Point(0, 0, 0), Point(1, 0, 0), Point(0, 1, 0)};

//     // Map to store the barycentric coordinates
//     std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> barycentric_coordinates;

//     // Create a meshUtility object and compute barycentric coordinates
//     meshUtility meshUtil;
//     meshUtil.computeBarycentric_coordinates(polygon, triangles, barycentric_coordinates);

//     // Verify the barycentric coordinates map has an entry for "Triangle1"
//     ASSERT_EQ(barycentric_coordinates.size(), 1);
//     ASSERT_TRUE(barycentric_coordinates.find("Triangle1") != barycentric_coordinates.end());

//     const auto &coords_vector = barycentric_coordinates["Triangle1"];
//     ASSERT_EQ(coords_vector.size(), 5); 

//     std::array<double, 3> bary_coords_1, bary_coords_2;
//     double distance_1, distance_2;


//     // Find the barycentric coordinates for the test points
//     for (const auto &[vertex_id, bary_coords, distance] : coords_vector) {
//         if (vertex_id == 3) {
//             bary_coords_1 = bary_coords;
//             distance_1 = distance;
//         } else if (vertex_id == 4) {
//             bary_coords_2 = bary_coords;
//             distance_2 = distance;
//         }
//     }


//     // Validate barycentric coordinates for the first point (0.25, 0.25, 0)
//     EXPECT_NEAR(bary_coords_1[0], 0.5, 1e-6);
//     EXPECT_NEAR(bary_coords_1[1], 0.25, 1e-6);
//     EXPECT_NEAR(bary_coords_1[2], 0.25, 1e-6);
//     EXPECT_NEAR(distance_1, 0.0, 1e-6);

//     // Validate barycentric coordinates for the second point (0.75, 0.25, 0)
//     EXPECT_NEAR(bary_coords_2[0], 0.0, 1e-6);
//     EXPECT_NEAR(bary_coords_2[1], 0.75, 1e-6);
//     EXPECT_NEAR(bary_coords_2[2], 0.25, 1e-6);
//     EXPECT_NEAR(distance_2, 0.0, 1e-6); 
// }


// TEST(MeshUtility, projectPoints)
// {
//     Mesh mesh;

//     Polygon_mesh polygonSource;
//     Polygon_mesh polygonTarget;
//     std::string filenameSource = "files/normal_sphere_export.obj";
//     std::string filenameTarget = "files/deformed_sphere_line.obj";

//     mesh.loadMesh(filenameSource, polygonSource);
//     mesh.loadMesh(filenameTarget, polygonTarget);

//     std::map<std::string, std::array<Point, 3>> trianglesSource;
//     std::map<std::string, std::array<Point, 3>> trianglesTarget;

//     meshUtility meshUtil;
//     trianglesSource = meshUtil.divideMeshForBarycentricComputing(polygonSource, 1e-3, 1e-3);
//     trianglesTarget = meshUtil.divideMeshForBarycentricComputing(polygonTarget, 0.1, 0.1);

//     std::map<std::string, std::array<Point, 3>> projected_points;

//     meshUtil.projectTrianglePoints(trianglesSource, trianglesTarget, projected_points);
//     EXPECT_EQ(projected_points.size(), 8);

// }

