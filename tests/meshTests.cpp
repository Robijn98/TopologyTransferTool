#include <gtest/gtest.h>
#include "mesh.h"

// TEST(Mesh, loadMesh)
// {
//     Mesh mesh;
//     Polygon_mesh validPolygon;   

//     EXPECT_TRUE(mesh.loadMesh("files/deformed_sphere_export.obj", validPolygon));

// }

// TEST(Mesh, empty_file)
// {
//     Mesh mesh;
//     Polygon_mesh invalidPolygon;
//     EXPECT_FALSE(mesh.loadMesh("files/empty.obj", invalidPolygon));

// }

// TEST(Mesh, non_manifold_Mesh)
// {
//     Mesh mesh;
//     Polygon_mesh validPolygon;   
//     Polygon_mesh invalidPolygon;
//     mesh.loadMesh("files/deformed_sphere_export.obj", validPolygon);
//     mesh.loadMesh("files/broken_mesh.obj", invalidPolygon);
    
//     EXPECT_TRUE(mesh.validateMesh(validPolygon));
//     EXPECT_FALSE(mesh.validateMesh(invalidPolygon));
// }

// TEST(Mesh, non_quad_Mesh)
// {
//     Mesh mesh;
//     Polygon_mesh validPolygon;   
//     Polygon_mesh invalidPolygon;
//     mesh.loadMesh("files/deformed_sphere_export.obj", validPolygon);
//     mesh.loadMesh("files/non_quad_export.obj", invalidPolygon);
    
//     EXPECT_TRUE(mesh.validateMesh(validPolygon));
//     EXPECT_FALSE(mesh.validateMesh(invalidPolygon));

// }

// TEST(Mesh, writeMesh)
// {
//     Mesh mesh;
//     Polygon_mesh polygon;
//     mesh.loadMesh("files/deformed_sphere_export.obj", polygon);
//     mesh.writeMesh("files/output.obj", polygon);
//     EXPECT_TRUE("files/output.obj");
//     EXPECT_FALSE("files/outputOther.obj");
// }