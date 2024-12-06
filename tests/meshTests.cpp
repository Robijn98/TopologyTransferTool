#include <gtest/gtest.h>
#include "mesh.h"

TEST(Mesh, loadMesh)
{
    Mesh mesh;
    Polygon_mesh validPolygon;   
    Polygon_mesh invalidPolygon; 

    EXPECT_TRUE(mesh.loadMesh("../test_files/deformed_sphere_export.obj", validPolygon));
    EXPECT_FALSE(mesh.loadMesh("../test_files/doesntexist.obj", invalidPolygon));

}

TEST(Mesh, empty_file)
{
    Mesh mesh;
    Polygon_mesh invalidPolygon;
    EXPECT_FALSE(mesh.loadMesh("../test_files/empty.obj", invalidPolygon));

}

TEST(Mesh, non_manifold_Mesh)
{
    Mesh mesh;
    Polygon_mesh validPolygon;   
    Polygon_mesh invalidPolygon;
    mesh.loadMesh("../test_files/deformed_sphere_export.obj", validPolygon);
    mesh.loadMesh("../test_files/broken_mesh.obj", invalidPolygon);
    
    //EXPECT_TRUE(mesh.validateMesh(validPolygon));
    EXPECT_FALSE(mesh.validateMesh(invalidPolygon));
}

// TEST(Mesh, non_quad_Mesh)
// {
//     Mesh mesh;
//     Polygon_mesh validPolygon;   
//     Polygon_mesh invalidPolygon;
//     mesh.loadMesh("../test_files/deformed_sphere_export.obj", validPolygon);
//     mesh.loadMesh("../test_files/non_quad_export.obj", invalidPolygon);
    
//     EXPECT_TRUE(mesh.validateMesh(validPolygon));
//     EXPECT_FALSE(mesh.validateMesh(invalidPolygon));

// }