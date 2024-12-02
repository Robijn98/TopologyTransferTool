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

