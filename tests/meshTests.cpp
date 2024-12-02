#include <gtest/gtest.h>
#include "mesh.h"

TEST(Mesh, loadMesh)
{
    Mesh mesh;
    EXPECT_TRUE(mesh.loadMesh("../test_files/deformed_sphere_export.obj"));
    EXPECT_FALSE(mesh.loadMesh("../test_file/doesntexcists.obj"));
}

