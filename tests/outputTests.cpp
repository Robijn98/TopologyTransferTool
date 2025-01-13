#include <gtest/gtest.h>
#include "output.h"
#include "mesh.h"

TEST(Output, writeMesh)
{
    Mesh mesh;
    Output output;
    Polygon_mesh polygon;
    mesh.loadMesh("files/deformed_sphere_export.obj", polygon);
    output.writeMesh("files/output.obj", polygon);
    EXPECT_TRUE("files/output.obj");
}