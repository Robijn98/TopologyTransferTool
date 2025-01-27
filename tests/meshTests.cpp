#include <gtest/gtest.h>
#include "mesh.h"

TEST(Mesh, loadMesh)
{
    Mesh mesh;
    Polygon_mesh validPolygon;   

    EXPECT_TRUE(mesh.loadMesh("files/deformedSphere.obj", validPolygon));

}

TEST(Mesh, emptyFile)
{
    Mesh mesh;
    Polygon_mesh invalidPolygon;
    EXPECT_FALSE(mesh.loadMesh("files/empty.obj", invalidPolygon));

}


TEST(Mesh, nonQuadMesh)
{
    Mesh mesh;
    Polygon_mesh validPolygon;   
    Polygon_mesh invalidPolygon;
    mesh.loadMesh("files/deformedSphere.obj", validPolygon);
    mesh.loadMesh("files/nonQuadMesh.obj", invalidPolygon);
    
    EXPECT_TRUE(mesh.validateMesh(validPolygon));
    EXPECT_FALSE(mesh.validateMesh(invalidPolygon));

}

TEST(Mesh, triangulateMesh)
{
    Mesh mesh;
    Polygon_mesh validPolygon;   
    mesh.loadMesh("files/deformedSphere.obj", validPolygon);
    mesh.triangulateMesh(validPolygon);
    EXPECT_TRUE(mesh.validateMesh(validPolygon));

}