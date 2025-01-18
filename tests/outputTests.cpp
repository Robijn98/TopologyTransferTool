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


TEST(Output, updateFileWithWrap)
{
    Mesh mesh;
    Output output;
    std::map<int, std::vector<Point>>  WrappedPoints;
    WrappedPoints[1].push_back(Point(1, 2, 3));

    output.updateFileWithWrap("files/output.obj", WrappedPoints);
    //read the file and check if the point is there
    std::string line;
    std::ifstream file("files/output.obj");
    bool found = false;
    while (std::getline(file, line))
    {
        if (line.find("v 1 2 3") != std::string::npos)
        {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found == true);
    
    found = false;
    while (std::getline(file, line))
    {
        if (line.find("v 1 5 3") != std::string::npos)
        {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found == false);
}