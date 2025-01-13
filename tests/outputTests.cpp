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
    std::map<std::string, Point> WrappedPoints;
    WrappedPoints["point1"] = Point(1, 2, 3);
    for(const auto &[key, point] : WrappedPoints)
    {
        std::cout << key << " " << point << "\n";
    }

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
}