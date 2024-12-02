#include <iostream>
#include "mesh.h"



int main()
{
    Mesh mesh;
    Polygon_mesh validPolygon;
    std::string filename = "../test_files/deformed_sphere_export.obj";
    mesh.loadMesh(filename, validPolygon);

    
}