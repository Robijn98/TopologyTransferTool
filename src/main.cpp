#include <iostream>
#include "mesh.h"



int main()
{
    Mesh mesh;
    Polygon_mesh polygon;
    std::string filename = "../test_files/deformed_sphere_export.obj";
    mesh.loadMesh(filename, polygon);
    mesh.validateMesh(polygon);
    
}