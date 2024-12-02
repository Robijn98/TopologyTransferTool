#include <iostream>
#include "mesh.h"



int main()
{
    Mesh mesh;
    std::string filename = "../test_files/deformed_sphere_export.obj";
    mesh.loadMesh(filename);

    
}