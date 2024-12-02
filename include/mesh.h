#ifndef MESH_H_
#define MESH_H_
#include <string>

class Mesh
{
public:
    bool loadMesh(std::string filename);
    //void convert_mesh(std::string inputMesh);
private:
    std::string filename;
};

#endif 