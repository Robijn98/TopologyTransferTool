#include "mesh.h"
#include <cstdio>
#include <filesystem>

bool Mesh::loadMesh(std::string filename, Polygon_mesh &polygon)
{
    //load mesh from file using CGAL, this CGAL function can take care of different file formats
  if(!PMP::IO::read_polygon_mesh(filename, polygon))
  {
    std::cerr << "Invalid input." << std::endl;
    return 0;
  }
  if(std::filesystem::is_empty(filename))
  {
    std::cerr << "Empty file." << std::endl;
    return 0;
  }
  
  return 1;

}	

bool Mesh::validateMesh(Polygon_mesh &polygon)
{
    //validate mesh using CGAL
    //https://doc.cgal.org/latest/Polygon_mesh_processing/index.html#Chapter_Manifoldness

    //check if geometry has non manifold vertex
    for(vertex_descriptor v : vertices(polygon))
    {
        if(PMP::is_non_manifold_vertex(v, polygon))
        {
            std::cout << "vertex " << v << " is non-manifold" << std::endl;
            return 0;
        }
    }

    //check if geometry has non manifold edges

  // Check if faces are non-quad or non-triangle
  for (Polygon_mesh::Face_index f : faces(polygon)) {
    int num_vertices = 0;
    for (vertex_descriptor v : vertices_around_face(polygon.halfedge(f), polygon)) {
      num_vertices++;
    }
    if (num_vertices != 3 && num_vertices != 4) {
      std::cout << "face " << f << " is not a triangle or a quad" << std::endl;
      return 0;
    }
  }

    //check if there is loose freefloating vertex 



    return 1;
}





