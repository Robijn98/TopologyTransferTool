#include "mesh.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>
#include <string>
#include <CGAL/boost/graph/iterator.h>
#include <iterator>

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


void Mesh::merge_vertices(vertex_descriptor v_keep, vertex_descriptor v_rm, Polygon_mesh& polygon)
{
  std::cout << "merging vertices " << v_keep << " and " << v_rm << std::endl;
 
  for(halfedge_descriptor h : CGAL::halfedges_around_target(v_rm, polygon))
    set_target(h, v_keep, polygon); // to ensure that no halfedge points at the deleted vertex
 
  remove_vertex(v_rm, polygon);
}
 



bool Mesh::validateMesh(Polygon_mesh &polygon)
{

  // vertex_descriptor v0 = *(vertices(polygon).begin());
  // vertex_descriptor v1 = *(--(vertices(polygon).end()));
  // merge_vertices(v0, v1, polygon);
 
  // Check if the mesh is non-manifold
  int counter = 0;
  for(vertex_descriptor v : vertices(polygon))
  {
    if(PMP::is_non_manifold_vertex(v, polygon))
    {
      //std::cout << "vertex " << v << " is non-manifold" << std::endl;
      ++counter;

    }
  }
  //std::cout<< "HERE" << std::endl;
  //std::cout << counter << " non-manifold occurrence(s)" << std::endl;

  if (counter > 0)
  {
    std::cout << "NON MANIFOLD" << std::endl;
    return 0;
  }

  // // Check if faces are non-quad or non-triangle
  // for (Polygon_mesh::Face_index f : faces(polygon)) {
  //   int num_vertices = 0;
  //   for (vertex_descriptor v : vertices_around_face(polygon.halfedge(f), polygon)) {
  //     num_vertices++;
  //   }
  //   if (num_vertices != 3 && num_vertices != 4) {
  //     std::cout << "face " << f << " is not a triangle or a quad" << std::endl;
  //     return 0;
  //   }
  // }

    return 1;
}


