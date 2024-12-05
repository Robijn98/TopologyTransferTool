#include "mesh.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>
#include <string>


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
  // Check if the mesh is non-manifold
  int counter = 0;
  for(vertex_descriptor v : vertices(polygon))
  {
    if(PMP::is_non_manifold_vertex(v, polygon))
    {
      std::cout << "vertex " << v << " is non-manifold" << std::endl;
      ++counter;
    }
  }
  std::cout << counter << " non-manifold occurrence(s)" << std::endl;



//   std::map<std::pair<Polygon_mesh::Vertex_index , Polygon_mesh::Vertex_index>, int> edge_map;
//   std::set<int> unique_vertices;

//   // Iterate over each face in the polygon mesh
//   for (const auto& f : faces(polygon)) {
//       // Get the first halfedge of the face
//       std::cout<<"face: "<<f<<std::endl;

//       auto h = halfedge(f, polygon);
      
//       do {
//           // Get the source and target vertices of the edge
//           auto v1 = source(h, polygon);  
//           auto v2 = target(h, polygon); 
//           //std::cout<<"SOURCE: "<<source(h, polygon)<<std::endl;
//           //std::cout<<"TARGET: "<<target(h, polygon)<<std::endl;

//           std::pair<Polygon_mesh::Vertex_index, Polygon_mesh::Vertex_index> edge(std::min(v1, v2), std::max(v1, v2));
          
//           // Increment the count of this edge in the map
//           edge_map[edge]++;
        

//           h = next(h, polygon);

//       } while (h != halfedge(f, polygon)); 
//   }


// std::map<std::pair<Polygon_mesh::Vertex_index, Polygon_mesh::Vertex_index>, int> edgeToNumFacesMap;

// for(const auto&[edge, count] : edge_map)
// {
//   edgeToNumFacesMap[edge]++;
//   std::cout<< edgeToNumFacesMap[edge] <<std::endl;
//   std::cout << "Edge (" << edge.first << ", " << edge.second << ") is shared by " << count << " faces." << std::endl;

//   if (count > 2)
//   {
//     std::cout << "Non-manifold edge detected: " << std::endl;
//     return 0;
//   }
// }



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

    return 1;
}


