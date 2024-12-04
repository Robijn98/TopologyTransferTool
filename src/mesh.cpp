#include "mesh.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>

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
  // for (vertex_descriptor v : vertices(polygon)) {
  //     if (PMP::is_non_manifold_vertex(v, polygon)) {
  //         std::cout << "Non-manifold vertex detected: " << v << std::endl;
  //         return 0;
  //     }
  // }





// Edge map to count edge occurrences
  std::map<std::pair<int, int>, int> edge_map;

  // Iterate over each face in the polygon mesh
  for (const auto& f : faces(polygon)) {
      // Get the first halfedge of the face
      auto h = halfedge(f, polygon);
      
      // Traverse the halfedges of the face
      do {
          // Get the source and target vertices of the edge
          auto v1 = source(h, polygon);  // Vertex 1
          auto v2 = target(h, polygon);  // Vertex 2

          // Create a pair representing the edge, sorted to avoid direction issues
          std::pair<int, int> edge(std::min(v1, v2), std::max(v1, v2));
          
          // Increment the count of this edge in the map
          edge_map[edge]++;
          
          // Move to the next halfedge
          h = next(h, polygon);

      } while (h != halfedge(f, polygon)); // Stop when we return to the first halfedge of the face
  }


  // After checking all faces, check for non-manifold edges
  for (const auto& edge_pair : edge_map) {
      std::cout<< "ENTERED FOR LOOP" << std::endl;
      
      if (edge_pair.second > 2) {
          std::cout << "Non-manifold edge detected: "
                    << edge_pair.first.first << " " << edge_pair.first.second << std::endl;
          return 0;  // Non-manifold edge detected
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





