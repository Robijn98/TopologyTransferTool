#include "mesh.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <iterator>

bool Mesh::loadMesh(std::string filename, Polygon_mesh &polygon)
{
    if(std::filesystem::is_empty(filename))
  {
    std::cerr << "Empty file." << std::endl;
    return 0;
  }

  //create temporary file to store mesh without line
  std::string tempFile = "temp.obj";
  std::ifstream infile(filename);

  std::ofstream temp(tempFile);

  std::string line;

  while(std::getline(infile, line))
  {
    std::istringstream iss(line);
    std::string prefix;
    iss >> prefix;
    if(prefix != "l")
    {
      temp << line << "\n";
    }
  }
  infile.close();
  temp.close();

  //load mesh from file using CGAL
  if(!PMP::IO::read_polygon_mesh(tempFile, polygon))
  {
    std::cerr << "Invalid input." << std::endl;
    std::remove(tempFile.c_str());
    return 0;
  }

  //remove temporary file
  std::remove(tempFile.c_str());

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

  if (counter > 0)
  {
    std::cout << "NON MANIFOLD" << std::endl;
    return 0;
  }

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

void Mesh::writeMesh(std::string filename, Polygon_mesh &polygon)
{
  if (filename.empty())
  {
    std::cout << "new file" << std::endl;
    std::ofstream{filename};
  }
  CGAL::IO::write_OBJ(filename, polygon);

}

void Mesh::triangulateMesh(Polygon_mesh &polygon)
{
  if(!CGAL::is_triangle_mesh(polygon))
      {
      std::cout << "Input mesh is not triangulated." << std::endl;
      PMP::triangulate_faces(polygon);
      std::cout << "Mesh triangulated." << std::endl;
      }
  else
    std::cout << "Input mesh is already triangulated." << std::endl;  
 }
 
