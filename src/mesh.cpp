#include "mesh.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <iterator>



bool Mesh::loadMesh(std::string filename, Polygon_mesh &polygon)
{
  //check if file is empty
  if(std::filesystem::is_empty(filename))
  {
    std::cerr << "Empty file.\n" ;
    return 0;
  }

  //create temporary file to store mesh without curve
  std::string tempFile = "temp.obj";
  std::ifstream infile(filename);

  std::ofstream temp(tempFile);

  std::string line;
  //read the file and store points that are not l lines
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
    std::cerr << "Invalid input.\n";
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
      std::cout << "vertex " << v << " is non-manifold\n";
      ++counter;

    }
  }

  if (counter > 0)
  {
    std::cout << "NON MANIFOLD\n";
    return 0;
  }

  // Check if faces are non-quad or non-triangle
  for (Polygon_mesh::Face_index f : faces(polygon)) {
    int num_vertices = 0;
    for (vertex_descriptor v : vertices_around_face(polygon.halfedge(f), polygon)) {
      num_vertices++;
    }
    if (num_vertices != 3 && num_vertices != 4) {
      std::cout << "face " << f << " is not a triangle or a quad\n";
      return 0;
    }
  }

    return 1;
}

void Mesh::triangulateMesh(Polygon_mesh &polygon)
{
  if(!CGAL::is_triangle_mesh(polygon))
      {
      std::cout << "Input mesh is not triangulated.\n";
      PMP::triangulate_faces(polygon);
      std::cout << "Mesh triangulated.\n";
      }
  else
    std::cout << "Input mesh is already triangulated.\n";  
 }
 


 void Mesh::assignColors(Polygon_mesh &polygon, std::string outputFile)
 {
  //check files
  Polygon_mesh outputMesh;
  if(!PMP::IO::read_polygon_mesh(outputFile, outputMesh))
  {
    std::cerr << "Invalid input for assigning colours.\n";
    return;
  }


  std::ofstream file("vertex_colors.txt");
  if(!file)
  {
    std::cerr << "Error opening file vertex colors\n";
    return;
  }

//find the furthest point
double furthest_distance = 0;
for(vertex_descriptor v : vertices(polygon))
{
  Point p = outputMesh.point(v);
  Point p_org = polygon.point(v);

    // Calculate the squared distance between points
    double distance = CGAL::squared_distance(p, p_org);

    // Update the furthest point if this distance is greater
    if (distance > furthest_distance) {
        furthest_distance = distance;
    }
  }



  //based on the distance from the furthest point, assign colors in steps of 0.2
  for(vertex_descriptor v : vertices(polygon))
  {
      Point p = outputMesh.point(v);
      Point p_org = polygon.point(v);


      double distance = CGAL::squared_distance(p, p_org);
      double ratio;
      if(furthest_distance < 0.0001)
      {
          ratio = 0;
      }
      else
      {
          ratio = distance / furthest_distance;
      }
      if(ratio >= 0.0 && ratio < 0.2)
      {
        CGAL::IO::Color color = CGAL::IO::Color(248, 187, 208);
        file << color << "\n";
      }
      if(ratio >= 0.2 && ratio < 0.4)
      {
        CGAL::IO::Color color = CGAL::IO::Color(240, 98, 146);
        file << color << "\n";
      }
      if(ratio >= 0.4 && ratio < 0.6)
      {
        CGAL::IO::Color color = CGAL::IO::Color(233, 30, 99);
        file << color << "\n";
      }
      if(ratio >= 0.6 && ratio < 0.8)
      {
        CGAL::IO::Color color = CGAL::IO::Color(168, 50, 50);
        file << color << "\n";
      }
      if(ratio >= 0.8 && ratio <= 1.0)
      {
        CGAL::IO::Color color = CGAL::IO::Color(123, 30, 30);
        file << color << "\n";
      }


  }


    file.close();
    std::cout << "Colors written to file.\n";
 }


std::vector<std::array<float, 3>> Mesh::getVertexColors(const std::string &filename)
{
  //open the color file
  std::ifstream file(filename);
  if(!file)
  {
    std::cerr << "Error opening file.\n";
    return {};
  }
  
  //get colors from file and store in vector
  std::vector<std::array<float, 3>> colors;
  std::string line;
  while(std::getline(file, line))
  {
    std::istringstream iss(line);
    float r, g, b;
    iss >> r >> g >> b;
    colors.push_back({r, g, b});
  }

  file.close();
  return colors;
}

std::vector<ngl::Vec3> Mesh::interleavePosAndColor(Polygon_mesh &polygon, std::vector<std::array<float, 3>> &vertexColors)
{
  std::vector<ngl::Vec3> interleaved;
  // Interleave the positions and colors and return the interleaved vector
  int i = 0;
  for (vertex_descriptor v : vertices(polygon))
  {
    Point p = polygon.point(v);
    interleaved.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
    interleaved.push_back({vertexColors[i][0]/255, vertexColors[i][1]/255, vertexColors[i][2]/255});
    i++;
  }

  return interleaved;
}