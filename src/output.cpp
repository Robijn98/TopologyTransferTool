#include "output.h"

#include <cstdio>
#include <filesystem>

void Output::writeMesh(std::string filename, Polygon_mesh &polygon)
{
  if (filename.empty())
  {
    std::cout << "new file\n";
    std::ofstream{filename};
  }
  CGAL::IO::write_OBJ(filename, polygon);

}

void Output::updateFileWithWrap(std::string filename, std::map<std::string, Point> WrappedPoints)
{
  std::cout<< "Updating file with wrapped points\n";
  std::ifstream infile(filename);
  std::string line;

  std::string newFile = "new.obj";
  std::ofstream outfile(newFile);
  bool wrapProcessed = false;
    while(std::getline(infile, line))
    {
      std::string prefix;
      std::istringstream iss(line);
      iss >> prefix;

      if(prefix != "v")
      {
        outfile << line << "\n";
      }

      if(prefix == "v")
      {
        if(!wrapProcessed)
        {
          
            //std::cout << "found vertex\n";
            //we need to put all the vertex in the new file
            for(const auto &[key, point] : WrappedPoints)
            {
              //std::cout << key << " " << point << "\n";
              outfile << "v " << point.x() << " " << point.y() << " " << point.z() << "\n";
            }
            wrapProcessed = true;
        }

      }

    }
    infile.close();
    outfile.close();
    

    std::remove(filename.c_str());
    std::rename(newFile.c_str(), filename.c_str());
  }


