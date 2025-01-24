#include "output.h"

#include <cstdio>
#include <filesystem>

void Output::writeMesh(std::string filename, Polygon_mesh &polygon)
{
  //check file if file is not empty write to an obj
  if (filename.empty())
  {
    std::cout << "new file\n";
    std::ofstream{filename};
  }
  CGAL::IO::write_OBJ(filename, polygon);

}

void Output::updateFileWithWrap(std::string filename, std::map<int, std::vector<Point>> WrappedPoints)
{
  std::ifstream infile(filename);
  std::string line;

  std::string newFile = "new.obj";
  std::ofstream outfile(newFile);
  bool wrapProcessed = false;

  //read the file and store vertices
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
          
            //put all the vertices in the new file
            for(const auto &[key, point] : WrappedPoints)
            {
              for(const auto &point : point)
              {
                outfile << "v " << point.x() << " " << point.y() << " " << point.z() << "\n";
              }

            }
            wrapProcessed = true;
        }

      }

    }
    infile.close();
    outfile.close();
    
    //remove the old file and rename the new file
    std::remove(filename.c_str());
    std::rename(newFile.c_str(), filename.c_str());
  }


