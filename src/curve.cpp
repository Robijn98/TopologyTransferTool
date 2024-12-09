#include "curve.h"
#include <cstdio>
#include <filesystem>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <vector>


bool Curve::loadCurve(std::string filename, std::vector<Point> &curve)
 {
  std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Invalid input." << std::endl;
        return 0;
    }
    if (std::filesystem::is_empty(filename))
    {
        std::cerr << "Empty file." << std::endl;
        return 0;
    }
    std::vector<Point> curveVertices;
    std::string line;

    while(std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if(prefix == "v")
        {
            double x, y, z;
            iss >> x >> y >> z;
            curveVertices.emplace_back(x, y, z); 
        }
    }

    //rewind file
    file.clear();
    file.seekg(0, std::ios::beg);


    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        
        if(prefix == "l")
        {
            int idx; 
            while(iss >> idx)
            {
                if(idx -1 >= 0 && idx -1 <curveVertices.size())
                {
                    curve.push_back(curveVertices[idx - 1]);
                }
                else
                {
                    std::cerr << "Invalid index." << std::endl;
                    return 0;
            }
            
        }

    } 

 }
return 1;

 }



