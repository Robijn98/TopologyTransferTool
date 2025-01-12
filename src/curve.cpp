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
        std::cerr << "Invalid input, curve not loaded" << std::endl;
        return 0;
    }
    if (std::filesystem::is_empty(filename))
    {
        std::cerr << "Empty file, curve not loaded" << std::endl;
        return 0;
    }
    std::vector<Point> curveVertices;
    std::string line;

    while(std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        //std::cout << prefix << std::endl;
        if(prefix == "v")
        {
            double x, y, z;
            iss >> x >> y >> z;
            //std::cout << "X Y Z Coordinates" << x << " " << y << " " << z << std::endl;
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

            //for each l line create a 'curve' vector of points, and store the points in the curve vector
            try
            {
                while(iss >> idx)
                {
                //look up the vertex in the file
                //std::cout << curveVertices[idx - 1] << std::endl;
                curve.push_back(curveVertices[idx]);
                }
            }    
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        //print all points in the curve
        
        // for (auto point : curve)
        // {
        //     std::cout << "curveVertex: " << point << std::endl;
        // }
            
        std::cout << "Curve: " << filename << " loaded succesfully"  << std::endl;        
        
    } 

 }
return 1;

}


std::vector<Point> Curve::discretizeCurve(std::vector<Point> &curve, std::vector<Point> &curveOut, int numPoints)
{
    //curvature based discretization
    //use treshold to subdivide segments where curvature is high (specific value)
    //traverse the polyline 
    
    double totalLength = 0.0;
    //euclidean distance 
    
    for (int i = 0; i < curve.size() - 1; i++)
    {
        totalLength += CGAL::sqrt(CGAL::squared_distance(curve[i], curve[i + 1]));
    }
    //curvature based discretization
    for(int i=0; i<curve.size() -2; i++)
    {
        CGAL::Vector_3<Kernel> v1 = curve[i+1] - curve[i];
        CGAL::Vector_3<Kernel> v2 = curve[i+2] - curve[i+1];
    
        double angle = CGAL::approximate_angle(v1, v2);
    
        //std::cout << angle << std::endl;

    if(angle > 0.1)
    {
        for(int j = 0; j < numPoints; j++)
        {
            double t = static_cast<double>(j) / numPoints;
            curveOut.push_back(curve[i] + t * v1);
        }
    }
    else
    {
        curveOut.push_back(curve[i]);
    }

    while(curveOut.size() > numPoints)
    {
        curveOut.pop_back();
    }

    }
    //print curve name
    std::cout << "Curve discretized succesfully" << std::endl;
    
    return curveOut;
    
}


std::vector<Point> Curve:: projectPoints(std::vector<Point> &curveSource, std::vector<Point> &curveTarget, std::vector<Point> &projectedCurve)
{
    //project points from source curve to target curve
    if (curveSource.size() != curveTarget.size())
    {
        std::cerr << "Invalid input at projectpoints, curves not same length" << std::endl;
        return projectedCurve;
    }
    
    for(int i = 0; i < curveSource.size(); i++)
    {
        //parameteric comparison
        CGAL::Vector_3<Kernel> v = curveTarget[i] - curveSource[i];

        Point newPoint(
        curveSource[i].x() + v.x(),
        curveSource[i].y() + v.y(),
        curveSource[i].z() + v.z());


        projectedCurve.push_back(newPoint);
        //std::cerr<< curveSource[i] << std::endl;
    }
    std::cout << "Curve points projected succesfully" << std::endl;

    return projectedCurve;
}

