/// header file encapsulates the class Curve, which is used to load, discretize and project curves.
/// author: Robin van den Eerenbeemd
/// version: 1.0
/// date: 27 jan 2025 updated to NCCA coding standards


#ifndef _CURVE_H_
#define _CURVE_H_

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> Polygon_mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

class Curve
{
    public:
        //load curve from obj file
        bool loadCurve(std::string filename, std::vector<Point> &curve);
        //turns continuous curve into piecewise linear curve and returns points on that curve
        std::vector<Point> discretizeCurve(std::vector<Point> &curve, std::vector<Point> &curveOut, int numPoints);
        //project points from source curve to target curve
        std::vector<Point> projectPoints(std::vector<Point> &curveSource, std::vector<Point> &curveTarget, std::vector<Point> &projectedCurve);
    
    private:
        
};


#endif  //_CURVE_H_