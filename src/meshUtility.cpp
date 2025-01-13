#include "meshUtility.h"
#include "mesh.h"

#include <cstdio>
#include <filesystem>
#include <vector>
#include <limits> 

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>  

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Triangle_3 = Kernel::Triangle_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> AABB_Tree;
typedef Kernel::Vector_3 Vector_3;
typedef boost::graph_traits<Polygon_mesh>::vertex_descriptor vertex_descriptor;



std::map<std::string, std::array<Point, 3>> meshUtility::divideMeshForBarycentricComputing(Polygon_mesh &polygon, double z_threshold, double y_threshold)
{
    // Step 1: Cut the mesh along the XY plane
    std::vector<Point> xy_midline_points;
    //double z_threshold = 0.1; // Threshold for splitting along XY plane (based on Z value)
    for (vertex_descriptor v : vertices(polygon)) {
        Point vertex_point = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XY plane (i.e., its Z-coordinate is close to 0)
        if (std::fabs(vertex_point.z()) < z_threshold) {
            xy_midline_points.push_back(vertex_point);
        }
    }
    
    // std::cerr << "Number of points on XY midline: " << xy_midline_points.size() << std::endl;   
    // //print coordinates
    // for(const auto& point : xy_midline_points) {
    //     std::cout << "spaceLocator -p " << point << ";" << std::endl;
    // }

    // Step 2: Cut the mesh along the XZ plane
    std::vector<Point> xz_midline_points;
    //double y_threshold = 1e-3; // Threshold for splitting along XZ plane (based on Y value)
    for (vertex_descriptor v : vertices(polygon)) {
        Point vertex_point = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XZ plane (i.e., its Y-coordinate is close to 0)
        if (std::fabs(vertex_point.y()) < y_threshold) {
            xz_midline_points.push_back(vertex_point);
        }
    }
    
    // std::cerr << "Number of points on XZ midline: " << xz_midline_points.size() << std::endl;
    // //print coordinates
    // for(const auto& point : xz_midline_points) {
    //       std::cout << "spaceLocator -p " << point << ";" << std::endl;
    //  }

    
    //step 3: based on the four big intersections, we can make imaginary quads and divide those into triangles
    std::vector<Point> intersection_points;
    for(const auto& point: xy_midline_points)
    {
        //find the intersections with the xz midline
        for(const auto& point2: xz_midline_points)
        {
            if(point2.x() == point.x() && point2.y() == point.y() && point2.z() == point.z())
            {
               intersection_points.push_back(point);

            }
        }
    }
    
    //split up the lines xymidline using the intersection points and put back the points into a new vector
    Point intersection1 = intersection_points[0];
    Point intersection2 = intersection_points[1];

    // Step 3: Split XY midline
    std::vector<Point> top_xy_points;
    for (const auto &point : xy_midline_points) {
        // Only keep points between the two intersections
        if (point.y() >= intersection1.y())
            {
            top_xy_points.push_back(point);
            }
    }

    std::vector<Point> bottom_xy_points;
    for (const auto &point : xy_midline_points) {
        // Only keep points between the two intersections
        if (point.y() <= intersection1.y())
            {
            bottom_xy_points.push_back(point);
            }
    }

    // Step 4: split XZ midline using the intersection points and put back the points into a new vector
    std::vector<Point> right_xz_points;
    for (const auto &point : xz_midline_points) {
        // Only keep points between the two intersections
        if (point.z() >= intersection1.z())
            {
            right_xz_points.push_back(point);
            }
    }

    std::vector<Point> left_xz_points;
    for( const auto &point : xz_midline_points) {
        // Only keep points between the two intersections
        if (point.z() <= intersection1.z())
            {
            left_xz_points.push_back(point);
            }
    }


    //Step 5: create the actual triangles from the points
    
    Point_3 midpoint_right_xz = right_xz_points[ceil(right_xz_points.size() / 2)];
    Point_3 midpoint_bottom_xy = bottom_xy_points[ceil(bottom_xy_points.size() / 2)];
    Point_3 midpoint_left_xz = left_xz_points[ceil(left_xz_points.size() / 2)];
    Point_3 midpoint_top_xy = top_xy_points[ceil(top_xy_points.size() / 2)];

    //create dictionary for triangles to be used for barycentric coordinates
    std::map<std::string, std::array<Point, 3>> triangles; 
    triangles["triangle1"] = {midpoint_right_xz, midpoint_bottom_xy, intersection1};
    triangles["triangle2"] = {midpoint_right_xz, midpoint_bottom_xy, intersection2};
    triangles["triangle3"] = {midpoint_left_xz, midpoint_top_xy, intersection1};
    triangles["triangle4"] = {midpoint_left_xz, midpoint_top_xy, intersection2};
    triangles["triangle5"] = {midpoint_right_xz, midpoint_top_xy, intersection1};
    triangles["triangle6"] = {midpoint_right_xz, midpoint_top_xy, intersection2};
    triangles["triangle7"] = {midpoint_left_xz, midpoint_bottom_xy, intersection1};
    triangles["triangle8"] = {midpoint_left_xz, midpoint_bottom_xy, intersection2};

    std::cout << "Mesh divided successfully into: "<< triangles.size() << " triangles" << std::endl;

    return triangles;
}

void meshUtility::computeBarycentric_coordinates(Polygon_mesh &polygon, std::map<std::string, std::array<Point, 3>> triangles, std::map<std::string, std::vector<std::pair<std::array<double, 3>, double>>> &barycentric_coordinates)
{
    Mesh mesh; 
    mesh.triangulateMesh(polygon);
    //Create an AABB tree for the mesh
    AABB_Tree tree(faces(polygon).first, faces(polygon).second, polygon);
    
    
    //get the vertices that are inside each triangle
    for(const auto& triangle : triangles) {
        const std::string &triangle_id = triangle.first;
        std::array<Point, 3> triangle_points = triangle.second;
        const Point &A = triangle_points[0];
        const Point &B = triangle_points[1];
        const Point &C = triangle_points[2];  

        CGAL::Plane_3<Kernel> plane(A, B, C);

        double area_ABC = CGAL::sqrt(CGAL::squared_area(A, B, C));

        // Create a vector to store the barycentric coordinates

        std::array<double, 3> bary_coords;
        
        // Iterate over all vertices in the mesh
        for (vertex_descriptor v : vertices(polygon)) 
        {
            
            Point P = get(CGAL::vertex_point, polygon, v);
            Point P_projected = plane.projection(P);

            double distance_to_plane = CGAL::sqrt(CGAL::squared_distance(P, P_projected));

            double area_PBC = CGAL::sqrt(CGAL::squared_area(P_projected, B, C));
            double area_PCA = CGAL::sqrt(CGAL::squared_area(P_projected, C, A));
            double area_PAB = CGAL::sqrt(CGAL::squared_area(P_projected, A, B));

            double u = area_PBC / area_ABC;
            double vu = area_PCA / area_ABC;
            double w = area_PAB / area_ABC;

            if (u >= 0 && vu >= 0 && w >= 0 && std::abs(u + vu + w - 1) < 1e-6)
            {
                bary_coords[0] = u;
                bary_coords[1] = vu;
                bary_coords[2] = w;
                barycentric_coordinates[triangle_id].emplace_back(bary_coords, distance_to_plane);
            }

            }
        }

// //print barycentric coordinates

//     for (const auto &[triangle_id, coords] : barycentric_coordinates
// ) 
//     {
//         std::cout << "Triangle: " << triangle_id << "\n";
//         for (const auto &[bary_coords, distance] : coords) {
//             std::cout << "  Barycentric: (" << bary_coords[0] << ", " << bary_coords[1] << ", " << bary_coords[2]
//                       << "), Distance: " << distance << "\n";
//     }
        
        std::cout << "Barycentric coordinates computed successfully." << std::endl;

}



void meshUtility::projectTrianglePoints(std::map<std::string, std::array<Point, 3>> trianglesSource, std::map<std::string, std::array<Point, 3>> trianglesTarget , std::map<std::string, std::array<Point, 3>> &projected_points)
{
    if(trianglesSource.size() != trianglesTarget.size())
    {
        std::cerr << "Invalid input at projectTrianglePoints, not same amount of triangles" << std::endl;
        return;
    }

    for(const auto &[key, sourceTriangle] : trianglesSource)
    {
        auto targetTriangleIt = trianglesTarget.find(key);
        
        if(targetTriangleIt == trianglesTarget.end())
        {
            std::cerr << "Invalid input at projectTrianglePoints, triangle not found" << std::endl;
            return;
        }

    
    const auto &targetTriangle = targetTriangleIt->second;
    std::array<Point, 3> projectedTriangle;
    
    for(size_t j = 0; j < 3; j++)
    {
        CGAL::Vector_3<Kernel> displacement = targetTriangle[j] - sourceTriangle[j];
        projectedTriangle[j] = Point(sourceTriangle[j].x() + displacement.x(), 
                            sourceTriangle[j].y() + displacement.y(), 
                            sourceTriangle[j].z() + displacement.z());
    
    }
    
    projected_points[key] = projectedTriangle;
}
    std::cout << "Triangle points: " << projected_points.size() << " projected successfully." << std::endl;
}


Point operator*(double scalar, const Point& point)
{
    // Assuming Point is a 3D point with x(), y(), z() methods
    return Point(scalar * point.x(), scalar * point.y(), scalar * point.z());
}

Point operator+(const Point& p1, const Point& p2)
{
    return Point(p1.x() + p2.x(), p1.y() + p2.y(), p1.z() + p2.z());
}



std::map<std::string, Point> meshUtility::initialWrapping(std::map<std::string, std::array<Point, 3>> trianglesSource, std::map<std::string, std::array<Point, 3>> trianglesTarget, std::map<std::string, std::vector<std::pair<std::array<double, 3>, double>>> &barycentric_coordinatesSource)
{
    if(trianglesSource.size() != trianglesTarget.size())
    {
        std::cerr << "Invalid input at initialWrapping, not same amount of triangles" << std::endl;
        return {};
    }

    std::map<std::string, Point> WrappedPoints;

    for(const auto &[key, sourceTriangle] : trianglesSource)
    {
        auto targetTriangleIt = trianglesTarget.find(key);
        
        if(targetTriangleIt == trianglesTarget.end())
            {
                std::cerr << "Invalid input at initialWrapping, triangle not found" << std::endl;
                continue;
            }
    
        const auto &targetTriangle = targetTriangleIt->second;

        auto baryIt = std::find_if(
            barycentric_coordinatesSource[key].begin(),
            barycentric_coordinatesSource[key].end(),
            [&](const auto &entry)
            {
                const auto &[baryCoords, distance] = entry;
                return distance < 1e-6;
            }
        );

        if(baryIt == barycentric_coordinatesSource[key].end())
        {
            std::cerr << "Invalid input at initialWrapping, barycentric coordinates not found" << std::endl;
            continue;
        }

        const auto &[baryCoords, distance] = *baryIt;

        Point wrappedPoint = 
            baryCoords[0] * targetTriangle[0] +
            baryCoords[1] * targetTriangle[1] +
            baryCoords[2] * targetTriangle[2];   

        WrappedPoints[key] = wrappedPoint;

    }
    std::cout << "Initial wrapping completed successfully." << std::endl;
    return WrappedPoints;
}

