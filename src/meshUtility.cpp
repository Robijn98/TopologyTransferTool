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



std::map<std::string, std::array<Point, 3>> meshUtility::divideMeshForBarycentricComputing(Polygon_mesh &polygon, Polygon_mesh &debugMesh, double z_threshold, double y_threshold)
{
    // Step 1: Cut the mesh along the XY plane
    std::vector<Point> xy_midline_points;

    for (vertex_descriptor v : vertices(polygon)) {
        Point vertex_point = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XY plane its Z-coordinate is close to 0
        if (std::fabs(vertex_point.z()) < z_threshold) {
            xy_midline_points.push_back(vertex_point);
        }
    }
    
    // Step 2: Cut the mesh along the XZ plane
    std::vector<Point> xz_midline_points;

    for (vertex_descriptor v : vertices(polygon)) {
        Point vertex_point = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XZ plane its Y-coordinate is close to 0
        if (std::fabs(vertex_point.y()) < y_threshold) {
            xz_midline_points.push_back(vertex_point);
        }
    }


    //step 3: divide quads into triangles
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

        if (point.y() >= intersection1.y())
            {
            top_xy_points.push_back(point);
            }
    }

    std::vector<Point> bottom_xy_points;
    for (const auto &point : xy_midline_points) {

        if (point.y() <= intersection1.y())
            {
            bottom_xy_points.push_back(point);
            }
    }

    // Step 4: split XZ midline using the intersection points and put back the points into a new vector
    std::vector<Point> right_xz_points;
    for (const auto &point : xz_midline_points) {

        if (point.z() >= intersection1.z())
            {
            right_xz_points.push_back(point);
            }
    }

    std::vector<Point> left_xz_points;
    for( const auto &point : xz_midline_points) {

        if (point.z() <= intersection1.z())
            {
            left_xz_points.push_back(point);
            }
    }


    //Step 5: create the actual triangles from the points
    
    Point_3 midpoint_right_xz = right_xz_points[right_xz_points.size() / 2];
    Point_3 midpoint_bottom_xy = bottom_xy_points[bottom_xy_points.size() / 2];
    Point_3 midpoint_left_xz = left_xz_points[left_xz_points.size() / 2];
    Point_3 midpoint_top_xy = top_xy_points[top_xy_points.size() / 2];

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

    //make sure the triangles have the right normal
    //do it for triangle 0,2,5,7

    //print
    for(const auto& triangle : triangles)
    {
        std::cout << "//Triangle: " << triangle.first << "\n";
        for(const auto& point : triangle.second)
        {
            std::cout << "spaceLocator -p " << point << ";\n";
        }
    }


    for(auto& triangle : triangles)
    {
        if(triangle.first == "triangle1" || triangle.first == "triangle3" || triangle.first == "triangle6" || triangle.first == "triangle8")
        {
        Point A = triangle.second[0];
        Point B = triangle.second[1];
        Point C = triangle.second[2];

        std::swap(triangle.second[0], triangle.second[1]);
        }
    }


    //turn points into faces    
    for(const auto& triangle : triangles) 
    {
        vertex_descriptor v0 = add_vertex(triangle.second[0], debugMesh);
        vertex_descriptor v1 = add_vertex(triangle.second[1], debugMesh);
        vertex_descriptor v2 = add_vertex(triangle.second[2], debugMesh);
        debugMesh.add_face(v0, v1, v2);
    }


    std::cout << "Mesh divided successfully into: "<< triangles.size() << " triangles\n";

    return triangles;
}

void meshUtility::computeBarycentric_coordinates(Polygon_mesh &polygon, Polygon_mesh &octahedron ,std::map<std::string, std::array<Point, 3>> triangles, std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentric_coordinates)
{
    Mesh mesh; 
    mesh.triangulateMesh(polygon);

    // Create an AABB tree for the octahedron
    AABB_Tree tree(faces(octahedron).first, faces(octahedron).second, octahedron);

    std::set<vertex_descriptor> processed_vertices;
    std::map<vertex_descriptor, Polygon_mesh::Face_index> closest_face_map;

    // First, find the closest face for each vertex in the polygon mesh
    for (vertex_descriptor v : vertices(polygon)) {
        Point P = get(CGAL::vertex_point, polygon, v);
        
        auto closest_face = tree.closest_point_and_primitive(P);
        closest_face_map[v] = closest_face.second;  
    }

    for (vertex_descriptor v : vertices(polygon)) {
        if (processed_vertices.count(v)) continue;  

        // Get the closest face for the current vertex
        Polygon_mesh::Face_index closest_face_descriptor = closest_face_map[v];
        
        // Get the halfedge descriptor for the closest face
        auto halfedge_descriptor = halfedge(closest_face_descriptor, octahedron);
        auto vertices_around = vertices_around_face(halfedge_descriptor, octahedron);

        // Get the vertices of the closest face
        auto vertex_iter = vertices_around.begin();
        Point A = get(CGAL::vertex_point, octahedron, *vertex_iter);
        vertex_iter++;
        Point B = get(CGAL::vertex_point, octahedron, *vertex_iter);
        vertex_iter++;
        Point C = get(CGAL::vertex_point, octahedron, *vertex_iter);

        // Compute the plane of the triangle defined by vertices A, B, C
        CGAL::Plane_3<Kernel> plane(A, B, C);
        double area_ABC = CGAL::sqrt(CGAL::squared_area(A, B, C));
        std::array<double, 3> bary_coords;

        // Project the point onto the plane
        Point P_projected = plane.projection(get(CGAL::vertex_point, polygon, v));
        double distance_to_plane = CGAL::sqrt(CGAL::squared_distance(get(CGAL::vertex_point, polygon, v), P_projected));

        // Calculate areas for barycentric coordinates
        double area_PBC = CGAL::sqrt(CGAL::squared_area(P_projected, B, C));
        double area_PCA = CGAL::sqrt(CGAL::squared_area(P_projected, C, A));
        double area_PAB = CGAL::sqrt(CGAL::squared_area(P_projected, A, B));

        // Barycentric coordinates
        double u_bary = area_PBC / area_ABC;
        double v_bary = area_PCA / area_ABC;
        double w_bary = area_PAB / area_ABC;

        std::cout << "Vertex: " << v << " Closest Face Descriptor: " << closest_face_descriptor << "\n";
        //convert the closest face descriptor to a string
        // Check if the barycentric coordinates are valid (non-negative and sum to 1)
        if (u_bary >= 0 && v_bary >= 0 && w_bary >= 0 && std::abs(u_bary + v_bary + w_bary - 1) < 1e-6)
        {
            std::string closest_face_descriptor_str = std::to_string(closest_face_descriptor);
            bary_coords = {u_bary, v_bary, w_bary};
            barycentric_coordinates[closest_face_descriptor_str].emplace_back(v, bary_coords, distance_to_plane);
            processed_vertices.insert(v);  // Mark the vertex as processed
        }
        
    }

    // Print barycentric coordinates
    // for (const auto &[triangle_id, coords] : barycentric_coordinates) {
    //     std::cout << "Face: " << triangle_id << "\n";
    //     for (const auto &[vertex_id, bary_coords, distance] : coords) {
    //         std::cout << "Vertex: " << vertex_id << 
    //         "  Barycentric: (" << bary_coords[0] << ", " << bary_coords[1] << ", " << bary_coords[2] << "), Distance: " << distance << "\n";
    //     }
    // }

    std::cout << "Barycentric coordinates computed successfully\n";
}




// void meshUtility::projectTrianglePoints([std::map<std::string, std::array<Point, 3>> trianglesSource, std::map<std::string, std::array<Point, 3>> trianglesTarget] , std::map<std::string, std::array<Point, 3>> &projected_points)
// {
//     if(trianglesSource.size() != trianglesTarget.size())
//     {
//         std::cerr << "Invalid input at projectTrianglePoints, not same amount of triangles\n";
//         return;
//     }

//     for(const auto &[key, sourceTriangle] : trianglesSource)
//     {
//         auto targetTriangleIt = trianglesTarget.find(key);
        
//         if(targetTriangleIt == trianglesTarget.end())
//         {
//             std::cerr << "Invalid input at projectTrianglePoints, triangle not found\n";
//             return;
//         }

    
//     const auto &targetTriangle = targetTriangleIt->second;
//     std::array<Point, 3> projectedTriangle;
    
//     for(size_t j = 0; j < 3; j++)
//     {
//         CGAL::Vector_3<Kernel> displacement = targetTriangle[j] - sourceTriangle[j];
//         projectedTriangle[j] = Point(sourceTriangle[j].x() + displacement.x(), 
//                             sourceTriangle[j].y() + displacement.y(), 
//                             sourceTriangle[j].z() + displacement.z());
    
//     }
    
//     projected_points[key] = projectedTriangle;
// }
//     std::cout << "Triangle points: " << projected_points.size() << " projected successfully\n";
// }


Point operator*(double scalar, const Point& point)
{
    return Point(scalar * point.x(), scalar * point.y(), scalar * point.z());
}

Point operator+(const Point& p1, const Point& p2)
{
    return Point(p1.x() + p2.x(), p1.y() + p2.y(), p1.z() + p2.z());
}



std::map<int, std::vector<Point>> meshUtility::initialWrapping(Polygon_mesh octahedronSource, Polygon_mesh octahedronTarget, Polygon_mesh &sourceMesh ,std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentric_coordinatesSource)
{

    std::map<int, std::vector<Point>> WrappedPoints;

    for(const auto &[face_id, bary_coords]: barycentric_coordinatesSource)
    {
        Polygon_mesh::Face_index target_face = Polygon_mesh::Face_index(std::stoi(face_id));
        auto halfedge_descriptor = halfedge(target_face, octahedronTarget);
        auto vertices_around = vertices_around_face(halfedge_descriptor, octahedronTarget);

        //get points target triangle
        auto vertex_iter = vertices_around.begin();
        Point A = get(CGAL::vertex_point, octahedronTarget, *vertex_iter);
        vertex_iter++;
        Point B = get(CGAL::vertex_point, octahedronTarget, *vertex_iter);
        vertex_iter++;
        Point C = get(CGAL::vertex_point, octahedronTarget, *vertex_iter);

        //calculating the normal of the target triangle
        Vector_3 normal = CGAL::cross_product(B - A, C - A);
        normal = normal / CGAL::sqrt(normal.squared_length());


        //process each vertex in the source
        for(const auto &[vertex_id, bary_coords, distance]: bary_coords)
        {
            double u = bary_coords[0];
            double v = bary_coords[1];
            double w = bary_coords[2];

            Point wrapped_point = u * A + v * B + w * C;
            wrapped_point = wrapped_point + distance * normal;

            WrappedPoints[vertex_id].push_back(wrapped_point);
        }
    }

    //print wrapped points
    for(const auto &[vertexId, point] : WrappedPoints)
    {
        for(const auto &point : point)
        {
            std::cout<< "Vertex ID: " << vertexId << " Point: " << point << "\n";
        }
    }

    std::cout << "Initial wrapping completed successfully\n";
    

    //insert the vertices that are not wrapped from the source mesh //THIS IS FOR DEBUGGING < REMOVE LATER
    for(
        vertex_descriptor v : vertices(sourceMesh))
    {
        if(WrappedPoints.find(v) == WrappedPoints.end())
        {
            Point vertex_point = get(CGAL::vertex_point, sourceMesh, v);
            WrappedPoints[v].push_back(vertex_point);
        }
    }
    
    //print wrapped points
    for(const auto &[vertexId, point] : WrappedPoints)
    {
        for(const auto &point : point)
        {
            std::cout<< "Vertex ID: " << vertexId << " Point: " << point << "\n";
        
        }
    }

    return WrappedPoints;
}
