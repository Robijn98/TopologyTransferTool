#include "meshUtility.h"
#include <cstdio>
#include <filesystem>
#include <vector>
#include "mesh.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Kd_tree.h>
//#include <CGAL/Point_set_processing_3.h>
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Triangle_3 = Kernel::Triangle_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;


// Define the primitive used by the AABB tree: triangles
typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> AABB_Tree;
typedef Kernel::Vector_3 Vector_3;


void meshUtility::computeBarycentric_coordinates(Polygon_mesh &polygon, std::vector<Point> &curveRef ,std::vector<std::array<double, 3>> &barycentric_coordinates) 
{
    // Divide the mesh into triangulated regions
    //close the guiding curve if not closed 
    if(curveRef.front() != curveRef.back())
    {
        curveRef.push_back(curveRef.front());
    }
    //make an intersection perpundicular to the curve
    //get the mid point of the curve
    double curve_midPoint = floor(curveRef.size()/2);
    //get the vector from the lastpoint to the mid point
    Vector_3 vector = curveRef[curve_midPoint-1] - curveRef[curve_midPoint];
    Vector_3 perpendicular = CGAL::cross_product(vector, Vector_3(0,0,1));
    perpendicular = perpendicular / std::sqrt(perpendicular.squared_length());

    //find the next vertex along the perpendicular vector
    double step_size = 1.0;
    Point_3 next_point = curveRef[curve_midPoint] + step_size * perpendicular;

    //get vertices of the mesh
    AABB_Tree tree(faces(polygon).first, faces(polygon).second, polygon);
    Point_3 closest_point = tree.closest_point(next_point);
    std::cout << "Closest point: " << closest_point << std::endl;
    


}   



// void meshUtility:: computeBarycentric_coordinates(Polygon_mesh &polygon, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates)
// {
//     Mesh mesh;
//     mesh.triangulateMesh(polygon);
//     //build AABB tree structure
//     AABB_Tree tree(faces(polygon).first, faces(polygon).second, polygon);

//     for (Surface_mesh::Face_index face : faces(polygon)) 
//     {
//         std::vector<Point_3> vertices;
//         for (Surface_mesh::Vertex_index v : vertices_around_face(polygon.halfedge(face), polygon)) 
//         {
//             vertices.push_back(polygon.point(v));
//         }

//         Point_3 v1 = vertices[0];
//         Point_3 v2 = vertices[1];
//         Point_3 v3 = vertices[2];

//         int samples_per_edge = 10;
        
//         for (int i=0; i<= samples_per_edge; i++)
//         {
//             for (int j=0; j<= samples_per_edge; j++)
//             {
//                 double lambda1 = i/static_cast<double>(samples_per_edge);
//                 double lamdba2 = j/static_cast<double>(samples_per_edge);
//                 double lambda3 = 1 - lambda1 - lamdba2;

//                 Point_3 point = CGAL::ORIGIN + lambda1 * (v1 - CGAL::ORIGIN) + lamdba2 * (v2 - CGAL::ORIGIN) + lambda3 * (v3 - CGAL::ORIGIN);
                
//                 barycentric_coordinates.push_back(lambda1);
//                 barycentric_coordinates.push_back(lamdba2);
//                 barycentric_coordinates.push_back(lambda3);

//                 std::cout << "Barycentric coordinates: " << lambda1 << " " << lamdba2 << " " << lambda3 << std::endl;

//             }
//         }
    
//     }
// }

// void meshUtility::computeBarycentric_coordinates(Polygon_mesh &polygon,std::vector<std::array<double, 3>> &barycentric_coordinates) 
// {
//     // Triangulate the mesh
//     Mesh mesh;
//     mesh.triangulateMesh(polygon);

//     // Build AABB tree structure
//     AABB_Tree tree(faces(polygon).first, faces(polygon).second, polygon);

//     // Prepare barycentric storage
//     barycentric_coordinates.clear();
//     barycentric_coordinates.reserve(num_vertices(polygon)); // Optimize memory allocation

//     // Iterate over all vertices of the mesh
//     for (Surface_mesh::Vertex_index v : vertices(polygon)) {
//         Point_3 vertex_point = polygon.point(v);

//         // Find closest point on the mesh
//         Point_3 closest_point = tree.closest_point(vertex_point);
//         auto [face, primitive] = tree.closest_point_and_primitive(vertex_point);
//         Surface_mesh::Face_index closest_face(static_cast<int>(primitive.id()));

//         // Get the vertices of the closest face
//         std::vector<Point_3> vertices;
//         for (Surface_mesh::Vertex_index fv : vertices_around_face(polygon.halfedge(closest_face), polygon)) {
//             vertices.push_back(polygon.point(fv));
//         }

//         if (vertices.size() != 3) {
//             std::cerr << "Face does not have exactly 3 vertices!" << std::endl;
//             barycentric_coordinates.push_back({0.0, 0.0, 0.0});
//             continue;
//         }

//         Point_3 v1 = vertices[0], v2 = vertices[1], v3 = vertices[2];

//         // Inline triangle area computation
//         auto computeArea = [](const Point_3 &p1, const Point_3 &p2, const Point_3 &p3) {
//             // Compute the vectors from p1 to p2 and p1 to p3
//             Vector_3 edge1 = p2 - p1;
//             Vector_3 edge2 = p3 - p1;

//             // Compute the cross product of the two edges
//             Vector_3 cross_product = CGAL::cross_product(edge1, edge2);

//             // Compute the magnitude of the cross product
//             return 0.5 * std::sqrt(cross_product.squared_length());
//         };

//         // Compute the total area of the triangle
//         double total_area = computeArea(v1, v2, v3);

//         // Compute barycentric coordinates using sub-triangle areas
//         double lambda1 = computeArea(closest_point, v2, v3) / total_area;
//         double lambda2 = computeArea(closest_point, v3, v1) / total_area;
//         double lambda3 = 1.0 - lambda1 - lambda2;

//         // Store the barycentric coordinates
//         barycentric_coordinates.push_back({lambda1, lambda2, lambda3});

//         std::cout << "Vertex: " << vertex_point << " | Barycentric: " << lambda1 << ", " << lambda2 << ", " << lambda3 << std::endl;
//     }
// }


// void meshUtility::initialWrapping(Polygon_mesh &polygonSource, Polygon_mesh &polygonTarget, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates) 
// {
//     std::ofstream outfile("files/output.obj");
//     if (!outfile.is_open()) {
//         std::cerr << "Error: Could not open output file for writing." << std::endl;
//         return;
//     }

//     AABB_Tree treeTarget(faces(polygonTarget).first, faces(polygonTarget).second, polygonTarget);

//     for (size_t i = 0; i < barycentric_coordinates.size() / 3; ++i) {
//         double lambda1 = barycentric_coordinates[3 * i];
//         double lambda2 = barycentric_coordinates[3 * i + 1];
//         double lambda3 = barycentric_coordinates[3 * i + 2];
//         assert(std::abs(lambda1 + lambda2 + lambda3 - 1.0) < 1e-9);
//     }
//     std::cerr << "Barycentric coordinates are correct" << std::endl;


//     for (size_t i = 0; i < barycentric_coordinates.size() / 3; i++) 
//     {
//         double lambda1 = barycentric_coordinates[3 * i];
//         double lambda2 = barycentric_coordinates[3 * i + 1];
//         double lambda3 = barycentric_coordinates[3 * i + 2];

//         // Assume point corresponds to curveRef[i] (adjust as needed)
//         Point_3 point = curveRef[i];

//         // Find closest point on the target mesh
//         Point_3 closest_point_target = treeTarget.closest_point(point);
//         auto [faceTarget, primitiveTarget] = treeTarget.closest_point_and_primitive(point);
//         Surface_mesh::Face_index closest_face_target(static_cast<int>(primitiveTarget.id()));

//         // Retrieve vertices of the closest face on the target mesh
//         std::vector<Point_3> targetVertices;
//         for (Surface_mesh::Vertex_index v : vertices_around_face(polygonTarget.halfedge(closest_face_target), polygonTarget)) {
//             targetVertices.push_back(polygonTarget.point(v));
//         }


//         // Interpolate the position on the target mesh
//         Vector_3 vec1Target = targetVertices[0] - CGAL::ORIGIN;
//         Vector_3 vec2Target = targetVertices[1] - CGAL::ORIGIN;
//         Vector_3 vec3Target = targetVertices[2] - CGAL::ORIGIN;
//         //std::cerr << "passed through here" << std::endl;

//         Vector_3 interpolated_position_target = lambda1 * vec1Target + lambda2 * vec2Target + lambda3 * vec3Target;
//         Point_3 interpolated_point_target = CGAL::ORIGIN + interpolated_position_target;

//         // Write interpolated point to the file
//         outfile << "v " << interpolated_point_target.x() << " "
//                 << interpolated_point_target.y() << " "
//                 << interpolated_point_target.z() << std::endl;

//     }

//     outfile.close();
// }
