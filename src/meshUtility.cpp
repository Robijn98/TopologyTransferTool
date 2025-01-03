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

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Triangle_3 = Kernel::Triangle_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;


// Define the primitive used by the AABB tree: triangles
typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> AABB_Tree;
typedef Kernel::Vector_3 Vector_3;

void meshUtility:: computeBarycentric_coordinates(Polygon_mesh &polygon, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates)
{
    Mesh mesh;
    mesh.triangulateMesh(polygon);
    //build AABB tree structure
    AABB_Tree tree(faces(polygon).first, faces(polygon).second, polygon);

    for (Point p : curveRef)
    {
        //find closest point on the mesh
        Point_3 closest_point = tree.closest_point(p);
        //find the face that contains the closest point
        auto [face, primitive] = tree.closest_point_and_primitive(p);
        // Get the face from the primitive
        Surface_mesh::Face_index closest_face(static_cast<int>(primitive.id()));
        
        //get the vertices of the face
        std::vector<Point_3> vertices;
        for (Surface_mesh::Vertex_index v : vertices_around_face(polygon.halfedge(closest_face), polygon))
        {
            vertices.push_back(polygon.point(v));
        }
        //compute barycentric coordinates
        Point_3 v1 = vertices[0];
        Point_3 v2 = vertices[1];
        Point_3 v3 = vertices[2];

        double lambda1, lambda2, lambda3;
        
        double total_area = CGAL::to_double(CGAL::squared_area(v1, v2, v3));
        lambda1 = CGAL::to_double(CGAL::squared_area(closest_point, v2, v3)) / total_area;
        lambda2 = CGAL::to_double(CGAL::squared_area(closest_point, v3, v1)) / total_area;
        lambda3 = 1 - lambda1 - lambda2;
        
        barycentric_coordinates.push_back(lambda1);
        barycentric_coordinates.push_back(lambda2);
        barycentric_coordinates.push_back(lambda3);

        std::cout << "Barycentric coordinates: " << lambda1 << " " << lambda2 << " " << lambda3 << std::endl;
    
    }
}

void meshUtility::initialWrapping(Polygon_mesh &polygonSource, Polygon_mesh &polygonTarget, std::vector<Point> &curveRef, std::vector<double> &barycentric_coordinates)
{
    //only going over the curve points
    {
    std::ofstream outfile("files/output.obj");
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file for writing." << std::endl;
        return;
    }

    for (size_t i = 0; i < curveRef.size(); ++i) 
    {
        Point p = curveRef[i];
        double lambda1 = barycentric_coordinates[3 * i];
        double lambda2 = barycentric_coordinates[3 * i + 1];
        double lambda3 = barycentric_coordinates[3 * i + 2];
        
        // Define interpolate_position as a lambda function
        auto interpolate_position = [](const Point_3& v1, const Point_3& v2, const Point_3& v3, double lambda1, double lambda2, double lambda3) {
            // Convert points to vectors
            Vector_3 vec1 = v1 - CGAL::ORIGIN;
            Vector_3 vec2 = v2 - CGAL::ORIGIN;
            Vector_3 vec3 = v3 - CGAL::ORIGIN;

            // Compute the weighted sum of the vectors
            Vector_3 interpolated_vector = lambda1 * vec1 + lambda2 * vec2 + lambda3 * vec3;

            // Convert the result back to a point
            Point_3 interpolated_position = CGAL::ORIGIN + interpolated_vector;

            return interpolated_position;
        };

        // Find the closest point on the source mesh
        AABB_Tree tree(faces(polygonSource).first, faces(polygonSource).second, polygonSource);
        Point_3 closest_point = tree.closest_point(p);
        auto [face, primitive] = tree.closest_point_and_primitive(p);
        Surface_mesh::Face_index closest_face(static_cast<int>(primitive.id()));
        std::vector<Point_3> vertices;
        for (Surface_mesh::Vertex_index v : vertices_around_face(polygonSource.halfedge(closest_face), polygonSource))
        {
            vertices.push_back(polygonSource.point(v));
        }
        Point_3 v1 = vertices[0];
        Point_3 v2 = vertices[1];
        Point_3 v3 = vertices[2];

        // Compute the interpolated position
        Point_3 interpolatedPosition = interpolate_position(v1, v2, v3, lambda1, lambda2, lambda3);

        std::cout << "Interpolated Position: " << interpolatedPosition << std::endl;

        // Find the closest point on the target mesh
        AABB_Tree treeTarget(faces(polygonTarget).first, faces(polygonTarget).second, polygonTarget);
        Point_3 closest_point_target = treeTarget.closest_point(interpolatedPosition);
        auto [faceTarget, primitiveTarget] = treeTarget.closest_point_and_primitive(interpolatedPosition);
        Surface_mesh::Face_index closest_face_target(static_cast<int>(primitiveTarget.id()));
        std::vector<Point_3> verticesTarget;
        for (Surface_mesh::Vertex_index v : vertices_around_face(polygonTarget.halfedge(closest_face_target), polygonTarget))
        {
            verticesTarget.push_back(polygonTarget.point(v));
        }
        Point_3 v1Target = verticesTarget[0];
        Point_3 v2Target = verticesTarget[1];
        Point_3 v3Target = verticesTarget[2];
        // Perform barycentric interpolation on the target mesh

        Vector_3 vec1Target = v1Target - CGAL::ORIGIN;
        Vector_3 vec2Target = v2Target - CGAL::ORIGIN;
        Vector_3 vec3Target = v3Target - CGAL::ORIGIN;

        Vector_3 interpolated_vector_target = lambda1 * vec1Target + lambda2 * vec2Target + lambda3 * vec3Target;

        // Convert the result back to a point (origin + interpolated vector)
        Point_3 new_point_target = CGAL::ORIGIN + interpolated_vector_target;
        
        // Debugging before writing
        std::cout << "Writing point to file: v " 
                  << new_point_target.x() << " " 
                  << new_point_target.y() << " " 
                  << new_point_target.z() << std::endl;

        // Write to file
        outfile << "v " 
                << new_point_target.x() << " " 
                << new_point_target.y() << " " 
                << new_point_target.z() << std::endl;

        outfile.flush();  // Ensure the data is flushed to the file
    }

    outfile.close();
    std::cout << "Points written to file 'output.obj' successfully." << std::endl;
}
}

