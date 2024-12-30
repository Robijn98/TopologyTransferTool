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