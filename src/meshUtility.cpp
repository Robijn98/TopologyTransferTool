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
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

typedef CGAL::AABB_face_graph_triangle_primitive<Surface_mesh> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits> AABB_Tree;
typedef Kernel::Vector_3 Vector_3;
typedef boost::graph_traits<Polygon_mesh>::vertex_descriptor vertex_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;


std::map<std::string, std::array<Point, 3>> meshUtility::divideMeshForBarycentricComputing(Polygon_mesh &polygon, Polygon_mesh &debugMesh, double zThreshold, double yThreshold)
{
    // Step 1: Cut the mesh along the XY plane
    std::vector<Point> xyMidlinePoints;

    for (vertex_descriptor v : vertices(polygon)) {
        Point vertexPoint = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XY plane its Z-coordinate is close to 0
        if (std::fabs(vertexPoint.z()) < zThreshold) {
            xyMidlinePoints.push_back(vertexPoint);
        }
    }


    // Step 2: Cut the mesh along the XZ plane
    std::vector<Point> xzMidlinePoints;

    for (vertex_descriptor v : vertices(polygon)) {
        Point vertexPoint = get(CGAL::vertex_point, polygon, v);
        
        // If the vertex is near the XZ plane its Y-coordinate is close to 0
        if (std::fabs(vertexPoint.y()) < yThreshold) {
            xzMidlinePoints.push_back(vertexPoint);
        }
    }

    //step 3: divide quads into triangles
    std::vector<Point> intersectionPoints;
    double threshold = 1e-6;
    int maxThreshold = 1;

    while(intersectionPoints.size() != 2 && threshold < maxThreshold)
    {

        for(const auto& point: xyMidlinePoints)
        {
            //find the intersections with the xz midline within a small theshold
            for(const auto& point2: xzMidlinePoints)
            {
                if(std::fabs(point.z() - point2.z()) < threshold 
                && std::fabs(point.y() - point2.y()) < threshold
                && std::fabs(point.x() - point2.x()) < threshold)
                {
                intersectionPoints.push_back(point);

                }
            }
        }
    threshold = threshold * 10;
    }
    

    //split up the lines xymidline using the intersection points and put back the points into a new vector
    
    // Sort intersection points to ensure consistent ordering
    std::sort(intersectionPoints.begin(), intersectionPoints.end(), 
        [](const Point& a, const Point& b) {
            if (a.x() != b.x()) return a.x() < b.x();
            if (a.y() != b.y()) return a.y() < b.y();
            return a.z() < b.z();
        });
    
    
    Point intersection1 = intersectionPoints[0];
    Point intersection2 = intersectionPoints[1];


    // Step 3: Split XY midline
    std::vector<Point> xyTopPoints;
    for (const auto &point : xyMidlinePoints) {

        if (point.y() >= intersection1.y())
            {
            xyTopPoints.push_back(point);
            }
    }


    std::vector<Point> xyBottomPoints;
    for (const auto &point : xyMidlinePoints) {

        if (point.y() <= intersection1.y())
            {
            xyBottomPoints.push_back(point);
            }
    }


    // Step 4: split XZ midline using the intersection points and put back the points into a new vector
    std::vector<Point> xzRightPoints;
    for (const auto &point : xzMidlinePoints) {

        if (point.z() >= intersection1.z())
            {
            xzRightPoints.push_back(point);
            }
    }


    std::vector<Point> xzLeftPoints;
    for( const auto &point : xzMidlinePoints) {

        if (point.z() <= intersection1.z())
            {
            xzLeftPoints.push_back(point);
            }
    }


    //Step 5: create the actual triangles from the points
    
    std::sort(xzRightPoints.begin(), xzRightPoints.end(), 
    [](const Point& a, const Point& b) { return a.x() < b.x(); });
    Point_3 midpointRightXZ = xzRightPoints[xzRightPoints.size() / 2];

    
    std::sort(xyBottomPoints.begin(), xyBottomPoints.end(), 
    [](const Point& a, const Point& b) { return a.x() < b.x(); });
    Point_3 midpointBottomXY = xyBottomPoints[xyBottomPoints.size() / 2];
    
    
    std::sort(xzLeftPoints.begin(), xzLeftPoints.end(), 
    [](const Point& a, const Point& b) { return a.x() < b.x(); });
    Point_3 midpointLeftXZ = xzLeftPoints[xzLeftPoints.size() / 2];
    
    
    std::sort(xyTopPoints.begin(), xyTopPoints.end(), 
    [](const Point& a, const Point& b) { return a.x() < b.x(); });
    Point_3 midpointTopXY = xyTopPoints[xyTopPoints.size() / 2];


    //create dictionary for triangles to be used for barycentric coordinates
    std::map<std::string, std::array<Point, 3>> triangles; 
    triangles["triangle1"] = {midpointBottomXY, midpointRightXZ, intersection1};
    triangles["triangle2"] = {midpointRightXZ, midpointBottomXY, intersection2};
    triangles["triangle3"] = {midpointTopXY, midpointLeftXZ, intersection1};
    triangles["triangle4"] = {midpointLeftXZ, midpointTopXY, intersection2};
    triangles["triangle5"] = {midpointRightXZ, midpointTopXY, intersection1};
    triangles["triangle6"] = {midpointTopXY, midpointRightXZ, intersection2};
    triangles["triangle7"] = {midpointLeftXZ, midpointBottomXY, intersection1};
    triangles["triangle8"] = {midpointBottomXY, midpointLeftXZ, intersection2};

    //turn points into faces    
    for(const auto& triangle : triangles) 
    {
        vertex_descriptor v0 = add_vertex(triangle.second[0], debugMesh);
        vertex_descriptor v1 = add_vertex(triangle.second[1], debugMesh);
        vertex_descriptor v2 = add_vertex(triangle.second[2], debugMesh);
        debugMesh.add_face(v0, v1, v2);
    }

    //scale up the cage to be bigger than the original mesh
    for(vertex_descriptor v : vertices(debugMesh))
    {
        Point vertexPoint = get(CGAL::vertex_point, debugMesh, v);
        vertexPoint = Point(vertexPoint.x() * 2.5, vertexPoint.y() * 2.5, vertexPoint.z() * 2.5);
        put(CGAL::vertex_point, debugMesh, v, vertexPoint);
    }


    std::cout << "Mesh divided successfully into: "<< triangles.size() << " triangles\n";

    return triangles;
}



void meshUtility::computeBarycentricCoordinates(Polygon_mesh &polygon, Polygon_mesh &octahedron ,std::map<std::string, std::array<Point, 3>> triangles, std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentricCoordinates)
{
    Mesh mesh; 
    mesh.triangulateMesh(polygon);

    // Create an AABB tree for the octahedron
    AABB_Tree tree(faces(octahedron).first, faces(octahedron).second, octahedron);

    std::set<vertex_descriptor> processedVertices;
    std::map<vertex_descriptor, Polygon_mesh::Face_index> closestFaceMap;

    // First, find the closest face for each vertex in the polygon mesh
    for (vertex_descriptor v : vertices(polygon)) {
        Point P = get(CGAL::vertex_point, polygon, v);
        
        auto closestFace = tree.closest_point_and_primitive(P);
        closestFaceMap[v] = closestFace.second;  
    }

    for (vertex_descriptor v : vertices(polygon)) 
    {

        // Get the closest face for the current vertex
        Polygon_mesh::Face_index closestFaceDescriptor = closestFaceMap[v];
        
        // Get the halfedge descriptor for the closest face
        auto halfedge_descriptor = halfedge(closestFaceDescriptor, octahedron);
        auto verticesAround = vertices_around_face(halfedge_descriptor, octahedron);

        // Get the vertices of the closest face
        auto vertexIter = verticesAround.begin();
        Point A = get(CGAL::vertex_point, octahedron, *vertexIter);
        vertexIter++;
        Point B = get(CGAL::vertex_point, octahedron, *vertexIter);
        vertexIter++;
        Point C = get(CGAL::vertex_point, octahedron, *vertexIter);

        // Compute the plane of the triangle defined by vertices A, B, C
        CGAL::Plane_3<Kernel> plane(A, B, C);
        double areaABC = CGAL::sqrt(CGAL::squared_area(A, B, C));
        std::array<double, 3> baryCoords;

        // Project the point onto the plane
        Point pOrginal = get(CGAL::vertex_point, polygon, v);
        Point pProjected;
        double distanceToPlane;

        bool validBaryCoords = false;
        int maxIterations = 3;
        int iteration = 0;


        while(!validBaryCoords && iteration < maxIterations)
        {
            
            pProjected = plane.projection(pOrginal);
            distanceToPlane = CGAL::sqrt(CGAL::squared_distance(pOrginal, pProjected));

            // Calculate areas for barycentric coordinates
            double areaPBC = CGAL::sqrt(CGAL::squared_area(pProjected, B, C));
            double areaPCA = CGAL::sqrt(CGAL::squared_area(pProjected, C, A));
            double areaPAB = CGAL::sqrt(CGAL::squared_area(pProjected, A, B));

            // Barycentric coordinates
            double uBary = areaPBC / areaABC;
            double vBary = areaPCA / areaABC;
            double wBary = areaPAB / areaABC;

            // Check if the barycentric coordinates are valid (non-negative and sum to 1)
            if (uBary >= -1e-3  && vBary >= -1e-6  && wBary >= -1e-6 && std::abs(uBary + vBary + wBary - 1) < 1e-6)
            {
                validBaryCoords = true;
                baryCoords = {uBary, vBary, wBary};
            }
            else
            {
                
                // Perturb the point slightly
                Vector_3 edge1 = B - A;
                Vector_3 edge2 = C - A;

                double scale = 0.01;


                double factor1 = scale *(rand() % 100) / 100.0;
                double factor2 = scale *(rand() % 100) / 100.0;

                if(factor1 + factor2 > scale)
                {
                    factor1 = scale - factor1;
                    factor2 = scale - factor2;
                }

                pOrginal = pOrginal + factor1 * edge1 + factor2 * edge2;
            }

            iteration++;
        }

        
        //convert the closest face descriptor to a string
        // Check if the barycentric coordinates are valid (non-negative and sum to 1)
        if (validBaryCoords)
        {
            std::string closestFaceDescriptorString = std::to_string(closestFaceDescriptor);
            barycentricCoordinates[closestFaceDescriptorString].emplace_back(v, baryCoords, distanceToPlane);
            processedVertices.insert(v);  
        }
        else
        {
            std::cout << "Vertex: " << v << " has invalid barycentric coordinates\n";
        }


    }


    std::cout << "Barycentric coordinates computed successfully\n";
}



Point operator*(double scalar, const Point& point)
{
    return Point(scalar * point.x(), scalar * point.y(), scalar * point.z());
}

Point operator+(const Point& p1, const Point& p2)
{
    return Point(p1.x() + p2.x(), p1.y() + p2.y(), p1.z() + p2.z());
}



std::map<int, std::vector<Point>> meshUtility::initialWrapping(Polygon_mesh octahedronSource, Polygon_mesh octahedronTarget, Polygon_mesh &sourceMesh ,std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> &barycentricCoordinatesSource)
{

    std::map<int, std::vector<Point>> WrappedPoints;

    for(const auto &[faceID, baryCoords]: barycentricCoordinatesSource)
    {
        Polygon_mesh::Face_index targetFace = Polygon_mesh::Face_index(std::stoi(faceID));
        auto halfedge_descriptor = halfedge(targetFace, octahedronTarget);
        auto verticesAround = vertices_around_face(halfedge_descriptor, octahedronTarget);

        //get points target triangle
        auto vertexIter = verticesAround.begin();
        Point A = get(CGAL::vertex_point, octahedronTarget, *vertexIter);
        vertexIter++;
        Point B = get(CGAL::vertex_point, octahedronTarget, *vertexIter);
        vertexIter++;
        Point C = get(CGAL::vertex_point, octahedronTarget, *vertexIter);

        //calculating the normal of the target triangle
        Vector_3 normal = CGAL::cross_product(B - A, C - A);
        normal = normal / CGAL::sqrt(normal.squared_length());


        //process each vertex in the source
        for(const auto &[vertexID, baryCoords, distance]: baryCoords)
        {
            double u = baryCoords[0];
            double v = baryCoords[1];
            double w = baryCoords[2];

            Point wrappedPoint = u * A + v * B + w * C;
            wrappedPoint = wrappedPoint - distance * normal;

            WrappedPoints[vertexID].push_back(wrappedPoint);
        }
    }


    std::cout << "Initial wrapping completed successfully\n";
    

    return WrappedPoints;
}

// CGAL libary, 2023, Polygon_mesh_processing/shape_smoothing_example.cpp, Available from:
// https://doc.cgal.org/latest/Polygon_mesh_processing/Polygon_mesh_processing_2shape_smoothing_example_8cpp-example.html
// was used as a reference for the following function
void meshUtility::relaxMesh(Polygon_mesh &polygon)
{
    //constraint edges with a dihedral angle over 60 degrees
    typedef boost::property_map<Polygon_mesh, CGAL::edge_is_feature_t>::type EIFMap;
    EIFMap eif = get(CGAL::edge_is_feature, polygon);
    PMP::detect_sharp_edges(polygon, 90, eif);

    int sharp_counter = 0;
    

    PMP::angle_and_area_smoothing(polygon, PMP::parameters::number_of_iterations(100).use_safety_constraints(false).edge_is_constrained_map(eif));
    
    std::cout << "Mesh relaxed successfully\n";

}