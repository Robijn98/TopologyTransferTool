#include <iostream>
#include <QApplication>
#include <iostream>
#include "NGLScene.h"
#include "mesh.h"
#include "meshUtility.h"
#include "curve.h"
#include "output.h"

int main(int argc, char **argv)
{
    Mesh mesh;
    Polygon_mesh polygonSource;
    Polygon_mesh polygonTarget;
    Polygon_mesh polygonSource_original;
    std::string filenameSource = "files/cube_mesh.obj";
    std::string filenameTarget = "files/oval.obj";

    mesh.loadMesh(filenameSource, polygonSource);
    mesh.loadMesh(filenameSource, polygonSource_original);


    std::cout << "Mesh loaded successfully\n";
    mesh.validateMesh(polygonSource);
    mesh.triangulateMesh(polygonSource);

    mesh.loadMesh(filenameTarget, polygonTarget);
    //mesh.validateMesh(polygonTarget);
    //mesh.triangulateMesh(polygonTarget);


    //mesh utility
    meshUtility meshUtil;
    std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> barycentric_coordinates;
    std::map<std::string, std::array<Point, 3>> triangles;
    Polygon_mesh debugMesh;
    Polygon_mesh debugMesh2;

    triangles = meshUtil.divideMeshForBarycentricComputing(polygonSource, debugMesh,  1e-6, 1e-6);


    std::map<std::string, std::array<Point, 3>> trianglesTarget;
    trianglesTarget = meshUtil.divideMeshForBarycentricComputing(polygonTarget, debugMesh2,  1e-6, 1e-6);

    meshUtil.computeBarycentric_coordinates(polygonSource, debugMesh, triangles, barycentric_coordinates);


    std::map<int, std::vector<Point>> WrappedPoints;
    WrappedPoints = meshUtil.initialWrapping(debugMesh, debugMesh2, polygonSource, barycentric_coordinates);
    
   
    //output
    Output output;
    output.writeMesh("files/output.obj", polygonSource);
    output.updateFileWithWrap("files/output.obj", WrappedPoints);
    

    //colours
    mesh.assignColors(polygonSource, "files/output.obj");
    std::string vertexColorsFile = "vertex_colors.txt";
    



    //viewer
    if(!CGAL::is_triangle_mesh(polygonSource))
    {
    throw std::runtime_error("Input mesh is not triangulated");
    }
    else
    {
    std::cout << "Input mesh is triangulated, continuing with viewer\n";
    }

    
    
    QApplication app(argc, argv);
    QSurfaceFormat format;
    // set the number of samples for multisampling
    // will need to enable glEnable(GL_MULTISAMPLE); once we have a context
    format.setSamples(4);
    #if defined( __APPLE__)
        // at present mac osx Mountain Lion only supports GL3.2
        // the new mavericks will have GL 4.x so can change
        format.setMajorVersion(4);
        format.setMinorVersion(1);
    #else
        // with luck we have the latest GL version so set to this
        format.setMajorVersion(4);
        format.setMinorVersion(3);
    #endif
    // now we are going to set to CoreProfile OpenGL so we can't use and old Immediate mode GL
    format.setProfile(QSurfaceFormat::CoreProfile);
    // now set the depth buffer to 24 bits
    format.setDepthBufferSize(24);

    NGLScene window;
    //NGLScene window(oname);
    // and set the OpenGL format
    window.setFormat(format);
    // we can now query the version to see if it worked
    std::cout<<"Profile is "<<format.majorVersion()<<" "<<format.minorVersion()<<"\n";
    // set the window size
    window.resize(1024, 720);
    // and finally show
    window.show();

    return app.exec();
    
}

