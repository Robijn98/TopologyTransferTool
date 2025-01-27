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
    //all changable parameters
    std::string filenameSource = "files/sphere.obj";
    std::string filenameTarget = "files/deformedSphere.obj";
    double yThresholdSource = 1e-6;
    double zThresholdSource = 1e-6;
    double yThresholdTarget = 0.5;
    double zThresholdTarget = 0.5;
    

    //mesh processing, loading and validation
    Mesh mesh;
    Polygon_mesh polygonSource;
    Polygon_mesh polygonTarget;
    Polygon_mesh polygonSourceOriginal;

    mesh.loadMesh(filenameSource, polygonSource);
    mesh.loadMesh(filenameSource, polygonSourceOriginal);

    mesh.validateMesh(polygonSource);
    mesh.triangulateMesh(polygonSource);

    mesh.loadMesh(filenameTarget, polygonTarget);
    mesh.validateMesh(polygonTarget);
    mesh.triangulateMesh(polygonTarget);


    //mesh utility, barycentric coordinates, wrapping
    meshUtility meshUtil;
    std::map<std::string, std::vector<std::tuple<int, std::array<double, 3>, double>>> barycentricCoordinates;
    std::map<std::string, std::array<Point, 3>> triangles;
    Polygon_mesh debugMesh;
    Polygon_mesh debugMesh2;

    triangles = meshUtil.divideMeshForBarycentricComputing(polygonSource, debugMesh, zThresholdSource, yThresholdSource);

    std::map<std::string, std::array<Point, 3>> trianglesTarget;
    trianglesTarget = meshUtil.divideMeshForBarycentricComputing(polygonTarget, debugMesh2, zThresholdTarget, yThresholdTarget);

    meshUtil.computeBarycentricCoordinates(polygonSource, debugMesh, triangles, barycentricCoordinates);

    std::map<int, std::vector<Point>> WrappedPoints;
    WrappedPoints = meshUtil.initialWrapping(debugMesh, debugMesh2, polygonSource, barycentricCoordinates);
    
    //output writing obj file //dont change this
    std::string filenameOutput = "files/output.obj";
    Output output;
    output.writeMesh(filenameOutput, polygonSource);
    output.updateFileWithWrap(filenameOutput, WrappedPoints);
    
    //assign colours for viewer
    mesh.assignColors(polygonSource, filenameOutput);    

    //viewer
    //check if mesh is triangulated
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

