#include <iostream>
#include <QApplication>
#include <iostream>
#include "NGLScene.h"
#include "mesh.h"
#include "meshUtility.h"
#include "curve.h"

int main(int argc, char **argv)
{
    Mesh mesh;
    Polygon_mesh polygonSource;
    Polygon_mesh polygonTarget;
    std::string filenameSource = "files/normal_sphere_export.obj";
    std::string filenameTarget = "files/deformed_sphere_line.obj";

    mesh.loadMesh(filenameSource, polygonSource);
    mesh.validateMesh(polygonSource);
    mesh.triangulateMesh(polygonSource);

    mesh.loadMesh(filenameTarget, polygonTarget);
    mesh.validateMesh(polygonTarget);
    mesh.triangulateMesh(polygonTarget);

    //curves
    Curve curve;
    std::vector<Point> curveRef;
    std::vector<Point> discretizedCurve;
    std::vector<Point> projectedCurve;

    curve.loadCurve("files/deformed_sphere_line.obj", curveRef);
    int numPoints = 10; 
    discretizedCurve = curve.discretizeCurve(curveRef, discretizedCurve, numPoints);
    std::cout << "Curve discretized successfully." << std::endl;
    projectedCurve = curve.projectPoints(discretizedCurve, curveRef, projectedCurve);
    std::cout << "Points projected successfully onto the target curve." << std::endl;

    //mesh utility
    meshUtility meshUtil;
    std::vector<double> barycentric_coordinates;
    meshUtil.computeBarycentric_coordinates(polygonSource, curveRef, barycentric_coordinates);
    meshUtil.initialWrapping(polygonSource, polygonTarget, curveRef, barycentric_coordinates);
    
    
    //mesh.writeMesh("files/output.obj", polygonSource);

    //viewer
    if(!CGAL::is_triangle_mesh(polygonSource))
    {
    std::cout << "Input mesh is not triangulated." << std::endl;
    throw std::runtime_error("Input mesh is not triangulated.");
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
    // now we are going to create our scene window
    std::string oname("files/output.obj");

    if(argc ==2)
    {
        oname=argv[1];
    }
    else if(argc == 3)
    {
        oname=argv[1];
    }
    NGLScene window(oname);
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