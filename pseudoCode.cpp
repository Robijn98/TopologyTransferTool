//goal

//piece of code that takes two meshes as input, one source and one target
//curves will be placed on both models to guide wrap optimazation
//process meshes into a easy to read file (OBJ or libAssimp)
//source wraps around target while maintaining patch layout source


//Jian Chang feedback
'''
well, the key is to have a isometric preserved mapping from source to target. unfortunately, 
such mapping is not always possible, therefore we need to find some good approximation. 
barycentric coordinate can do it if you can break both source and target into similar tiles 
(could be a low detailed mesh or some volumes) and map all points in one tile of the source to the corresponding one of the target.
 it may not work out well if you treat the whole mesh together.  
 Also, it will be hard to have both tile partitions in the same number and with good correspondence- 
 so the tiling may need to be done manually or design some clever way to do it. 
 
AGenetic Isometric Shape Correspondence Algorithm with Adaptive Sampling YUSUF SAHILLIOĞLU, MiddleEast Technical University
 
This paper may provide some details to explain how to find the shape correspondence.  
 
Here is another paper to show how to simplify a quad mesh: 
Integer-Grid Maps for Reliable Quad Meshing David Bommes1 Marcel Campen2 Hans-Christian Ebke2 Pierre Alliez1 Leif Kobbelt
'''




FUNCTION loadMeshes(inputMesh_x, inputMesh_y)
    loadMesh(inputMesh_x) //load mesh from file
    loadMesh(inputMesh_y) //load mesh from file


//https://github.com/assimp/assimp //libray to convert mesh files //CGAL libary, easier to use than assimp //https://doc.cgal.org/latest/Manual/packages.html#PartGeometryProcessing
FUNCTION convertX(inputMesh_x)
    if format works with assimp
        use assimp libary to convert inputMesh to a easy to read file
    else
        give error message to user
    endif
    return convertedMesh_x

FUNCTION convertY(inputMesh_y)
    if format works with assimp
        use assimp libary to convert inputMesh to a easy to read file
    else
        give error message to user
    endif
    return convertedMesh_y

//https://en.wikipedia.org/wiki/Discretization //explains discretization
FUNCTION discretizeCurves(sourceMesh, guidanceCurves)
    //discretize guidance curves to get a set of points
    //points will be used to guide the wrap optimization
    for curve in guidanceCurves
        discretize curve
        store discretized curve
    endfor
    return discretizedCurves


//https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf   //explains how to use barycentric coordinates in a 3D setting
//https://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__locate__grp.html#ga72808d7750dab4989b4613dfa1dd46fc 
FUNCTION computeBaryCentricCoordinates(convertedMesh_x, discretizedCurves)
    //compute barycentric coordinates for each vertex in the source mesh
    //barycentric coordinates are used to determine the position of a point in a triangle
    for vertex in convertedMesh_x
        find closest points on two closest discretized curves
        compute barycentric coordinates for the vertex
        store barycentric coordinates for the vertex
    endfor

    return barycentricCoordinates_x


FUNCTION projectPoints(discretizedCurves, curves_Y)
    //project points from the discretized curves onto the target curves
    for points in discretizedCurves
        find corosponding point on the target curve
        project point
        store projected point
    endfor
    return projectedPoints


//http://geometry.cs.cmu.edu/ddgshortcourse/notes/01_DiscreteLaplaceOperators.pdf
FUNCTION discreteLaplacians(barycentricCoordinates_x, projectedPoints)
    //compute discrete laplacians for each vertex in the source mesh
    //laplacians are used to maintain the patch layout of the source mesh
    

'''
Summary of Steps Involving Discrete Laplacians:
Compute Barycentric Coordinates: For each vertex in the source mesh, 
you compute its barycentric coordinates relative to the guidance curves (as described in your pseudocode). 
These coordinates will help define how each vertex is mapped to the target geometry.

Define Boundary Conditions: The boundary vertices are fixed by the source curves, 
and these act as boundary conditions for the Laplacian computation.

Compute Discrete Laplacians: Using cotangent weights or other weighting schemes, 
compute the discrete Laplacian for the interior vertices, ensuring that the geometry of the mesh transitions smoothly between boundary points.

Solve for Interior Points: The interior points are computed by solving a system of equations that enforces smoothness, 
ensuring that the transformation respects both the geometry of the source and target meshes.

Validate Mapping: Finally, validate that the computed solution respects the initial mapping and the constraints imposed by the source curves. 

//source: chatgpt ; how do discrete laplacians work together with barycentric coordinates
'''

    return laplacians_x


FUNCTION iteriorPoints(laplacians_x, boundaryConditions)
    //compute interior points for each vertex in the source mesh
    //interior points are used to maintain the patch layout of the source mesh
    for vertex in laplacians_x
        solve for interior points
        store interior points
    endfor
    return interiorPoints_x

FUNCTION validateMesh(convertedMesh_x, interiorPoints_x)
    //validate the converted meshes
    //check for any errors or inconsistencies

    //NOTE TO SELF: RESEARCH HOW TO VALIDATE MESHES


FUNCTION setIteriorPoints(interiorPoints_x, laplacians_x)
    //set interior points for each vertex in the source mesh
    //interior points are used to maintain the patch layout of the source mesh
    //write to file
    for vertex in interiorPoints_x
        set interior points
        write to file


FUNCTION convertToMeshFile(outputFile, format)
    //convert the output file to a mesh file format
    //write to file
    convert file to given format using assimp    convertToMeshFile(outputFile)