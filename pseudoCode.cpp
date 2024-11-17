//goal

//piece of code that takes two meshes as input, one source and one target
//curves will be placed on both models to guide wrap optimazation
//process meshes into a easy to read file (OBJ or libAssimp)
//source wraps around target while maintaining patch layout source


//https://www.sciencedirect.com/topics/engineering/iterative-closest-point-algorithm#:~:text=Iterative%20closest%20point%20(ICP)%20is,be%20identified%20in%20both%20images.


FUNCTION loadMeshes(inputMesh_x, inputMesh_y)
    loadMesh(inputMesh_x) //load mesh from file
    loadMesh(inputMesh_y) //load mesh from file


FUNCTION convertX(inputMesh_x)
    if format works with assimp
        use assimp libary to convert inputMesh to a easy to read file
    endif
    else
        give error message to user

    return convertedMesh_x

FUNCTION convertY(inputMesh_y)
    if format works with assimp
        use assimp libary to convert inputMesh to a easy to read file
    endif
    else
        give error message to user

    return convertedMesh_y

FUNCTION computeBaryCentricCoordinates(convertedMesh_x, convertedMesh_y)
    //compute barycentric coordinates for each vertex in the source mesh
    //barycentric coordinates are used to determine the position of a point in a triangle

    return barycentricCoordinates_x

//discretize guidance curves
FUNCTION discretizeCurves(sourceMesh, guidanceCurves)
    //discretize guidance curves to get a set of points
    //points will be used to guide the wrap optimization

    return discretizedCurves

FUNCTION projectPoints(discretizedCurves, curves_Y)
    //project points from the discretized curves onto the target curves

    return projectedPoints

FUNCTION discreteLaplacians(barycentricCoordinates_x, projectedPoints)
    //compute discrete laplacians for each vertex in the source mesh
    //laplacians are used to maintain the patch layout of the source mesh

    return laplacians_x


FUNCTION iteriorPoints(laplacians_x, boundaryConditions)
    //compute interior points for each vertex in the source mesh
    //interior points are used to maintain the patch layout of the source mesh

    return interiorPoints_x

FUNCTION validateMesh(convertedMesh_x, interiorPoints_x)
    //validate the converted meshes
    //check for any errors or inconsistencies

FUNCTION setIteriorPoints(interiorPoints_x, laplacians_x)
    //set interior points for each vertex in the source mesh
    //interior points are used to maintain the patch layout of the source mesh
    //write to file