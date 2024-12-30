#include <gtest/gtest.h>
#include "curve.h"
#include <vector>
#include <filesystem> 


// TEST(Curve, loadCurve) {

//     Curve curve;
//     std::vector<Point> validCurve;   
//     std::vector<Point> invalidCurve;   

//     EXPECT_TRUE(curve.loadCurve("files/deformed_sphere_line.obj", validCurve));
//     EXPECT_FALSE(validCurve.empty());

//     EXPECT_FALSE(curve.loadCurve("files/doesntexist.obj", invalidCurve));
//     EXPECT_TRUE(invalidCurve.empty());
// }


// TEST(Curve, DiscretizeCurve) {

//     Curve curve;
//     std::vector<Point> curveIn;   
//     std::vector<Point> curveOut;   

//     curve.loadCurve("files/deformed_sphere_line.obj", curveIn);
//     curve.discretizeCurve(curveIn, curveOut, 20);
//     EXPECT_EQ(curveOut.size(), 20);
// }

// TEST(Curve, ProjectPoints) 
// {

//     Curve curve;
//     std::vector<Point> curveSource;   
//     std::vector<Point> curveTarget;
//     std::vector<Point> curveOutSource;
//     std::vector<Point> curveOutTarget;   
//     std::vector<Point> curveOut;   

//     curve.loadCurve("files/deformed_sphere_line.obj", curveTarget);
//     curve.loadCurve("files/normal_sphere_export.obj", curveSource);
//     curve.discretizeCurve(curveSource, curveOutSource, 5);
//     curve.discretizeCurve(curveTarget, curveOutTarget, 5);
//     curve.projectPoints(curveOutSource, curveOutTarget, curveOut);
//     EXPECT_EQ(curveOut.size(), curveOutSource.size());
// }