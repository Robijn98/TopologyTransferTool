#include <gtest/gtest.h>
#include "curve.h"
#include <vector>
#include <filesystem> 

TEST(Curve, loadCurve) {

    Curve curve;
    std::vector<Point> validCurve;   
    std::vector<Point> invalidCurve;   

    EXPECT_TRUE(curve.loadCurve("../test_files/deformed_sphere_line.obj", validCurve));
    EXPECT_FALSE(validCurve.empty());

    EXPECT_FALSE(curve.loadCurve("../test_files/doesntexist.obj", invalidCurve));
    EXPECT_TRUE(invalidCurve.empty());
}