#include <filesystem>
#include <gtest/gtest.h>
#include "Convert.h"

TEST(Convert, file)
{
    Convert file;
    EXPECT_TRUE(file.open_file("test.obj"));
}


