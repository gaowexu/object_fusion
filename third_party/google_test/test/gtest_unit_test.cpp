///
/// @copyright Copyright (C) 2021-2025, Gawei Xu
///
/// @author Gaowei Xu (gaowexu1991@gmail.com)
/// @date 2023-03-01
///

#include <gtest/gtest.h>

TEST(HelloTest, BasicAssertions)
{
    EXPECT_STRNE("hello", "world");
    EXPECT_EQ(6 * 7, 42);
    EXPECT_FLOAT_EQ(0.0000000001F, 0.000000001F);
}