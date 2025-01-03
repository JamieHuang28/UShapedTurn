#include "gtest/gtest.h"

namespace {
    TEST(Exp1Test, add)
    {
        double res;
        res = 1.0 + 2.0;
        ASSERT_NEAR(res, 3.0, 1.0e-11);
    }

    TEST(Exp1Test, subtract)
    {
        double res;
        res = 1.0 - 2.0;
        ASSERT_NEAR(res, -1.0, 1.0e-11);
    }
}
