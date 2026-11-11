// Assertions for the module tests, small enough not to need a framework.

#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace PlutoSkyTest {

inline int &failures()
{
    static int count = 0;
    return count;
}

inline int &checks()
{
    static int count = 0;
    return count;
}

inline void report(bool ok, const char *file, int line, const std::string &what)
{
    checks()++;
    if (ok) return;
    failures()++;
    std::fprintf(stderr, "%s:%d: FAIL %s\n", file, line, what.c_str());
}

inline int summarize(const char *name)
{
    std::printf("%s: %d checks, %d failures\n", name, checks(), failures());
    return failures() == 0 ? 0 : 1;
}

}  // namespace PlutoSkyTest

#define CHECK(cond) \
    PlutoSkyTest::report((cond), __FILE__, __LINE__, #cond)

#define CHECK_EQ(a, b)                                                                  \
    do {                                                                                \
        const auto lhs_ = (a);                                                          \
        const auto rhs_ = (b);                                                          \
        PlutoSkyTest::report(lhs_ == rhs_, __FILE__, __LINE__,                          \
                             std::string(#a " == " #b " (") + std::to_string(lhs_)      \
                                 + " vs " + std::to_string(rhs_) + ")");                \
    } while (false)

#define CHECK_NEAR(a, b, tol)                                                           \
    do {                                                                                \
        const double lhs_ = static_cast<double>(a);                                     \
        const double rhs_ = static_cast<double>(b);                                     \
        PlutoSkyTest::report(std::fabs(lhs_ - rhs_) <= (tol), __FILE__, __LINE__,        \
                             std::string(#a " ~ " #b " (") + std::to_string(lhs_)       \
                                 + " vs " + std::to_string(rhs_) + ")");                \
    } while (false)

#define CHECK_STR(a, b)                                                                 \
    do {                                                                                \
        const std::string lhs_ = (a);                                                   \
        const std::string rhs_ = (b);                                                   \
        PlutoSkyTest::report(lhs_ == rhs_, __FILE__, __LINE__,                          \
                             std::string(#a " == " #b " (\"") + lhs_ + "\" vs \""       \
                                 + rhs_ + "\")");                                       \
    } while (false)
