#include "Check.hpp"
#include "IntegerHz.hpp"
#include "Json.hpp"

#include <limits>
#include <stdexcept>

using namespace PlutoSky;

static bool rejects(double value)
{
    try { (void)integerHz(value, "frequency"); }
    catch (const std::invalid_argument &) { return true; }
    return false;
}

int main()
{
    CHECK_EQ(integerHz(1852500000.0, "frequency"), 1852500000LL);
    CHECK_EQ(integerHz(1852500000.25, "frequency"), 1852500000LL);
    CHECK_EQ(integerHz(1852500000.75, "frequency"), 1852500001LL);
    CHECK_STR(Json(integerHz(1852500000.25, "frequency")).dump(), "1852500000");
    CHECK(rejects(-1.0));
    CHECK(rejects(std::numeric_limits<double>::infinity()));
    CHECK(rejects(std::numeric_limits<double>::quiet_NaN()));
    CHECK(rejects(static_cast<double>(std::numeric_limits<long long>::max())));
    return PlutoSkyTest::summarize("test_integer_hz");
}
