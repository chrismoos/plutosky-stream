#include "Check.hpp"
#include "StatusLog.hpp"

using namespace PlutoSky;

int main()
{
    StatusLogLimiter limiter(1000);
    uint64_t folded = 99;
    CHECK(limiter.due(5000, folded));
    CHECK(folded == 0);
    CHECK(!limiter.due(5001, folded));
    CHECK(!limiter.due(5500, folded));
    CHECK(!limiter.due(5999, folded));
    CHECK(limiter.due(6000, folded));
    CHECK(folded == 3);
    CHECK(limiter.due(9000, folded));
    CHECK(folded == 0);

    // Every 1.2 ms for 10 s, the rate a starved 1.083 MS/s stream reports at.
    StatusLogLimiter steady(1000);
    int lines = 0;
    for (int64_t t = 0; t < 10000; t += 1)
        if (steady.due(t, folded)) lines++;
    CHECK(lines == 10);

    CHECK(!txStarved(5000, 4500, true, 1000));
    CHECK(txStarved(5500, 4500, true, 1000));
    CHECK(txStarved(9000, 4500, true, 1000));
    CHECK(!txStarved(9000, 4500, false, 1000));
    CHECK(!txStarved(9000, 0, true, 1000));

    return PlutoSkyTest::summarize("test_status_log");
}
