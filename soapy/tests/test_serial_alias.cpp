#include "Check.hpp"
#include "UhdSerial.hpp"

#include <limits>
#include <stdexcept>

using namespace PlutoSky;

int main()
{
    CHECK_STR(uhdSerialAlias("000a35000122"), "64206845");
    CHECK_STR(uhdSerialAlias("000a35000123"), "632066b2");
    CHECK_STR(uhdSerialAlias("not-hex-serial"), "5802edf9");

    for (const auto &serial : {"000a35000122", "not-hex-serial", ""})
    {
        const auto alias = uhdSerialAlias(serial);
        CHECK(alias.size() == 8);
        CHECK(std::stoi(alias, nullptr, 16) <= std::numeric_limits<int>::max());
    }
    bool overflow = false;
    try { (void)std::stoi("000a35000122", nullptr, 16); }
    catch (const std::out_of_range &) { overflow = true; }
    CHECK(overflow);

    return PlutoSkyTest::summarize("test_serial_alias");
}
