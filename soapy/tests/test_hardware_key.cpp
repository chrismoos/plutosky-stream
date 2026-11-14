#include "Check.hpp"
#include "HardwareKey.hpp"

using namespace PlutoSky;

int main()
{
    CHECK_STR(hardwareKey({}, "000a35000122"), "000a35000122");
    CHECK_STR(hardwareKey({}, ""), "PlutoSky R1");
    CHECK_STR(hardwareKey({{"hardware", "B200"}}, "000a35000122"), "B200");
    CHECK_STR(hardwareKey({{"hardware", "B210"}}, ""), "B210");
    CHECK_STR(hardwareKey({{"hardware", ""}}, "000a35000122"), "000a35000122");

    DeviceArgs found{{"driver", "plutosky"}, {"addr", "192.168.200.10"}};
    keepHardwareArg(found, {{"driver", "plutosky"}});
    CHECK(found.count("hardware") == 0);
    keepHardwareArg(found, {{"driver", "plutosky"}, {"hardware", "B200"}});
    CHECK_STR(found.at("hardware"), "B200");
    CHECK_STR(hardwareKey(found, "000a35000122"), "B200");

    return PlutoSkyTest::summarize("test_hardware_key");
}
