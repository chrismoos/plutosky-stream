#include "Check.hpp"
#include "Json.hpp"

using PlutoSky::Json;

static void testScalars()
{
    CHECK(Json::parse("null").isNull());
    CHECK_EQ(Json::parse("true").asBool(), true);
    CHECK_EQ(Json::parse("-17").asInt(), -17);
    CHECK_NEAR(Json::parse("1.5e3").asDouble(), 1500.0, 1e-9);
    CHECK_STR(Json::parse("\"a\\nb\\u00e9\"").asString(), "a\nb\xc3\xa9");
}

static void testLargeIntegers()
{
    // Sample counts outgrow a double, so they must survive as integers.
    const int64_t ticks = 9007199254740993LL;
    Json message = Json::object();
    message.set("ticks", Json(static_cast<long long>(ticks)));
    CHECK_EQ(Json::parse(message.dump())["ticks"].asInt(), ticks);
}

static void testObjectsAndArrays()
{
    const Json value = Json::parse(
        "{\"stream_id\": 3, \"formats\": [\"cs12\", \"cs8\"], \"nested\": {\"a\": [1, 2]}}");
    CHECK_EQ(value["stream_id"].asInt(), 3);
    CHECK_EQ(value["formats"].size(), size_t(2));
    CHECK_STR(value["formats"].at(1).asString(), "cs8");
    CHECK_EQ(value["nested"]["a"].at(0).asInt(), 1);
    CHECK(value["missing"].isNull());
    CHECK(value["missing"]["deeper"].isNull());
}

static void testRoundTrip()
{
    Json request = Json::object();
    request.set("id", Json(42));
    request.set("op", Json("open_stream"));
    Json chans = Json::array();
    chans.push(Json(0));
    chans.push(Json(1));
    request.set("chans", chans);
    request.set("host_ip", Json("192.168.2.10"));

    const Json parsed = Json::parse(request.dump());
    CHECK_EQ(parsed["id"].asInt(), 42);
    CHECK_STR(parsed["op"].asString(), "open_stream");
    CHECK_EQ(parsed["chans"].size(), size_t(2));
    CHECK_STR(parsed["host_ip"].asString(), "192.168.2.10");
}

static void testMalformed()
{
    bool threw = false;
    try { Json::parse("{\"a\": }"); } catch (const std::exception &) { threw = true; }
    CHECK(threw);

    threw = false;
    try { Json::parse("[1, 2"); } catch (const std::exception &) { threw = true; }
    CHECK(threw);

    threw = false;
    try { Json::parse("{} trailing"); } catch (const std::exception &) { threw = true; }
    CHECK(threw);
}

int main()
{
    testScalars();
    testLargeIntegers();
    testObjectsAndArrays();
    testRoundTrip();
    testMalformed();
    return PlutoSkyTest::summarize("test_json");
}
