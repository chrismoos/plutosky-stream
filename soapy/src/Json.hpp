// Minimal JSON encoder and decoder for the control protocol.
//
// Integers keep 64-bit fidelity because the protocol sends sample counts,
// which outgrow a double.

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace PlutoSky {

class Json {
public:
    enum class Type { Null, Bool, Int, Real, String, Array, Object };

    Json();
    Json(bool value);
    Json(int value);
    Json(unsigned value);
    Json(long value);
    Json(unsigned long value);
    Json(long long value);
    Json(unsigned long long value);
    Json(double value);
    Json(const char *value);
    Json(const std::string &value);

    static Json object();
    static Json array();

    // Throws std::runtime_error on malformed input.
    static Json parse(const std::string &text);

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isBool() const { return type_ == Type::Bool; }
    bool isNumber() const { return type_ == Type::Int || type_ == Type::Real; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBool(bool fallback = false) const;
    int64_t asInt(int64_t fallback = 0) const;
    double asDouble(double fallback = 0.0) const;
    std::string asString(const std::string &fallback = std::string()) const;

    bool has(const std::string &key) const;
    const Json &operator[](const std::string &key) const;
    const Json &at(size_t index) const;
    size_t size() const;
    std::vector<std::string> keys() const;
    std::vector<std::string> asStringVector() const;

    void set(const std::string &key, const Json &value);
    void push(const Json &value);

    std::string dump() const;

private:
    Type type_;
    bool bool_ = false;
    int64_t int_ = 0;
    double real_ = 0.0;
    std::string str_;
    std::vector<Json> arr_;
    std::map<std::string, Json> obj_;
};

}  // namespace PlutoSky
