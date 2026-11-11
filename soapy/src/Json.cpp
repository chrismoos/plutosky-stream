#include "Json.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace PlutoSky {

static const Json kNull;

Json::Json() : type_(Type::Null) {}
Json::Json(bool value) : type_(Type::Bool), bool_(value) {}
Json::Json(int value) : type_(Type::Int), int_(value) {}
Json::Json(unsigned value) : type_(Type::Int), int_(static_cast<int64_t>(value)) {}
Json::Json(long value) : type_(Type::Int), int_(static_cast<int64_t>(value)) {}
Json::Json(unsigned long value) : type_(Type::Int), int_(static_cast<int64_t>(value)) {}
Json::Json(long long value) : type_(Type::Int), int_(static_cast<int64_t>(value)) {}
Json::Json(unsigned long long value) : type_(Type::Int), int_(static_cast<int64_t>(value)) {}
Json::Json(double value) : type_(Type::Real), real_(value) {}
Json::Json(const char *value) : type_(Type::String), str_(value ? value : "") {}
Json::Json(const std::string &value) : type_(Type::String), str_(value) {}

Json Json::object()
{
    Json out;
    out.type_ = Type::Object;
    return out;
}

Json Json::array()
{
    Json out;
    out.type_ = Type::Array;
    return out;
}

bool Json::asBool(bool fallback) const
{
    if (type_ == Type::Bool) return bool_;
    if (type_ == Type::Int) return int_ != 0;
    return fallback;
}

int64_t Json::asInt(int64_t fallback) const
{
    if (type_ == Type::Int) return int_;
    if (type_ == Type::Real) return static_cast<int64_t>(std::llround(real_));
    if (type_ == Type::Bool) return bool_ ? 1 : 0;
    return fallback;
}

double Json::asDouble(double fallback) const
{
    if (type_ == Type::Real) return real_;
    if (type_ == Type::Int) return static_cast<double>(int_);
    if (type_ == Type::Bool) return bool_ ? 1.0 : 0.0;
    return fallback;
}

std::string Json::asString(const std::string &fallback) const
{
    if (type_ == Type::String) return str_;
    return fallback;
}

bool Json::has(const std::string &key) const
{
    return type_ == Type::Object && obj_.count(key) != 0;
}

const Json &Json::operator[](const std::string &key) const
{
    if (type_ != Type::Object) return kNull;
    const auto it = obj_.find(key);
    return it == obj_.end() ? kNull : it->second;
}

const Json &Json::at(size_t index) const
{
    if (type_ != Type::Array || index >= arr_.size()) return kNull;
    return arr_[index];
}

size_t Json::size() const
{
    if (type_ == Type::Array) return arr_.size();
    if (type_ == Type::Object) return obj_.size();
    if (type_ == Type::String) return str_.size();
    return 0;
}

std::vector<std::string> Json::keys() const
{
    std::vector<std::string> out;
    if (type_ == Type::Object)
    {
        for (const auto &entry : obj_) out.push_back(entry.first);
    }
    return out;
}

std::vector<std::string> Json::asStringVector() const
{
    std::vector<std::string> out;
    for (size_t i = 0; i < size(); i++) out.push_back(at(i).asString());
    return out;
}

void Json::set(const std::string &key, const Json &value)
{
    if (type_ != Type::Object)
    {
        type_ = Type::Object;
        obj_.clear();
    }
    obj_[key] = value;
}

void Json::push(const Json &value)
{
    if (type_ != Type::Array)
    {
        type_ = Type::Array;
        arr_.clear();
    }
    arr_.push_back(value);
}

static void dumpString(const std::string &in, std::string &out)
{
    out += '"';
    for (const unsigned char c : in)
    {
        switch (c)
        {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20)
            {
                char esc[8];
                std::snprintf(esc, sizeof(esc), "\\u%04x", c);
                out += esc;
            }
            else
            {
                out += static_cast<char>(c);
            }
        }
    }
    out += '"';
}

static void dumpInto(const Json &value, std::string &out)
{
    char buf[64];
    switch (value.type())
    {
    case Json::Type::Null:
        out += "null";
        break;
    case Json::Type::Bool:
        out += value.asBool() ? "true" : "false";
        break;
    case Json::Type::Int:
        std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(value.asInt()));
        out += buf;
        break;
    case Json::Type::Real:
        std::snprintf(buf, sizeof(buf), "%.17g", value.asDouble());
        out += buf;
        break;
    case Json::Type::String:
        dumpString(value.asString(), out);
        break;
    case Json::Type::Array:
        out += '[';
        for (size_t i = 0; i < value.size(); i++)
        {
            if (i) out += ',';
            dumpInto(value.at(i), out);
        }
        out += ']';
        break;
    case Json::Type::Object:
    {
        out += '{';
        bool first = true;
        for (const auto &key : value.keys())
        {
            if (!first) out += ',';
            first = false;
            dumpString(key, out);
            out += ':';
            dumpInto(value[key], out);
        }
        out += '}';
        break;
    }
    }
}

std::string Json::dump() const
{
    std::string out;
    dumpInto(*this, out);
    return out;
}

namespace {

class Parser {
public:
    explicit Parser(const std::string &text) : text_(text) {}

    Json parseValue()
    {
        skipSpace();
        if (pos_ >= text_.size()) fail("unexpected end of input");
        const char c = text_[pos_];
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return Json(parseString());
        if (c == 't' || c == 'f') return Json(parseBool());
        if (c == 'n') { expect("null"); return Json(); }
        return parseNumber();
    }

    void finish()
    {
        skipSpace();
        if (pos_ != text_.size()) fail("trailing data after the JSON value");
    }

private:
    [[noreturn]] void fail(const char *why) const
    {
        throw std::runtime_error(std::string("json: ") + why + " at offset "
                                 + std::to_string(pos_));
    }

    void skipSpace()
    {
        while (pos_ < text_.size())
        {
            const char c = text_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') pos_++;
            else break;
        }
    }

    void expect(const char *word)
    {
        const size_t len = std::char_traits<char>::length(word);
        if (text_.compare(pos_, len, word) != 0) fail("unexpected token");
        pos_ += len;
    }

    bool parseBool()
    {
        if (text_[pos_] == 't') { expect("true"); return true; }
        expect("false");
        return false;
    }

    std::string parseString()
    {
        if (text_[pos_] != '"') fail("expected a string");
        pos_++;
        std::string out;
        while (true)
        {
            if (pos_ >= text_.size()) fail("unterminated string");
            const char c = text_[pos_++];
            if (c == '"') break;
            if (c != '\\')
            {
                out += c;
                continue;
            }
            if (pos_ >= text_.size()) fail("unterminated escape");
            const char esc = text_[pos_++];
            switch (esc)
            {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': out += parseUnicodeEscape(); break;
            default: fail("unknown escape");
            }
        }
        return out;
    }

    // Decodes one \uXXXX escape, and a surrogate pair when one follows, to UTF-8.
    std::string parseUnicodeEscape()
    {
        uint32_t code = parseHex4();
        if (code >= 0xD800 && code <= 0xDBFF && pos_ + 1 < text_.size()
            && text_[pos_] == '\\' && text_[pos_ + 1] == 'u')
        {
            const size_t mark = pos_;
            pos_ += 2;
            const uint32_t low = parseHex4();
            if (low >= 0xDC00 && low <= 0xDFFF)
            {
                code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
            }
            else
            {
                pos_ = mark;
            }
        }
        std::string out;
        if (code < 0x80)
        {
            out += static_cast<char>(code);
        }
        else if (code < 0x800)
        {
            out += static_cast<char>(0xC0 | (code >> 6));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
        else if (code < 0x10000)
        {
            out += static_cast<char>(0xE0 | (code >> 12));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
        else
        {
            out += static_cast<char>(0xF0 | (code >> 18));
            out += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (code & 0x3F));
        }
        return out;
    }

    uint32_t parseHex4()
    {
        if (pos_ + 4 > text_.size()) fail("short unicode escape");
        uint32_t value = 0;
        for (int i = 0; i < 4; i++)
        {
            const char c = text_[pos_++];
            value <<= 4;
            if (c >= '0' && c <= '9') value |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f') value |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') value |= static_cast<uint32_t>(c - 'A' + 10);
            else fail("bad hex digit");
        }
        return value;
    }

    Json parseNumber()
    {
        const size_t start = pos_;
        bool real = false;
        if (pos_ < text_.size() && (text_[pos_] == '-' || text_[pos_] == '+')) pos_++;
        while (pos_ < text_.size())
        {
            const char c = text_[pos_];
            if (c >= '0' && c <= '9') { pos_++; continue; }
            if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-')
            {
                real = real || c == '.' || c == 'e' || c == 'E';
                pos_++;
                continue;
            }
            break;
        }
        if (pos_ == start) fail("expected a number");
        const std::string token = text_.substr(start, pos_ - start);
        if (real) return Json(std::strtod(token.c_str(), nullptr));
        errno = 0;
        char *end = nullptr;
        const long long value = std::strtoll(token.c_str(), &end, 10);
        if (errno == ERANGE) return Json(std::strtod(token.c_str(), nullptr));
        return Json(value);
    }

    Json parseArray()
    {
        Json out = Json::array();
        pos_++;
        skipSpace();
        if (pos_ < text_.size() && text_[pos_] == ']') { pos_++; return out; }
        while (true)
        {
            out.push(parseValue());
            skipSpace();
            if (pos_ >= text_.size()) fail("unterminated array");
            const char c = text_[pos_++];
            if (c == ']') break;
            if (c != ',') fail("expected a comma");
        }
        return out;
    }

    Json parseObject()
    {
        Json out = Json::object();
        pos_++;
        skipSpace();
        if (pos_ < text_.size() && text_[pos_] == '}') { pos_++; return out; }
        while (true)
        {
            skipSpace();
            const std::string key = parseString();
            skipSpace();
            if (pos_ >= text_.size() || text_[pos_++] != ':') fail("expected a colon");
            out.set(key, parseValue());
            skipSpace();
            if (pos_ >= text_.size()) fail("unterminated object");
            const char c = text_[pos_++];
            if (c == '}') break;
            if (c != ',') fail("expected a comma");
        }
        return out;
    }

    const std::string &text_;
    size_t pos_ = 0;
};

}  // namespace

Json Json::parse(const std::string &text)
{
    Parser parser(text);
    Json value = parser.parseValue();
    parser.finish();
    return value;
}

}  // namespace PlutoSky
