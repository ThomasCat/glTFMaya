#include "io/Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace gltfmaya::io {
namespace {

void escapeTo(std::string& out, const std::string& s) {
    out.push_back('"');
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(static_cast<char>(c));
                }
        }
    }
    out.push_back('"');
}

void numberTo(std::string& out, double v) {
    if (std::isfinite(v) && v == std::floor(v) && std::abs(v) < 1e15) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(v));
        out += buf;
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    out += buf;
}

void dumpValue(std::string& out, const Json& j, bool pretty, int depth);

void indent(std::string& out, int depth) {
    for (int i = 0; i < depth; ++i) out += "  ";
}

void dumpValue(std::string& out, const Json& j, bool pretty, int depth) {
    switch (j.type()) {
        case Json::Type::Null: out += "null"; break;
        case Json::Type::Bool: out += j.asBool() ? "true" : "false"; break;
        case Json::Type::Number: numberTo(out, j.asNumber()); break;
        case Json::Type::String: escapeTo(out, j.asString()); break;
        case Json::Type::Array: {
            const auto& items = j.items();
            if (items.empty()) { out += "[]"; break; }
            out.push_back('[');
            for (size_t i = 0; i < items.size(); ++i) {
                if (pretty) { out.push_back('\n'); indent(out, depth + 1); }
                dumpValue(out, items[i], pretty, depth + 1);
                if (i + 1 < items.size()) out.push_back(',');
            }
            if (pretty) { out.push_back('\n'); indent(out, depth); }
            out.push_back(']');
            break;
        }
        case Json::Type::Object: {
            const auto& members = j.members();
            if (members.empty()) { out += "{}"; break; }
            out.push_back('{');
            for (size_t i = 0; i < members.size(); ++i) {
                if (pretty) { out.push_back('\n'); indent(out, depth + 1); }
                escapeTo(out, members[i].first);
                out.push_back(':');
                if (pretty) out.push_back(' ');
                dumpValue(out, members[i].second, pretty, depth + 1);
                if (i + 1 < members.size()) out.push_back(',');
            }
            if (pretty) { out.push_back('\n'); indent(out, depth); }
            out.push_back('}');
            break;
        }
    }
}

class Parser {
public:
    Parser(const std::string& text) : s_(text) {}

    bool parse(Json& out) {
        skipWs();
        if (!parseValue(out)) return false;
        skipWs();
        if (pos_ != s_.size()) return fail("trailing characters after JSON value");
        return true;
    }

    const std::string& error() const { return error_; }

private:
    const std::string& s_;
    size_t pos_ = 0;
    std::string error_;

    bool fail(const std::string& msg) {
        if (error_.empty()) {
            std::ostringstream ss;
            ss << msg << " at offset " << pos_;
            error_ = ss.str();
        }
        return false;
    }

    void skipWs() {
        while (pos_ < s_.size()) {
            char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++pos_;
            else break;
        }
    }

    bool parseValue(Json& out) {
        skipWs();
        if (pos_ >= s_.size()) return fail("unexpected end of input");
        char c = s_[pos_];
        switch (c) {
            case '{': return parseObject(out);
            case '[': return parseArray(out);
            case '"': {
                std::string str;
                if (!parseString(str)) return false;
                out = Json(std::move(str));
                return true;
            }
            case 't': case 'f': return parseBool(out);
            case 'n': return parseNull(out);
            default: return parseNumber(out);
        }
    }

    bool parseObject(Json& out) {
        out = Json::makeObject();
        ++pos_; // {
        skipWs();
        if (pos_ < s_.size() && s_[pos_] == '}') { ++pos_; return true; }
        while (true) {
            skipWs();
            if (pos_ >= s_.size() || s_[pos_] != '"') return fail("expected object key string");
            std::string key;
            if (!parseString(key)) return false;
            skipWs();
            if (pos_ >= s_.size() || s_[pos_] != ':') return fail("expected ':' after object key");
            ++pos_;
            Json value;
            if (!parseValue(value)) return false;
            out.set(std::move(key), std::move(value));
            skipWs();
            if (pos_ >= s_.size()) return fail("unterminated object");
            if (s_[pos_] == ',') { ++pos_; continue; }
            if (s_[pos_] == '}') { ++pos_; return true; }
            return fail("expected ',' or '}' in object");
        }
    }

    bool parseArray(Json& out) {
        out = Json::makeArray();
        ++pos_; // [
        skipWs();
        if (pos_ < s_.size() && s_[pos_] == ']') { ++pos_; return true; }
        while (true) {
            Json value;
            if (!parseValue(value)) return false;
            out.push_back(std::move(value));
            skipWs();
            if (pos_ >= s_.size()) return fail("unterminated array");
            if (s_[pos_] == ',') { ++pos_; continue; }
            if (s_[pos_] == ']') { ++pos_; return true; }
            return fail("expected ',' or ']' in array");
        }
    }

    bool parseString(std::string& out) {
        ++pos_; // opening quote
        while (pos_ < s_.size()) {
            char c = s_[pos_++];
            if (c == '"') return true;
            if (c == '\\') {
                if (pos_ >= s_.size()) return fail("unterminated escape");
                char e = s_[pos_++];
                switch (e) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    case 'u': {
                        if (pos_ + 4 > s_.size()) return fail("truncated \\u escape");
                        unsigned code = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = s_[pos_++];
                            code <<= 4;
                            if (h >= '0' && h <= '9') code |= unsigned(h - '0');
                            else if (h >= 'a' && h <= 'f') code |= unsigned(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') code |= unsigned(h - 'A' + 10);
                            else return fail("invalid hex in \\u escape");
                        }
                        appendUtf8(out, code);
                        break;
                    }
                    default: return fail("invalid escape character");
                }
            } else {
                out.push_back(c);
            }
        }
        return fail("unterminated string");
    }

    static void appendUtf8(std::string& out, unsigned code) {
        if (code < 0x80) {
            out.push_back(static_cast<char>(code));
        } else if (code < 0x800) {
            out.push_back(static_cast<char>(0xC0 | (code >> 6)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        } else {
            out.push_back(static_cast<char>(0xE0 | (code >> 12)));
            out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
            out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        }
    }

    bool parseBool(Json& out) {
        if (s_.compare(pos_, 4, "true") == 0) { pos_ += 4; out = Json(true); return true; }
        if (s_.compare(pos_, 5, "false") == 0) { pos_ += 5; out = Json(false); return true; }
        return fail("invalid literal");
    }

    bool parseNull(Json& out) {
        if (s_.compare(pos_, 4, "null") == 0) { pos_ += 4; out = Json(); return true; }
        return fail("invalid literal");
    }

    bool parseNumber(Json& out) {
        size_t start = pos_;
        if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) ++pos_;
        bool any = false;
        while (pos_ < s_.size()) {
            char c = s_[pos_];
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' ||
                c == '+' || c == '-') {
                ++pos_;
                any = true;
            } else {
                break;
            }
        }
        if (!any) return fail("invalid number");
        const char* begin = s_.c_str() + start;
        char* end = nullptr;
        double v = std::strtod(begin, &end);
        if (end == begin) return fail("invalid number");
        out = Json(v);
        return true;
    }
};

} // namespace

std::string Json::dump(bool pretty) const {
    std::string out;
    dumpValue(out, *this, pretty, 0);
    return out;
}

Json Json::parse(const std::string& text, std::string& error) {
    Parser parser(text);
    Json out;
    if (!parser.parse(out)) {
        error = parser.error();
        return Json();
    }
    error.clear();
    return out;
}

} // namespace gltfmaya::io
