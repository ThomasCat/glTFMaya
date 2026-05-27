#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace gltfmaya::io {

// A minimal JSON document model sufficient for glTF 2.0: objects preserve
// insertion order so serialized output stays stable and diff-friendly.
class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    using Array = std::vector<Json>;
    using Member = std::pair<std::string, Json>;
    using Object = std::vector<Member>;

    Json() = default;
    Json(bool b) : type_(Type::Bool), bool_(b) {}
    Json(int v) : type_(Type::Number), num_(static_cast<double>(v)) {}
    Json(double v) : type_(Type::Number), num_(v) {}
    Json(const char* s) : type_(Type::String), str_(s) {}
    Json(std::string s) : type_(Type::String), str_(std::move(s)) {}

    static Json makeArray() { Json j; j.type_ = Type::Array; return j; }
    static Json makeObject() { Json j; j.type_ = Type::Object; return j; }

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }

    bool asBool(bool fallback = false) const { return type_ == Type::Bool ? bool_ : fallback; }
    double asNumber(double fallback = 0.0) const { return type_ == Type::Number ? num_ : fallback; }
    int asInt(int fallback = 0) const {
        return type_ == Type::Number ? static_cast<int>(num_) : fallback;
    }
    const std::string& asString() const { return str_; }

    const Array& items() const { return array_; }
    Array& items() { return array_; }
    void push_back(Json v) { array_.push_back(std::move(v)); }

    // Object access. has()/find() return nullptr when absent.
    bool has(const std::string& key) const { return find(key) != nullptr; }
    const Json* find(const std::string& key) const {
        for (const auto& m : object_) if (m.first == key) return &m.second;
        return nullptr;
    }
    void set(std::string key, Json value) {
        for (auto& m : object_) {
            if (m.first == key) { m.second = std::move(value); return; }
        }
        object_.emplace_back(std::move(key), std::move(value));
    }
    const Object& members() const { return object_; }

    // Convenience: typed object-member lookups with defaults.
    int intAt(const std::string& key, int fallback = 0) const {
        const Json* j = find(key);
        return j ? j->asInt(fallback) : fallback;
    }
    double numberAt(const std::string& key, double fallback = 0.0) const {
        const Json* j = find(key);
        return j ? j->asNumber(fallback) : fallback;
    }
    bool boolAt(const std::string& key, bool fallback = false) const {
        const Json* j = find(key);
        return j ? j->asBool(fallback) : fallback;
    }
    std::string stringAt(const std::string& key, const std::string& fallback = {}) const {
        const Json* j = find(key);
        return j && j->isString() ? j->asString() : fallback;
    }

    std::string dump(bool pretty = false) const;
    static Json parse(const std::string& text, std::string& error);

private:
    Type type_ = Type::Null;
    bool bool_ = false;
    double num_ = 0.0;
    std::string str_;
    Array array_;
    Object object_;
};

} // namespace gltfmaya::io
