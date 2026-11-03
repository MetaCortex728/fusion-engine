// Minimal JSON reader for tests: parses a complete JSON text into a tree of
// Values and throws std::runtime_error on anything that is not valid JSON
// (RFC 8259), including trailing data. Test-only; not part of the library.
#pragma once

#include <cctype>
#include <cstddef>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace fusion::testing::json {

struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<Value> array;
    std::map<std::string, Value> object;

    bool is_null() const { return type == Type::Null; }
    const Value& operator[](const std::string& key) const {
        if (type != Type::Object) throw std::runtime_error("not an object (key " + key + ")");
        const auto it = object.find(key);
        if (it == object.end()) throw std::runtime_error("missing key " + key);
        return it->second;
    }
    const Value& operator[](std::size_t i) const {
        if (type != Type::Array) throw std::runtime_error("not an array");
        return array.at(i);
    }
    bool has(const std::string& key) const {
        return type == Type::Object && object.count(key) != 0;
    }
    std::size_t size() const {
        if (type == Type::Array) return array.size();
        if (type == Type::Object) return object.size();
        throw std::runtime_error("size of a scalar");
    }
};

class Parser {
public:
    explicit Parser(const std::string& text) : s_(text) {}

    Value parse() {
        Value v = value();
        ws();
        if (i_ != s_.size()) fail("trailing data");
        return v;
    }

private:
    [[noreturn]] void fail(const std::string& what) const {
        throw std::runtime_error("JSON: " + what + " at offset " + std::to_string(i_));
    }
    void ws() {
        while (i_ < s_.size() &&
               (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) {
            ++i_;
        }
    }
    char peek() {
        ws();
        if (i_ >= s_.size()) fail("unexpected end");
        return s_[i_];
    }
    void expect(const char* lit) {
        for (const char* p = lit; *p; ++p, ++i_) {
            if (i_ >= s_.size() || s_[i_] != *p) fail(std::string("expected ") + lit);
        }
    }
    static bool digit(char c) { return c >= '0' && c <= '9'; }

    Value value() {
        Value v;
        const char c = peek();
        if (c == '{') {
            v.type = Value::Type::Object;
            ++i_;
            if (peek() == '}') {
                ++i_;
                return v;
            }
            while (true) {
                if (peek() != '"') fail("expected a key");
                std::string key = string();
                if (peek() != ':') fail("expected ':'");
                ++i_;
                v.object[key] = value();
                const char d = peek();
                ++i_;
                if (d == '}') break;
                if (d != ',') fail("expected ',' or '}'");
            }
        } else if (c == '[') {
            v.type = Value::Type::Array;
            ++i_;
            if (peek() == ']') {
                ++i_;
                return v;
            }
            while (true) {
                v.array.push_back(value());
                const char d = peek();
                ++i_;
                if (d == ']') break;
                if (d != ',') fail("expected ',' or ']'");
            }
        } else if (c == '"') {
            v.type = Value::Type::String;
            v.string = string();
        } else if (c == 't') {
            expect("true");
            v.type = Value::Type::Bool;
            v.boolean = true;
        } else if (c == 'f') {
            expect("false");
            v.type = Value::Type::Bool;
        } else if (c == 'n') {
            expect("null");
        } else {
            v.type = Value::Type::Number;
            v.number = number();
        }
        return v;
    }

    double number() {
        const std::size_t start = i_;
        if (i_ < s_.size() && s_[i_] == '-') ++i_;
        if (i_ >= s_.size() || !digit(s_[i_])) fail("bad number");
        if (s_[i_] == '0') {
            ++i_;
        } else {
            while (i_ < s_.size() && digit(s_[i_])) ++i_;
        }
        if (i_ < s_.size() && s_[i_] == '.') {
            ++i_;
            if (i_ >= s_.size() || !digit(s_[i_])) fail("bad fraction");
            while (i_ < s_.size() && digit(s_[i_])) ++i_;
        }
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            ++i_;
            if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
            if (i_ >= s_.size() || !digit(s_[i_])) fail("bad exponent");
            while (i_ < s_.size() && digit(s_[i_])) ++i_;
        }
        return std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
    }

    std::string string() {
        ++i_;  // opening quote
        std::string r;
        while (true) {
            if (i_ >= s_.size()) fail("unterminated string");
            const char c = s_[i_++];
            if (c == '"') break;
            if (static_cast<unsigned char>(c) < 0x20) fail("control character in string");
            if (c != '\\') {
                r += c;
                continue;
            }
            if (i_ >= s_.size()) fail("unterminated escape");
            const char e = s_[i_++];
            switch (e) {
                case '"': r += '"'; break;
                case '\\': r += '\\'; break;
                case '/': r += '/'; break;
                case 'b': r += '\b'; break;
                case 'f': r += '\f'; break;
                case 'n': r += '\n'; break;
                case 'r': r += '\r'; break;
                case 't': r += '\t'; break;
                case 'u': {
                    if (i_ + 4 > s_.size()) fail("short \\u escape");
                    const std::string hex = s_.substr(i_, 4);
                    for (char h : hex) {
                        if (!std::isxdigit(static_cast<unsigned char>(h))) fail("bad \\u escape");
                    }
                    i_ += 4;
                    const auto cp = static_cast<unsigned>(std::strtoul(hex.c_str(), nullptr, 16));
                    // Basic Multilingual Plane only; enough for these tests.
                    if (cp < 0x80) {
                        r += static_cast<char>(cp);
                    } else if (cp < 0x800) {
                        r += static_cast<char>(0xC0 | (cp >> 6));
                        r += static_cast<char>(0x80 | (cp & 0x3F));
                    } else {
                        r += static_cast<char>(0xE0 | (cp >> 12));
                        r += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                        r += static_cast<char>(0x80 | (cp & 0x3F));
                    }
                    break;
                }
                default: fail("bad escape");
            }
        }
        return r;
    }

    const std::string& s_;
    std::size_t i_ = 0;
};

inline Value parse(const std::string& text) { return Parser(text).parse(); }

}  // namespace fusion::testing::json
