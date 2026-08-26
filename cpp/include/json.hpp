#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <cstdlib>

namespace route_sim {

enum class JsonType { Null, Bool, Number, String, Array, Object };

struct JsonValue {
    JsonType type = JsonType::Null;
    bool bool_val = false;
    double num_val = 0.0;
    std::string str_val;
    std::vector<JsonValue> arr_val;
    std::unordered_map<std::string, JsonValue> obj_val;

    bool is_number() const { return type == JsonType::Number; }
    bool is_string() const { return type == JsonType::String; }
    bool is_object() const { return type == JsonType::Object; }
    bool is_array() const { return type == JsonType::Array; }

    double as_double() const { return num_val; }
    int as_int() const { return static_cast<int>(num_val); }
    const std::string& as_string() const { return str_val; }
    const std::vector<JsonValue>& as_array() const { return arr_val; }
    const std::unordered_map<std::string, JsonValue>& as_object() const { return obj_val; }

    const JsonValue& operator[](const std::string& key) const {
        auto it = obj_val.find(key);
        if (it != obj_val.end()) return it->second;
        static const JsonValue null_val;
        return null_val;
    }

    const JsonValue& operator[](size_t idx) const {
        if (idx < arr_val.size()) return arr_val[idx];
        static const JsonValue null_val;
        return null_val;
    }

    bool contains(const std::string& key) const {
        return obj_val.find(key) != obj_val.end();
    }
};

class JsonParser {
public:
    static JsonValue parse(const std::string& src) {
        size_t idx = 0;
        skip_whitespace(src, idx);
        return parse_value(src, idx);
    }

private:
    static void skip_whitespace(const std::string& src, size_t& idx) {
        while (idx < src.size() && (src[idx] == ' ' || src[idx] == '\t' || src[idx] == '\n' || src[idx] == '\r')) {
            idx++;
        }
    }

    static JsonValue parse_value(const std::string& src, size_t& idx) {
        skip_whitespace(src, idx);
        if (idx >= src.size()) return JsonValue{};

        char c = src[idx];
        if (c == '{') return parse_object(src, idx);
        if (c == '[') return parse_array(src, idx);
        if (c == '"') return parse_string(src, idx);
        if (c == 't' || c == 'f') return parse_bool(src, idx);
        if (c == 'n') return parse_null(src, idx);
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parse_number(src, idx);

        throw std::runtime_error(std::string("Unexpected character in JSON: ") + c);
    }

    static JsonValue parse_object(const std::string& src, size_t& idx) {
        JsonValue val;
        val.type = JsonType::Object;
        idx++; // skip '{'
        skip_whitespace(src, idx);

        if (idx < src.size() && src[idx] == '}') {
            idx++;
            return val;
        }

        while (idx < src.size()) {
            skip_whitespace(src, idx);
            if (src[idx] != '"') throw std::runtime_error("Expected string key in object");
            JsonValue key_val = parse_string(src, idx);
            skip_whitespace(src, idx);
            if (idx >= src.size() || src[idx] != ':') throw std::runtime_error("Expected ':' after key");
            idx++; // skip ':'
            JsonValue item = parse_value(src, idx);
            val.obj_val.emplace(key_val.str_val, std::move(item));

            skip_whitespace(src, idx);
            if (idx < src.size() && src[idx] == ',') {
                idx++;
                continue;
            }
            if (idx < src.size() && src[idx] == '}') {
                idx++;
                break;
            }
            throw std::runtime_error("Expected ',' or '}' in object");
        }
        return val;
    }

    static JsonValue parse_array(const std::string& src, size_t& idx) {
        JsonValue val;
        val.type = JsonType::Array;
        idx++; // skip '['
        skip_whitespace(src, idx);

        if (idx < src.size() && src[idx] == ']') {
            idx++;
            return val;
        }

        while (idx < src.size()) {
            JsonValue item = parse_value(src, idx);
            val.arr_val.push_back(std::move(item));
            skip_whitespace(src, idx);
            if (idx < src.size() && src[idx] == ',') {
                idx++;
                continue;
            }
            if (idx < src.size() && src[idx] == ']') {
                idx++;
                break;
            }
            throw std::runtime_error("Expected ',' or ']' in array");
        }
        return val;
    }

    static JsonValue parse_string(const std::string& src, size_t& idx) {
        JsonValue val;
        val.type = JsonType::String;
        idx++; // skip opening quote
        std::string s;
        while (idx < src.size()) {
            char c = src[idx++];
            if (c == '"') {
                val.str_val = s;
                return val;
            }
            if (c == '\\' && idx < src.size()) {
                char esc = src[idx++];
                if (esc == '"') s.push_back('"');
                else if (esc == '\\') s.push_back('\\');
                else if (esc == '/') s.push_back('/');
                else if (esc == 'n') s.push_back('\n');
                else if (esc == 't') s.push_back('\t');
                else if (esc == 'r') s.push_back('\r');
                else s.push_back(esc);
            } else {
                s.push_back(c);
            }
        }
        throw std::runtime_error("Unterminated string in JSON");
    }

    static JsonValue parse_number(const std::string& src, size_t& idx) {
        const char* start_ptr = src.c_str() + idx;
        char* end_ptr = nullptr;
        double num = std::strtod(start_ptr, &end_ptr);
        idx += static_cast<size_t>(end_ptr - start_ptr);

        JsonValue val;
        val.type = JsonType::Number;
        val.num_val = num;
        return val;
    }

    static JsonValue parse_bool(const std::string& src, size_t& idx) {
        JsonValue val;
        val.type = JsonType::Bool;
        if (src.compare(idx, 4, "true") == 0) {
            val.bool_val = true;
            idx += 4;
        } else if (src.compare(idx, 5, "false") == 0) {
            val.bool_val = false;
            idx += 5;
        } else {
            throw std::runtime_error("Invalid boolean in JSON");
        }
        return val;
    }

    static JsonValue parse_null(const std::string& src, size_t& idx) {
        if (src.compare(idx, 4, "null") == 0) {
            idx += 4;
            return JsonValue{};
        }
        throw std::runtime_error("Invalid null in JSON");
    }
};

} // namespace route_sim
