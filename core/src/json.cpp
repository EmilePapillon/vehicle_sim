#include "vd/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace vd::json {

const Value* Value::get(const std::string& key) const {
  if (type_ != Type::Object) return nullptr;
  for (const auto& kv : obj_)
    if (kv.first == key) return &kv.second;
  return nullptr;
}

Value& Value::operator[](const std::string& key) {
  if (type_ == Type::Null) type_ = Type::Object;
  for (auto& kv : obj_)
    if (kv.first == key) return kv.second;
  obj_.emplace_back(key, Value());
  return obj_.back().second;
}

void Value::push(Value v) {
  if (type_ == Type::Null) type_ = Type::Array;
  arr_.push_back(std::move(v));
}

double Value::number(const std::string& key, double def) const {
  const Value* v = get(key);
  return v ? v->asNumber(def) : def;
}

std::string Value::string(const std::string& key, const std::string& def) const {
  const Value* v = get(key);
  return (v && v->isString()) ? v->asString() : def;
}

namespace {

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}

  bool parseDocument(Value& out) {
    skipWs();
    if (!parseValue(out, 0)) return false;
    skipWs();
    if (pos_ != s_.size()) return fail("trailing characters");
    return true;
  }
  std::string error() const { return err_; }

 private:
  bool fail(const char* msg) {
    if (err_.empty()) {
      size_t line = 1, col = 1;
      for (size_t i = 0; i < pos_ && i < s_.size(); ++i) {
        if (s_[i] == '\n') { ++line; col = 1; } else { ++col; }
      }
      err_ = std::to_string(line) + ":" + std::to_string(col) + ": " + msg;
    }
    return false;
  }

  void skipWs() {
    while (pos_ < s_.size()) {
      const char c = s_[pos_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++pos_;
      } else if (c == '/' && pos_ + 1 < s_.size() && s_[pos_ + 1] == '/') {  // allow // comments
        while (pos_ < s_.size() && s_[pos_] != '\n') ++pos_;
      } else {
        break;
      }
    }
  }

  bool literal(const char* lit) {
    size_t n = 0;
    while (lit[n]) ++n;
    if (s_.compare(pos_, n, lit) != 0) return false;
    pos_ += n;
    return true;
  }

  bool parseValue(Value& out, int depth) {
    if (depth > 128) return fail("nesting too deep");
    if (pos_ >= s_.size()) return fail("unexpected end of input");
    const char c = s_[pos_];
    if (c == '{') return parseObject(out, depth);
    if (c == '[') return parseArray(out, depth);
    if (c == '"') {
      std::string str;
      if (!parseString(str)) return false;
      out = Value(std::move(str));
      return true;
    }
    if (literal("true")) { out = Value(true); return true; }
    if (literal("false")) { out = Value(false); return true; }
    if (literal("null")) { out = Value(); return true; }
    return parseNumber(out);
  }

  bool parseNumber(Value& out) {
    const char* begin = s_.c_str() + pos_;
    char* end = nullptr;
    const double d = std::strtod(begin, &end);
    if (end == begin) return fail("invalid value");
    pos_ += size_t(end - begin);
    out = Value(d);
    return true;
  }

  static void appendUtf8(std::string& out, unsigned cp) {
    if (cp < 0x80) {
      out += char(cp);
    } else if (cp < 0x800) {
      out += char(0xC0 | (cp >> 6));
      out += char(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
      out += char(0xE0 | (cp >> 12));
      out += char(0x80 | ((cp >> 6) & 0x3F));
      out += char(0x80 | (cp & 0x3F));
    } else {
      out += char(0xF0 | (cp >> 18));
      out += char(0x80 | ((cp >> 12) & 0x3F));
      out += char(0x80 | ((cp >> 6) & 0x3F));
      out += char(0x80 | (cp & 0x3F));
    }
  }

  bool parseHex4(unsigned& cp) {
    if (pos_ + 4 > s_.size()) return fail("bad unicode escape");
    cp = 0;
    for (int i = 0; i < 4; ++i) {
      const char h = s_[pos_++];
      cp <<= 4;
      if (h >= '0' && h <= '9') cp |= unsigned(h - '0');
      else if (h >= 'a' && h <= 'f') cp |= unsigned(h - 'a' + 10);
      else if (h >= 'A' && h <= 'F') cp |= unsigned(h - 'A' + 10);
      else return fail("bad unicode escape");
    }
    return true;
  }

  bool parseString(std::string& out) {
    ++pos_;  // opening quote
    while (pos_ < s_.size()) {
      const char c = s_[pos_++];
      if (c == '"') return true;
      if (c != '\\') { out += c; continue; }
      if (pos_ >= s_.size()) break;
      const char e = s_[pos_++];
      switch (e) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
          unsigned cp = 0;
          if (!parseHex4(cp)) return false;
          if (cp >= 0xD800 && cp < 0xDC00 && pos_ + 6 <= s_.size() && s_[pos_] == '\\' && s_[pos_ + 1] == 'u') {
            pos_ += 2;
            unsigned lo = 0;
            if (!parseHex4(lo)) return false;
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          }
          appendUtf8(out, cp);
          break;
        }
        default: return fail("bad escape");
      }
    }
    return fail("unterminated string");
  }

  bool parseArray(Value& out, int depth) {
    ++pos_;
    out = Value::makeArray();
    skipWs();
    if (pos_ < s_.size() && s_[pos_] == ']') { ++pos_; return true; }
    for (;;) {
      skipWs();
      Value item;
      if (!parseValue(item, depth + 1)) return false;
      out.push(std::move(item));
      skipWs();
      if (pos_ >= s_.size()) return fail("unterminated array");
      if (s_[pos_] == ',') { ++pos_; continue; }
      if (s_[pos_] == ']') { ++pos_; return true; }
      return fail("expected ',' or ']'");
    }
  }

  bool parseObject(Value& out, int depth) {
    ++pos_;
    out = Value::makeObject();
    skipWs();
    if (pos_ < s_.size() && s_[pos_] == '}') { ++pos_; return true; }
    for (;;) {
      skipWs();
      if (pos_ >= s_.size() || s_[pos_] != '"') return fail("expected key string");
      std::string key;
      if (!parseString(key)) return false;
      skipWs();
      if (pos_ >= s_.size() || s_[pos_] != ':') return fail("expected ':'");
      ++pos_;
      skipWs();
      Value item;
      if (!parseValue(item, depth + 1)) return false;
      out[key] = std::move(item);
      skipWs();
      if (pos_ >= s_.size()) return fail("unterminated object");
      if (s_[pos_] == ',') { ++pos_; continue; }
      if (s_[pos_] == '}') { ++pos_; return true; }
      return fail("expected ',' or '}'");
    }
  }

  const std::string& s_;
  size_t pos_ = 0;
  std::string err_;
};

void escapeString(std::string& out, const std::string& s) {
  out += '"';
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", unsigned(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  out += '"';
}

void dumpImpl(std::string& out, const Value& v, int indent, int level) {
  const auto newline = [&](int lvl) {
    if (indent <= 0) return;
    out += '\n';
    out.append(size_t(indent * lvl), ' ');
  };
  switch (v.type()) {
    case Value::Type::Null: out += "null"; break;
    case Value::Type::Bool: out += v.asBool() ? "true" : "false"; break;
    case Value::Type::Number: {
      const double d = v.asNumber();
      if (!std::isfinite(d)) { out += "null"; break; }
      char buf[32];
      std::snprintf(buf, sizeof buf, "%.10g", d);
      out += buf;
      break;
    }
    case Value::Type::String: escapeString(out, v.asString()); break;
    case Value::Type::Array: {
      const auto& a = v.asArray();
      // Short arrays of numbers stay on one line (torque curves, sweep values).
      bool flat = a.size() <= 16;
      for (const auto& e : a) flat = flat && (e.isNumber() || (e.isArray() && e.asArray().size() <= 4));
      out += '[';
      for (size_t i = 0; i < a.size(); ++i) {
        if (i) out += flat ? ", " : ",";
        if (!flat) newline(level + 1);
        dumpImpl(out, a[i], flat ? 0 : indent, level + 1);
      }
      if (!flat && !a.empty()) newline(level);
      out += ']';
      break;
    }
    case Value::Type::Object: {
      const auto& o = v.asObject();
      out += '{';
      for (size_t i = 0; i < o.size(); ++i) {
        if (i) out += ',';
        newline(level + 1);
        escapeString(out, o[i].first);
        out += indent > 0 ? ": " : ":";
        dumpImpl(out, o[i].second, indent, level + 1);
      }
      if (!o.empty()) newline(level);
      out += '}';
      break;
    }
  }
}

}  // namespace

Value parse(const std::string& text, std::string* error) {
  Parser p(text);
  Value v;
  if (!p.parseDocument(v)) {
    if (error) *error = p.error();
    return Value();
  }
  if (error) error->clear();
  return v;
}

std::string dump(const Value& v, int indent) {
  std::string out;
  dumpImpl(out, v, indent, 0);
  return out;
}

bool readFile(const std::string& path, std::string& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

bool writeFile(const std::string& path, const std::string& content) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << content;
  return bool(f);
}

}  // namespace vd::json
