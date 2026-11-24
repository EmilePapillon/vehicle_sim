// Minimal dependency-free JSON value, parser and writer (objects keep insertion order).
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace vd::json {

class Value {
 public:
  enum class Type { Null, Bool, Number, String, Array, Object };
  using Object = std::vector<std::pair<std::string, Value>>;
  using Array = std::vector<Value>;

  Value() = default;
  Value(std::nullptr_t) {}
  Value(bool b) : type_(Type::Bool), bool_(b) {}
  Value(double d) : type_(Type::Number), num_(d) {}
  Value(int i) : type_(Type::Number), num_(double(i)) {}
  Value(std::string s) : type_(Type::String), str_(std::move(s)) {}
  Value(const char* s) : type_(Type::String), str_(s) {}
  static Value makeArray() { Value v; v.type_ = Type::Array; return v; }
  static Value makeObject() { Value v; v.type_ = Type::Object; return v; }

  Type type() const { return type_; }
  bool isNull() const { return type_ == Type::Null; }
  bool isBool() const { return type_ == Type::Bool; }
  bool isNumber() const { return type_ == Type::Number; }
  bool isString() const { return type_ == Type::String; }
  bool isArray() const { return type_ == Type::Array; }
  bool isObject() const { return type_ == Type::Object; }

  double asNumber(double def = 0.0) const { return isNumber() ? num_ : (isBool() ? double(bool_) : def); }
  bool asBool(bool def = false) const { return isBool() ? bool_ : (isNumber() ? num_ != 0.0 : def); }
  const std::string& asString() const { return str_; }
  const Array& asArray() const { return arr_; }
  const Object& asObject() const { return obj_; }
  Array& asArray() { return arr_; }
  Object& asObject() { return obj_; }

  // Object access. get() returns nullptr when missing or not an object.
  const Value* get(const std::string& key) const;
  Value& operator[](const std::string& key);  // inserts (converting Null to Object)
  void push(Value v);                         // appends (converting Null to Array)

  double number(const std::string& key, double def) const;
  std::string string(const std::string& key, const std::string& def) const;

 private:
  Type type_ = Type::Null;
  bool bool_ = false;
  double num_ = 0.0;
  std::string str_;
  Array arr_;
  Object obj_;
};

// Parses text; on failure returns Null and sets *error (with line:column).
Value parse(const std::string& text, std::string* error = nullptr);
std::string dump(const Value& v, int indent = 2);

bool readFile(const std::string& path, std::string& out);
bool writeFile(const std::string& path, const std::string& content);

}  // namespace vd::json
