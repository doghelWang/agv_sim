// ============================================================================
// 极简 JSON 解析 (只读 DOM): 仿真推送流的元信息/IO 帧、执行进程下发的配置/模式消息
//   不依赖第三方库 (板卡离线部署，镜像里不额外装包)；数字一律 double，字符串支持常见转义与 \uXXXX (转 UTF-8)
// ============================================================================
#pragma once
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace jl {

struct Value {
  enum Type { NUL, BOOL, NUM, STR, ARR, OBJ } type = NUL;
  bool b = false;
  double n = 0.0;
  std::string s;
  std::vector<Value> a;
  std::map<std::string, Value> o;

  bool is_null() const { return type == NUL; }
  const Value &operator[](const std::string &k) const {
    static const Value nul;
    if (type != OBJ) return nul;
    auto it = o.find(k);
    return it == o.end() ? nul : it->second;
  }
  const Value &operator[](size_t i) const {
    static const Value nul;
    return (type == ARR && i < a.size()) ? a[i] : nul;
  }
  size_t size() const { return type == ARR ? a.size() : (type == OBJ ? o.size() : 0); }
  double num(double d = 0.0) const { return type == NUM ? n : (type == BOOL ? (b ? 1.0 : 0.0) : d); }
  bool truthy(bool d = false) const {
    if (type == BOOL) return b;
    if (type == NUM) return n != 0.0;
    if (type == STR) return !s.empty();
    return type == NUL ? d : true;
  }
  const std::string &str() const { static const std::string e; return type == STR ? s : e; }
};

class Parser {
 public:
  explicit Parser(const std::string &t) : t_(t) {}
  bool parse(Value &v) {
    i_ = 0;
    if (!value(v)) return false;
    ws();
    return i_ == t_.size();
  }

 private:
  void ws() { while (i_ < t_.size() && (t_[i_] == ' ' || t_[i_] == '\n' || t_[i_] == '\r' || t_[i_] == '\t')) ++i_; }
  bool lit(const char *w) {
    size_t n = 0;
    while (w[n]) ++n;
    if (t_.compare(i_, n, w) != 0) return false;
    i_ += n;
    return true;
  }
  static void utf8(std::string &o, unsigned cp) {
    if (cp < 0x80) { o += static_cast<char>(cp); }
    else if (cp < 0x800) { o += static_cast<char>(0xC0 | (cp >> 6)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
    else { o += static_cast<char>(0xE0 | (cp >> 12)); o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
  }
  bool string(std::string &o) {
    if (t_[i_] != '"') return false;
    ++i_;
    while (i_ < t_.size()) {
      char c = t_[i_++];
      if (c == '"') return true;
      if (c != '\\') { o += c; continue; }
      if (i_ >= t_.size()) return false;
      char e = t_[i_++];
      switch (e) {
        case 'n': o += '\n'; break;
        case 't': o += '\t'; break;
        case 'r': o += '\r'; break;
        case 'b': o += '\b'; break;
        case 'f': o += '\f'; break;
        case 'u': {
          if (i_ + 4 > t_.size()) return false;
          unsigned cp = static_cast<unsigned>(std::strtoul(t_.substr(i_, 4).c_str(), nullptr, 16));
          i_ += 4;
          utf8(o, cp);
          break;
        }
        default: o += e;
      }
    }
    return false;
  }
  bool value(Value &v) {
    ws();
    if (i_ >= t_.size()) return false;
    char c = t_[i_];
    if (c == '{') {
      v.type = Value::OBJ;
      ++i_;
      ws();
      if (i_ < t_.size() && t_[i_] == '}') { ++i_; return true; }
      while (true) {
        ws();
        std::string k;
        if (!string(k)) return false;
        ws();
        if (i_ >= t_.size() || t_[i_] != ':') return false;
        ++i_;
        if (!value(v.o[k])) return false;
        ws();
        if (i_ < t_.size() && t_[i_] == ',') { ++i_; continue; }
        if (i_ < t_.size() && t_[i_] == '}') { ++i_; return true; }
        return false;
      }
    }
    if (c == '[') {
      v.type = Value::ARR;
      ++i_;
      ws();
      if (i_ < t_.size() && t_[i_] == ']') { ++i_; return true; }
      while (true) {
        v.a.emplace_back();
        if (!value(v.a.back())) return false;
        ws();
        if (i_ < t_.size() && t_[i_] == ',') { ++i_; continue; }
        if (i_ < t_.size() && t_[i_] == ']') { ++i_; return true; }
        return false;
      }
    }
    if (c == '"') { v.type = Value::STR; return string(v.s); }
    if (lit("true")) { v.type = Value::BOOL; v.b = true; return true; }
    if (lit("false")) { v.type = Value::BOOL; v.b = false; return true; }
    if (lit("null")) { v.type = Value::NUL; return true; }
    if (lit("NaN")) { v.type = Value::NUM; v.n = 0.0 / 0.0; return true; }
    if (lit("Infinity")) { v.type = Value::NUM; v.n = 1e308 * 10; return true; }
    char *end = nullptr;
    v.n = std::strtod(t_.c_str() + i_, &end);
    if (end == t_.c_str() + i_) return false;
    v.type = Value::NUM;
    i_ = static_cast<size_t>(end - t_.c_str());
    return true;
  }
  const std::string &t_;
  size_t i_ = 0;
};

inline bool parse(const std::string &text, Value &out) { return Parser(text).parse(out); }

}  // namespace jl
