// ============================================================================
// 仿真进程推送流客户端 (C++)：GET /api/v1/stream?hz=50&io_hz=20&scans=1
//   帧 = <u32 负载长度><u8 类型><负载> (sim_server/service.py)；类型 1 状态 2 元信息 3 IO+光电 4 2D 激光/融合扫描
//   断线自动重连；另提供一次性 HTTP GET (取 /api/v1/sim 的 UDP 指令端口等)
// ============================================================================
#pragma once
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace simstream {

struct Url { std::string host = "127.0.0.1"; int port = 8090; std::string base; };

inline Url parse_url(const std::string &u) {
  Url r;
  std::string s = u;
  auto p = s.find("://");
  if (p != std::string::npos) s = s.substr(p + 3);
  auto slash = s.find('/');
  std::string hp = slash == std::string::npos ? s : s.substr(0, slash);
  r.base = slash == std::string::npos ? "" : s.substr(slash);
  while (!r.base.empty() && r.base.back() == '/') r.base.pop_back();
  auto c = hp.rfind(':');
  if (c != std::string::npos) { r.host = hp.substr(0, c); r.port = std::atoi(hp.c_str() + c + 1); }
  else r.host = hp;
  return r;
}

inline int connect_tcp(const Url &u, int timeout_ms) {
  addrinfo hints{}, *res = nullptr;
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if (getaddrinfo(u.host.c_str(), std::to_string(u.port).c_str(), &hints, &res) != 0 || !res) return -1;
  int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if (fd < 0) { freeaddrinfo(res); return -1; }
  timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) { close(fd); freeaddrinfo(res); return -1; }
  freeaddrinfo(res);
  return fd;
}

inline bool send_all(int fd, const std::string &s) {
  size_t off = 0;
  while (off < s.size()) {
    ssize_t n = send(fd, s.data() + off, s.size() - off, MSG_NOSIGNAL);
    if (n <= 0) return false;
    off += static_cast<size_t>(n);
  }
  return true;
}

// 读到 n 字节或出错；stop 置位时返回 false
inline bool read_n(int fd, uint8_t *p, size_t n, const std::atomic<bool> &stop) {
  size_t got = 0;
  while (got < n) {
    if (stop) return false;
    ssize_t r = recv(fd, p + got, n - got, 0);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      return false;
    }
    got += static_cast<size_t>(r);
  }
  return true;
}

// 读 HTTP 响应头，返回状态码 (失败 -1)；body 起始的多余字节放进 rest
inline int read_headers(int fd, std::string &rest, const std::atomic<bool> &stop) {
  std::string h;
  char buf[1024];
  while (h.find("\r\n\r\n") == std::string::npos) {
    if (stop) return -1;
    ssize_t r = recv(fd, buf, sizeof(buf), 0);
    if (r <= 0) {
      if (r < 0 && (errno == EAGAIN || errno == EINTR)) continue;
      return -1;
    }
    h.append(buf, static_cast<size_t>(r));
    if (h.size() > 65536) return -1;
  }
  auto e = h.find("\r\n\r\n");
  rest = h.substr(e + 4);
  auto sp = h.find(' ');
  return sp == std::string::npos ? -1 : std::atoi(h.c_str() + sp + 1);
}

// 一次性 GET，返回 body (失败返回空串，status 置 -1)
inline std::string http_get(const Url &u, const std::string &path, int *status = nullptr, int timeout_ms = 3000) {
  std::atomic<bool> stop{false};
  if (status) *status = -1;
  int fd = connect_tcp(u, timeout_ms);
  if (fd < 0) return {};
  std::string req = "GET " + u.base + path + " HTTP/1.1\r\nHost: " + u.host + "\r\nAccept: application/json\r\nConnection: close\r\n\r\n";
  std::string body;
  if (send_all(fd, req)) {
    int st = read_headers(fd, body, stop);
    if (status) *status = st;
    char buf[4096];
    while (st > 0) {
      ssize_t r = recv(fd, buf, sizeof(buf), 0);
      if (r <= 0) break;
      body.append(buf, static_cast<size_t>(r));
    }
  }
  close(fd);
  return body;
}

class Stream {
 public:
  using FrameCb = std::function<void(uint8_t type, const uint8_t *body, size_t len)>;
  using StateCb = std::function<void(bool connected)>;

  Stream(const std::string &url, FrameCb cb, StateCb st) : url_(parse_url(url)), cb_(std::move(cb)), st_(std::move(st)) {}
  ~Stream() { stop(); }

  void start() { th_ = std::thread([this] { loop(); }); }
  void stop() {
    stop_ = true;
    if (th_.joinable()) th_.join();
  }
  const Url &url() const { return url_; }

 private:
  void loop() {
    while (!stop_) {
      int fd = connect_tcp(url_, 1000);
      if (fd < 0) { sleep_ms(500); continue; }
      std::string req = "GET " + url_.base + "/api/v1/stream?hz=50&io_hz=20&scans=1 HTTP/1.1\r\nHost: " + url_.host +
                        "\r\nConnection: close\r\n\r\n";
      std::string rest;
      if (!send_all(fd, req) || read_headers(fd, rest, stop_) != 200) { close(fd); sleep_ms(1000); continue; }
      st_(true);
      std::vector<uint8_t> buf(rest.begin(), rest.end());
      size_t off = 0;
      std::vector<uint8_t> body;
      while (!stop_) {
        uint8_t hdr[5];
        if (!take(fd, buf, off, hdr, 5)) break;
        uint32_t n;
        std::memcpy(&n, hdr, 4);
        if (n > (64u << 20)) break;
        body.resize(n);
        if (n && !take(fd, buf, off, body.data(), n)) break;
        cb_(hdr[4], body.data(), n);
      }
      close(fd);
      st_(false);
      sleep_ms(300);
    }
  }
  // 先用响应头之后已读入的字节，再从套接字读
  bool take(int fd, std::vector<uint8_t> &buf, size_t &off, uint8_t *dst, size_t n) {
    size_t have = buf.size() - off;
    size_t k = std::min(have, n);
    if (k) { std::memcpy(dst, buf.data() + off, k); off += k; }
    if (off >= buf.size() && !buf.empty()) { buf.clear(); off = 0; }
    return k == n || read_n(fd, dst + k, n - k, stop_);
  }
  static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  Url url_;
  FrameCb cb_;
  StateCb st_;
  std::atomic<bool> stop_{false};
  std::thread th_;
};

}  // namespace simstream
