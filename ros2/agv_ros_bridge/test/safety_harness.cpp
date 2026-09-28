// 安全层一致性测试用: 从 stdin 读 JSON 用例 (每行一个)，输出 C++ 结果 (每行一个 JSON)。由 tests/test_cpp_safety.py 驱动
#include <iomanip>
#include <iostream>
#include <string>

#include "../src/json_lite.hpp"
#include "../src/safety.hpp"

int main() {
  std::string line;
  std::cout << std::setprecision(17);
  while (std::getline(std::cin, line)) {
    jl::Value v;
    if (!jl::parse(line, v)) { std::cout << "{\"err\":1}\n"; continue; }
    agvsafe::Config c;
    const auto &P = v["prot"];
    c.enabled = P["enabled"].truthy(true);
    for (const auto &f : P["fields"].a) c.fields.push_back({f["name"].str(), f["v_max"].num(), f["front"].num(), f["rear"].num(), f["side"].num()});
    c.slow_ratio = P["slow_ratio"].num(2.0);
    c.rotate_margin = P["rotate_margin"].num(0.02);
    c.rotate_lookahead = P["rotate_lookahead_rad"].num(0.25);
    c.docking_front = P["docking"]["front"].num(0.02);
    c.photo_mode = P["photo"]["mode"].str();
    c.photo_front = P["photo"]["front"].num(); c.photo_rear = P["photo"]["rear"].num(); c.photo_side = P["photo"]["side"].num();
    c.mute_near_stop = P["photo"]["mute_near_stop"].num(0.1);
    c.h = v["outline"][0].num(); c.t = v["outline"][1].num(); c.l = v["outline"][2].num(); c.r = v["outline"][3].num();
    c.max_decel = v["max_decel"].num(0.5);
    for (const auto &p : v["photos"].a) c.photos.push_back({p["name"].str(), p["di"].str(), p["x"].num(), p["y"].num(), p["yaw"].num()});
    std::vector<agvsafe::Pt> pts;
    for (const auto &p : v["pts"].a) pts.push_back({p[0].num(), p[1].num()});
    agvsafe::Env e;
    e.estop = v["estop"].truthy();
    e.speed_cap = v["speed_cap"].num(0.0);
    e.has_left = !v["left"].is_null();
    e.approach_left = v["left"].num(0.0);
    e.in_arc = v["in_arc"].truthy();
    e.v_meas = v["v_meas"].num();
    e.bands = agvsafe::compute_bands(c, pts);
    e.pts = &pts;
    for (const auto &h : v["hits"].a)
      for (const auto &p : c.photos) if (p.name == h[0].str()) e.photo_hits.emplace_back(&p, h[1].is_null() ? -1.0 : h[1].num());
    auto r = agvsafe::filter(c, e, v["cmd"][0].num(), v["cmd"][1].num(), v["cmd"][2].num());
    std::cout << "{\"vx\":" << r.vx << ",\"vy\":" << r.vy << ",\"wz\":" << r.wz << ",\"zone\":\"" << r.zone << "\",\"layer\":\"" << r.layer
              << "\",\"bands\":[";
    for (size_t i = 0; i < e.bands.size(); ++i) std::cout << (i ? "," : "") << "[" << e.bands[i].first << "," << e.bands[i].second << "]";
    std::cout << "]}\n";
  }
}
