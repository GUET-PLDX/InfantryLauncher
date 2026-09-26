#include "heat_rate_ctrl.inc"

#include <cassert>
#include <cmath>

int main() {
  using namespace launcher;
  HeatCtrl::Config c; /* single_heat=10, max_frequency=15 */
  HeatCtrl::Observation o;
  HeatCtrl ctrl;

  // 全速档：剩余热量 120 > 100 -> max_frequency。
  o = {240.0f, 120.0f, 60.0f, true};
  auto r = ctrl.Update(c, o);
  assert(r.allow_fire && std::fabs(r.target_frequency - 15.0f) < 1e-5f);

  // 停射档：剩余热量 15 <= 20 -> 不允许发射。
  ctrl.Reset();
  o.current_heat = 225.0f;
  r = ctrl.Update(c, o);
  assert(!r.allow_fire && r.target_frequency == 0.0f);

  // 突发-持续档：m=80, a=60, d=10。
  // 开窗：T=(80+120)*10=2000 ms，突发射频=(800-60-70)/200+6=9.35；
  // 开窗周期输出可持续 6，随后突发段 9.35 计满 2000 个计数（计数在同周期
  // 末尾复位），下一周期重新开窗再输出一拍可持续 6，周而复始。
  ctrl.Reset();
  o = {240.0f, 160.0f, 60.0f, true};
  r = ctrl.Update(c, o); /* 开窗周期 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  for (int i = 0; i < 1999; ++i) {
    r = ctrl.Update(c, o);
    assert(r.allow_fire && std::fabs(r.target_frequency - 9.35f) < 1e-5f);
  }
  r = ctrl.Update(c, o); /* 计满复位后的重新开窗周期：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  r = ctrl.Update(c, o); /* 新突发段 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 9.35f) < 1e-5f);

  // 低热量锁存：m=22 (<=25)。开窗周期内计数即被钉到窗长，此后每周期
  // 均输出可持续射频 6，突发射频不再出现。
  ctrl.Reset();
  o.current_heat = 218.0f; /* m = 22 */
  r = ctrl.Update(c, o);   /* 开窗周期：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  r = ctrl.Update(c, o); /* 锁存周期：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  r = ctrl.Update(c, o);
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  r = ctrl.Update(c, o);
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);

  // 中段维持：25 < m=30 < 40。突发段计满后维持可持续射频，不重开窗。
  ctrl.Reset();
  o.current_heat = 210.0f; /* m = 30 */
  r = ctrl.Update(c, o);   /* 开窗周期：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  const float MID_BURST = (10.0f * 30.0f - 60.0f - 30.0f) /
                              (10.0f * (((30.0f + 120.0f) * 10.0f) / 100.0f)) +
                          6.0f;
  for (int i = 0; i < 1499; ++i) {
    r = ctrl.Update(c, o);
    assert(r.allow_fire && std::fabs(r.target_frequency - MID_BURST) < 1e-4f);
  }
  r = ctrl.Update(c, o); /* 计满周期：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);
  r = ctrl.Update(c, o); /* 中段不重开窗：维持可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);

  // 裁判数据失效：不允许发射且调度状态清零（恢复后重新开窗）。
  ctrl.Reset();
  o = {240.0f, 160.0f, 60.0f, false};
  r = ctrl.Update(c, o);
  assert(!r.allow_fire && r.target_frequency == 0.0f);
  o.data_valid = true;
  r = ctrl.Update(c, o); /* 失效后首周期重新开窗：可持续 6 */
  assert(r.allow_fire && std::fabs(r.target_frequency - 6.0f) < 1e-5f);

  // 非法单发热量：不允许发射。
  c.single_heat = 0.0f;
  r = ctrl.Update(c, o);
  assert(!r.allow_fire && r.target_frequency == 0.0f);
  c.single_heat = 10.0f;

  // At a 100 ms boundary a 10-heat shot settles before 20/s cooling: 50+10-2.
  assert(std::fabs(HeatCtrl::AdvanceBudget(c, 50.0f, 20.0f, 1U, 1U) - 58.0f) <
         1e-5f);
  // No completed period means no cooling may be released early.
  assert(std::fabs(HeatCtrl::AdvanceBudget(c, 50.0f, 20.0f, 1U, 0U) - 60.0f) <
         1e-5f);

  // Sequence: shot at 50 ms -> 60; at 100 ms another shot then cool -> 68;
  // through 250 ms only the 200 ms boundary releases another 2 heat -> 66.
  float budget = HeatCtrl::AdvanceBudget(c, 50.0f, 20.0f, 1U, 0U);
  assert(std::fabs(budget - 60.0f) < 1e-5f);
  budget = HeatCtrl::AdvanceBudget(c, budget, 20.0f, 1U, 1U);
  assert(std::fabs(budget - 68.0f) < 1e-5f);
  budget = HeatCtrl::AdvanceBudget(c, budget, 20.0f, 0U, 1U);
  assert(std::fabs(budget - 66.0f) < 1e-5f);
  budget = HeatCtrl::AdvanceBudget(c, budget, 20.0f, 0U, 0U);
  assert(std::fabs(budget - 66.0f) < 1e-5f);
}
