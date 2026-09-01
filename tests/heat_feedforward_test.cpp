#include <cassert>
#include <cmath>

#include "../HeatFeedforward.hpp"

int main() {
  using namespace launcher;
  HeatFeedforwardConfig c;
  HeatFeedforwardObservation o{100.0f, 30.0f, 20.0f, true};
  auto r = HeatFeedforward::Calculate(c, o);
  assert(r.allow_fire && std::fabs(r.target_frequency - 4.9f) < 1e-5f);
  o.current_heat = 85.0f;
  assert(!HeatFeedforward::Calculate(c, o).allow_fire);
  o.current_heat = 60.0f;
  o.cooling_rate = 1000.0f;
  assert(std::fabs(HeatFeedforward::Calculate(c, o).target_frequency - 15.0f) <
         1e-5f);
  c.max_frequency = 100.0f;
  o.current_heat = 80.0f;
  o.cooling_rate = 200.0f;
  assert(std::fabs(HeatFeedforward::Calculate(c, o).target_frequency - 20.0f) <
         1e-5f);
  c.max_frequency = 15.0f;
  o.cooling_rate = 0.0f;
  o.current_heat = 50.0f;
  assert(HeatFeedforward::Calculate(c, o, 3).allow_fire);
  o.current_heat = 61.0f;
  assert(!HeatFeedforward::Calculate(c, o, 3).allow_fire);
  o.data_valid = false;
  assert(!HeatFeedforward::Calculate(c, o).allow_fire);
  c.single_heat = NAN;
  o.data_valid = true;
  assert(!HeatFeedforward::Calculate(c, o).allow_fire);

  // At a 100 ms boundary a 10-heat shot settles before 20/s cooling: 50+10-2.
  c.single_heat = 10.0f;
  assert(std::fabs(HeatFeedforward::AdvanceBudget(c, 50.0f, 20.0f, 1U, 1U) -
                   58.0f) < 1e-5f);
  // No completed period means no cooling may be released early.
  assert(std::fabs(HeatFeedforward::AdvanceBudget(c, 50.0f, 20.0f, 1U, 0U) -
                   60.0f) < 1e-5f);

  // 2.01 s and 2.05 s both require 21 complete 100 ms planning periods.
  c.burst_duration = 2.01f;
  o = {100.0f, 30.0f, 20.0f, true};
  assert(std::fabs(HeatFeedforward::Calculate(c, o).target_frequency -
                   4.7619047f) < 1e-5f);
  c.burst_duration = 2.05f;
  assert(std::fabs(HeatFeedforward::Calculate(c, o).target_frequency -
                   4.7619047f) < 1e-5f);
  c.heat_margin = -1.0f;
  assert(!HeatFeedforward::Calculate(c, o).allow_fire);

  // Sequence: shot at 50 ms -> 60; at 100 ms another shot then cool -> 68;
  // through 250 ms only the 200 ms boundary releases another 2 heat -> 66.
  c = {};
  float budget = HeatFeedforward::AdvanceBudget(c, 50.0f, 20.0f, 1U, 0U);
  assert(std::fabs(budget - 60.0f) < 1e-5f);
  budget = HeatFeedforward::AdvanceBudget(c, budget, 20.0f, 1U, 1U);
  assert(std::fabs(budget - 68.0f) < 1e-5f);
  budget = HeatFeedforward::AdvanceBudget(c, budget, 20.0f, 0U, 1U);
  assert(std::fabs(budget - 66.0f) < 1e-5f);
  budget = HeatFeedforward::AdvanceBudget(c, budget, 20.0f, 0U, 0U);
  assert(std::fabs(budget - 66.0f) < 1e-5f);
}
