# InfantryLauncher

The launcher consumes the native `online_info` topic (`SentryInfoOnline`) for
the referee cooling rate, heat limit, remaining 17 mm heat, and remaining
projectiles. Set `heat_control_enabled: true` in the module constructor
arguments to enable dynamic trigger-frequency limiting and fail-closed behavior
when online-info heat data is invalid or stale. Set it to `false` only for
controlled testing; this bypasses heat limiting and freshness checks while
retaining motor fault and friction-wheel readiness protection.

The heat controller parameters are `single_heat` (heat per projectile, `d`)
and `max_frequency` (rounds/s, the full-rate ceiling). Default values are 10
and 15; a non-positive `single_heat` fails closed.

The rate scheduler is a faithful port of the rmcod2026 sentry `Fire_Ctrl`
algorithm, stepped at 1 kHz on the launcher thread. With remaining heat
`m = heat_limit - current_heat` and cooling rate `a` (both from referee data):

- `m > 100`: full rate, target frequency = `max_frequency`.
- `20 < m <= 100`: burst/sustainable scheduling. On burst-window open the
  window length `shoot_time = (m + 2a) * 10` ms (clamped to 100–5600 ms) and
  the burst rate `(d*m - a - k*d) / (d * shoot_time/100) + a/d` are fixed at
  once (`k = 3` for `m < 50`, else `k = 7`). After the window elapses the rate
  falls back to the sustainable `a/d` (zero when below 1). With `m >= 40` a
  new window opens from fresh heat once the count elapses; with `m <= 25` the
  scheduler latches the sustainable rate; between 25 and 40 it holds it until
  heat recovers.
- `m <= 20`: firing denied.

Compared with the source, the `m == 100` exact-equality fall-through gap of
the original else-if chain is closed, and the stop threshold is the
chain-effective value 20.

The local prediction applies cooling only at completed 100 ms referee
settlement boundaries. Shots detected from trigger-wheel progress are charged
immediately, before any later boundary cooling. The larger of referee-observed
heat and locally predicted heat is always used.

## Referee-state freshness

发射机构依赖跨板 `online_info` 中的热量数据进行发射许可。模块会记录最近一次
`SentryInfoOnline` 接收时间，超过 300 ms 未收到新数据时，将裁判数据视为失效。

数据失效后，模块会清除当前发射意图，禁止拨弹，拨弹频率和摩擦轮目标转速归零，并把 READY 摩擦轮模式收敛到已有的 SAFE 状态。后续收到新鲜裁判数据时不会自动恢复 READY 或自动发射，仍需由现有事件和状态机重新进入发射准备流程。
