# InfantryLauncher

The launcher consumes the native `online_info` topic (`SentryInfoOnline`) for
the referee cooling rate, heat limit, remaining 17 mm heat, and remaining
projectiles. Set `heat_control_enabled: true` in the module constructor
arguments to enable dynamic trigger-frequency limiting and fail-closed behavior
when online-info heat data is invalid or stale. Set it to `false` only for
controlled testing; this bypasses heat limiting and freshness checks while
retaining motor fault and friction-wheel readiness protection.

The feedforward parameters are `single_heat` (heat per projectile),
`max_frequency` (rounds/s), `burst_duration` (seconds to preserve the requested
burst), and `heat_margin` (reserved heat). Defaults are 10, 15, 2.0, and 10.
Invalid values, including a negative heat margin, fail closed.
For remaining usable heat `R`, cooling rate `a`, single-shot heat `d`, and
`n = ceil(burst_duration / 0.1)`, the target frequency is bounded by
`(10R-a)/(d*n)+a/d`, clamped between the sustainable `a/d` and the configured
maximum. Less than one shot of usable heat denies firing; a three-shot command
must have the full three-shot budget before trigger motion starts.

The local prediction applies cooling only at completed 100 ms referee
settlement boundaries. Shots detected from trigger-wheel progress are charged
immediately, before any later boundary cooling. The larger of referee-observed
heat and locally predicted heat is always used.

## Referee-state freshness

发射机构依赖跨板 `online_info` 中的热量数据进行发射许可。模块会记录最近一次
`SentryInfoOnline` 接收时间，超过 300 ms 未收到新数据时，将裁判数据视为失效。

数据失效后，模块会清除当前发射意图，禁止拨弹，拨弹频率和摩擦轮目标转速归零，并把 READY 摩擦轮模式收敛到已有的 SAFE 状态。后续收到新鲜裁判数据时不会自动恢复 READY 或自动发射，仍需由现有事件和状态机重新进入发射准备流程。
