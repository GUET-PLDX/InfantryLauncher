#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_0'
  - motor_fric_front_right: '@&motor_fric_1'
  - motor_trig: '@&motor_trig'
  - task_stack_depth: 4096
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 4000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.0005
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_0:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_1:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - launcher_param:
      fric1_setpoint_speed: 6500.0
      target_bullet_speed: 25.0
      bullet_speed_tolerance: 1.5
      trig_gear_ratio: 36.0
      num_trig_tooth: 10
  - heat_control_enabled: true
  - single_heat: 10.0
  - max_frequency: 15.0
  - cmd: '@&cmd'
  - referee: '@nullptr'
  - thread_priority: LibXR::Thread::Priority::HIGH
required_hardware:
  - dr16
  - can
depends:
  - pldx/CMD
  - pldx/RMMotor
  - pldx/NavHostData
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "NavHostData.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace launcher {

constexpr float HEAT_SETTLEMENT_PERIOD_SEC = 0.1f;

/* === 热量控制（移植自 rmcod2026 哨兵工程 Fire_Ctrl，1 kHz 节拍）===
 * 以裁判系统剩余热量 m = heat_limit - current_heat（源工程 SYS_Qres）为输入，
 * 分三档调度允许射频（单位：发/秒）：
 *   m > 100        ：全速档，射频 = max_frequency（源工程固定 18 发/秒）；
 *   20 < m <= 100  ：突发-持续两段式调度。突发段开窗时一次算定：
 *     时长 shoot_time = (m + 2*cooling) * 10 ms（限幅 100~5600 ms），
 *     射频 = (d*m - a - k*d) / (d * shoot_time/100) + a/d，
 *     其中 d 为单发热量、a 为冷却速率，m < 50 取余量 k=3，否则 k=7；
 *     突发段结束后回落至可持续射频 a/d（< 1 发/秒视为 0）；
 *     m >= 40 时突发段计满后按新热量重新开窗，m <= 25 时锁定可持续射频，
 *     25 < m < 40 时维持可持续射频直至热量回升；
 *   m <= 20        ：停止发射。
 * 与源工程的差异：闭合了 m == 100 恰等时 else-if 链全部落空、射频保持陈旧值
 * 的缝隙（裁判热量为整数量纲，恰等可复现）；停射阈值取分支链的实际生效值 20。
 * 本地热量估计（源工程 Heat_Detection 的职能：记弹 + 冷却积分 + 上限封顶）
 * 由 InfantryLauncher::CurrentHeat 的拨弹齿进度记账 + AdvanceBudget 周期结算
 * 承担，并以 std::max 与裁判热量融合，取代源工程的摩擦轮电流尖峰检弹。 */
class HeatCtrl {
 public:
  struct Config {
    float single_heat = 10.0f;   /* 单发热量 d */
    float max_frequency = 15.0f; /* 射频上限 = 全速档射频 */
  };

  struct Observation {
    float heat_limit = 0.0f;
    float current_heat = 0.0f;
    float cooling_rate = 0.0f;
    bool data_valid = false;
  };

  struct Result {
    bool allow_fire = false;
    float target_frequency = 0.0f;
  };

  /* 本地热量记账的周期结算：先记弹、后按完成周期数冷却（100 ms 一结算） */
  static float AdvanceBudget(const Config& config, float current_heat,
                             float cooling_rate, unsigned int launched_shots,
                             unsigned int completed_periods) {
    if (!std::isfinite(current_heat) || !std::isfinite(cooling_rate) ||
        !std::isfinite(config.single_heat) || config.single_heat <= 0.0f ||
        cooling_rate < 0.0f) {
      return current_heat;
    }
    const float settled_heat =
        current_heat + config.single_heat * static_cast<float>(launched_shots);
    return std::max(0.0f,
                    settled_heat - cooling_rate * HEAT_SETTLEMENT_PERIOD_SEC *
                                       static_cast<float>(completed_periods));
  }

  /* 弹频调度步进：每毫秒调用一次（对齐源工程 Fire_Ctrl 的调用节拍） */
  Result Update(const Config& config, const Observation& obs) {
    if (!obs.data_valid || config.single_heat <= 0.0f) {
      Reset();
      return {};
    }
    const float d = config.single_heat;
    const float a = obs.cooling_rate;
    const float m = obs.heat_limit - obs.current_heat; /* 剩余热量 */

    if (m > BURST_ZONE_HIGH) {
      rate_ = config.max_frequency; /* 全速档 */
    } else if (m > BURST_ZONE_LOW) {
      /* 突发-持续档：开窗周期只算定参数，输出沿用上一周期射频 */
      if (shoot_count_ == 0.0f) {
        shoot_time_ = std::clamp((m + 2.0f * a) * 10.0f, BURST_TIME_MIN_MS,
                                 BURST_TIME_MAX_MS);
        const float headroom = (m < 50.0f) ? 3.0f : 7.0f;
        burst_rate_ =
            (d * m - a - headroom * d) / (d * (shoot_time_ / 100.0f)) + a / d;
      }
      if (shoot_count_ > 0.0f && shoot_count_ < shoot_time_) {
        rate_ = std::clamp(burst_rate_, 0.0f, config.max_frequency);
      } else {
        rate_ = a / d; /* 可持续射频 = 冷却速率 / 单发热量 */
        if (rate_ < 1.0f) {
          rate_ = 0.0f;
        }
        rate_ = std::clamp(rate_, 0.0f, config.max_frequency);
      }
      last_shoot_time_ = shoot_time_;
      if (shoot_count_ < shoot_time_) {
        shoot_count_ += 1.0f;
      }
      if (m >= 40.0f) {
        if (shoot_count_ >= shoot_time_) {
          shoot_count_ = 0.0f; /* 突发段计满，下一周期按新热量重新开窗 */
        }
      } else if (m <= 25.0f) {
        shoot_count_ = last_shoot_time_; /* 低热量：锁定可持续射频 */
      }
    } else {
      rate_ = 0.0f; /* 停射档 */
    }

    return {rate_ > 0.0f, rate_};
  }

  void Reset() {
    rate_ = 0.0f;
    burst_rate_ = 0.0f;
    shoot_count_ = 0.0f;
    shoot_time_ = 0.0f;
    last_shoot_time_ = 0.0f;
  }

 private:
  /* 源工程 Fire_Ctrl 的分档阈值与突发段时长限幅（单位 ms） */
  static constexpr float BURST_ZONE_HIGH = 100.0f;
  static constexpr float BURST_ZONE_LOW = 20.0f;
  static constexpr float BURST_TIME_MIN_MS = 100.0f;
  static constexpr float BURST_TIME_MAX_MS = 5600.0f;

  /* 源工程 Shoot_DP / Shoot_Speed / ShootCount / ShootTime / Last_Shoot_time */
  float rate_ = 0.0f;
  float burst_rate_ = 0.0f;
  float shoot_count_ = 0.0f;
  float shoot_time_ = 0.0f;
  float last_shoot_time_ = 0.0f;
};

}  // namespace launcher

namespace launcher::param {
constexpr float TRIG_STEP = static_cast<float>(LibXR::TWO_PI) / 10.0f;
constexpr float JAM_TORQUE = 0.028f;
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.1f;
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
constexpr uint32_t HEAT_SETTLEMENT_PERIOD_MS = 100U;
constexpr float SHOT_PROGRESS_EPSILON = 1e-4f;
constexpr float TRIGGER_SETTLE_ANGLE = 0.2f * TRIG_STEP;
constexpr float FRIC_READY_RPM_MARGIN = 200.0f;
constexpr float FRIC_DROP_RPM = 150.0f;
constexpr uint32_t ONLINE_INFO_TIMEOUT_MS = 300;
}  // namespace launcher::param

/**
 * @brief 步兵发射机构实现
 * @details
 * 负责摩擦轮、拨弹盘控制与热量约束发射逻辑。
 */
class InfantryLauncher {
 public:
  enum class LauncherState : uint8_t {
    RELAX,
    STOP,
    NORMAL,
    JAMMED,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
    SET_SHOTMODE_SINGLE,
    SET_SHOTMODE_CONTINUE,
    SET_SHOTMODE_BOOST_3,
  };

  enum class TrigMode : uint8_t {
    RELAX,
    SAFE,
    SINGLE,
    CONTINUE,
    JAM,
  };

  struct RefereeData {
    float cooling_rate = 0.0f;
    float heat_limit = 0.0f;
    float current_heat_17 = 0.0f;
    float bullet_speed = 0.0f;
  };

  struct LauncherParam {
    float fric1_setpoint_speed;
    float target_bullet_speed;
    float bullet_speed_tolerance;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
  };

  struct HeatLimit {
    float launched_num;
    float current_heat;
    bool allow_fire;
  };

  InfantryLauncher(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      RMMotor* motor_fric_0, RMMotor* motor_fric_1, RMMotor* motor_trig,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_param_trig_angle,
      LibXR::PID<float>::Param pid_param_trig_speed,
      LibXR::PID<float>::Param pid_param_fric_speed_0,
      LibXR::PID<float>::Param pid_param_fric_speed_1,
      LauncherParam launcher_param, bool heat_control_enabled,
      float single_heat, float max_frequency, CMD* cmd,
      Referee* referee = nullptr,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : PARAM(launcher_param),
        HEAT_CONTROL_ENABLED(heat_control_enabled),
        HEAT_CONFIG{single_heat, max_frequency},
        motors_{
            .fric_0 = motor_fric_0, .fric_1 = motor_fric_1, .trig = motor_trig},
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_speed_0),
        pid_fric_1_(pid_param_fric_speed_1),
        feedback_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavHostData::GimbalFeedbackV1>(
                Pldx::NavHostData::LAUNCHER_FEEDBACK_TOPIC, nullptr, true)),
        referee_(referee) {
    trig_.expect_freq = max_frequency;
    fric_.expect_rpm = PARAM.fric1_setpoint_speed;
    UNUSED(hw);
    UNUSED(app);

    thread_.Create(this, ThreadFunc, "LauncherThread", task_stack_depth,
                   thread_priority);

    if (referee_ != nullptr) {
      timer_ui_ = LibXR::Timer::CreateTask(DrawUI, this, UI_REFRESH_PERIOD_MS);
      LibXR::Timer::Add(timer_ui_);
      LibXR::Timer::Start(timer_ui_);
    }

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    cmd->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_SINGLE),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_CONTINUE),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_SHOTMODE_BOOST_3),
        event_callback);
  }

  static void ThreadFunc(InfantryLauncher* self) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub("launcher_cmd");
    LibXR::Topic::ASyncSubscriber<Pldx::NavHostData::SentryInfoOnline>
        online_info_sub(Pldx::NavHostData::ONLINE_INFO_TOPIC);
    cmd_sub.StartWaiting();
    online_info_sub.StartWaiting();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (true) {
      auto now = LibXR::Timebase::GetMicroseconds();
      self->dt_ = (now - self->last_online_time_).ToSecondf();
      self->last_online_time_ = now;

      if (cmd_sub.Available()) {
        self->launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if (online_info_sub.Available()) {
        const auto online = online_info_sub.GetData();
        self->online_.valid = true;
        self->online_.last_rx_ms = LibXR::Timebase::GetMilliseconds();
        self->heat_.data_valid = online.heat_limit > 0U;
        if (self->heat_.data_valid) {
          self->ref_data_.heat_limit = static_cast<float>(online.heat_limit);
          self->ref_data_.cooling_rate =
              static_cast<float>(online.cooling_value);
          self->ref_data_.current_heat_17 =
              static_cast<float>(online.current_heat);
        }
        self->online_.bullet_count = online.bullets_remaining;
        self->online_.bullet_count_valid = true;
        online_info_sub.StartWaiting();
      }
      self->mutex_.Lock();
      self->Update();
      self->RunStateMachine();
      self->mutex_.Unlock();
      self->Control();
      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if (now_ms - self->last_feedback_publish_ms_ >= 10U) {
        self->PublishFeedback();
        self->last_feedback_publish_ms_ = now_ms;
      }
      LibXR::Thread::SleepUntil(last_time, 1);
    }
  }

  void Update() {
    motors_.fric_0_status = motors_.fric_0->Update();
    motors_.fric_1_status = motors_.fric_1->Update();
    motors_.trig_status = motors_.trig->Update();
    /* 2026-09-25：取消"三电机全在线才工作"的整组锁存，
       各电机在 Control() 中按自身在线状态独立 Relax/输出 */

    motors_.fric_0_fb = motors_.fric_0->GetFeedback();
    motors_.fric_1_fb = motors_.fric_1->GetFeedback();
    motors_.trig_fb = motors_.trig->GetFeedback();

    float current_motor_angle = motors_.trig_fb.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(trig_.last_motor_angle);
    trig_.angle += delta_trig_angle / PARAM.trig_gear_ratio;
    trig_.last_motor_angle = current_motor_angle;

    UpdateLauncherState();
  }

  void Control() {
    /* 各电机按自身在线状态独立输出：离线的电机保持 Relax，不再互相阻塞 */
    const bool TRIG_ONLINE = motors_.trig_status == LibXR::ErrorCode::OK;
    const bool FRIC_0_ONLINE = motors_.fric_0_status == LibXR::ErrorCode::OK;
    const bool FRIC_1_ONLINE = motors_.fric_1_status == LibXR::ErrorCode::OK;
    const bool relax = state_.event == LauncherEvent::SET_FRICMODE_RELAX;
    /* 拨弹盘输出每周期重新推导（松弛/离线时恒为零），从结构上杜绝陈旧扭矩 */
    const bool TRIG_ACTIVE =
        !relax && TRIG_ONLINE && state_.mode != TrigMode::RELAX;

    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 && fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    SetFricTargetByEvent();

    if (!relax) {
      if (TRIG_ACTIVE) {
        TrigControl(out_trig, trig_.target_angle, dt_);
      }
      FricControl(out_fric_0, out_fric_1, fric_.target_rpm, dt_);
      trig_fb = motors_.trig_fb;
      fric_0_fb = motors_.fric_0_fb;
      fric_1_fb = motors_.fric_1_fb;
    }

    if (!TRIG_ACTIVE) {
      motors_.trig->Relax();
    } else {
      auto cmd_trig = Motor::MotorCmd{
          .mode = Motor::ControlMode::MODE_CURRENT,
          .reduction_ratio = PARAM.trig_gear_ratio,
          .velocity = out_trig,
      };
      motor_control(motors_.trig, trig_fb, cmd_trig);
    }

    if (!FRIC_0_ONLINE || !FRIC_1_ONLINE) {
      motors_.fric_0->Relax();
      motors_.fric_1->Relax();
    } else {
      auto cmd_fric_0 =
          Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                          .reduction_ratio = 1.0f,
                          .velocity = out_fric_0};
      auto cmd_fric_1 =
          Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                          .reduction_ratio = 1.0f,
                          .velocity = out_fric_1};
      motor_control(motors_.fric_0, fric_0_fb, cmd_fric_0);
      motor_control(motors_.fric_1, fric_1_fb, cmd_fric_1);
    }
  }

  void SetMode(uint32_t mode) {
    auto event = static_cast<LauncherEvent>(mode);
    switch (event) {
      case LauncherEvent::SET_SHOTMODE_SINGLE:
        trig_.shot_count = 1;
        trig_.continue_mode = false;
        ui_.fire_mode_text_initialized = false;
        ui_.refresh_tick = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
        trig_.shot_count = 1;
        trig_.continue_mode = true;
        ui_.fire_mode_text_initialized = false;
        ui_.refresh_tick = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        trig_.shot_count = 3;
        trig_.continue_mode = false;
        ui_.fire_mode_text_initialized = false;
        ui_.refresh_tick = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
      case LauncherEvent::SET_FRICMODE_READY:
        state_.event = event;
        ui_.fric_text_initialized = false;
        ui_.refresh_tick = UI_FRIC_TEXT_PHASE;
        if (event != LauncherEvent::SET_FRICMODE_READY) {
          trig_.calibrated = false;
          trig_.calibration_pending = false;
          trig_.target_shot_index = 0;
          trig_.reverse = false;
        }
        break;
    }

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  void LostCtrl() {
    state_.event = LauncherEvent::SET_FRICMODE_RELAX;
    state_.state = LauncherState::RELAX;
    state_.mode = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    trig_.target_angle = trig_.angle;
    trig_.press_continue = false;
    trig_.calibrated = false;
    trig_.calibration_pending = false;
    trig_.step_active = false;
    trig_.target_shot_index = 0;
    trig_.reverse = false;
    trig_.progress = 0.0f;
    launcher_cmd_.isfire = false;
    ui_.fric_text_initialized = false;
    ui_.fire_mode_text_initialized = false;
    ui_.shot_position_initialized = false;
    ui_.refresh_tick = UI_FRIC_TEXT_PHASE;

    motors_.trig->Disable();
    motors_.fric_0->Relax();
    motors_.fric_1->Relax();
  }

  LibXR::Event& GetEvent() { return launcher_event; }

  void OnMonitor() {}

 private:
  // 发射机构 UI 使用的图层编号
  static constexpr uint8_t UI_LAYER_LAUNCHER = 1;
  // 发射机构 UI 文字共用的线宽和字号
  static constexpr uint16_t UI_CHAR_WIDTH = 2;
  static constexpr uint16_t UI_FONT_SIZE = 20;
  // 摩擦轮状态文字 ON/OFF 的显示位置
  static constexpr uint16_t UI_FRIC_TEXT_X = 160;
  static constexpr uint16_t UI_FRIC_TEXT_Y = 580;
  // 发射模式文字显示位置
  static constexpr uint16_t UI_FIRE_MODE_TEXT_X = 160;
  static constexpr uint16_t UI_FIRE_MODE_TEXT_Y = 540;
  // 实际落点圆圈显示位置
  static constexpr uint16_t UI_SHOT_POSITION_X = 960;
  static constexpr uint16_t UI_SHOT_POSITION_Y = 480;
  static constexpr uint16_t UI_SHOT_POSITION_RADIUS = 18;
  static constexpr uint16_t UI_SHOT_POSITION_WIDTH = 3;
  // 发射机构 UI 的刷新周期和分时重发节奏
  static constexpr uint32_t UI_REFRESH_PERIOD_MS = 80;
  static constexpr uint32_t UI_REFRESH_PHASE_COUNT = 3;
  static constexpr uint32_t UI_SHOT_POSITION_PHASE = 0;
  static constexpr uint32_t UI_FRIC_TEXT_PHASE = 1;
  static constexpr uint32_t UI_FIRE_MODE_TEXT_PHASE = 2;
  static constexpr uint32_t UI_TEXT_READD_DIV = 10;
  static constexpr uint32_t UI_FIGURE_READD_DIV = 60;

  /* === 状态分层（对齐 Gimbal/Omni 范式）===
   * 1. PARAM / HEAT_CONTROL_ENABLED / HEAT_CONFIG  构造期配置，const；
   * 2. motors_   执行器层：句柄 + 反馈 + 在线状态（错误码）；
   * 3. state_    模式层：外部事件 → 状态机 → 触发模式；
   * 4. trig_     拨弹盘子系统：步进/标定/卡弹/连发会话；
   * 5. fric_     摩擦轮子系统：转速目标与弹速微调；
   * 6. heat_     热量子系统：热量记账与 Fire_Ctrl 分档弹频调度门控；
   * 7. online_   导航主机在线信息新鲜度与弹量；
   * 8. ui_       裁判系统 UI 绘制状态。
   * 其余平铺成员与 Gimbal 同构：PID 组 / 时序 / 框架句柄。
   * 调试：watch 直接展开 launcher 对象按声明顺序查看。 */

  const LauncherParam PARAM;
  const bool HEAT_CONTROL_ENABLED;
  const launcher::HeatCtrl::Config HEAT_CONFIG;

  /* 执行器层（fb = 最近一次电机反馈，status = 最近一次 Update 结果） */
  struct Motors {
    RMMotor* fric_0;
    RMMotor* fric_1;
    RMMotor* trig;
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    Motor::Feedback trig_fb{};
    LibXR::ErrorCode fric_0_status = LibXR::ErrorCode::FAILED;
    LibXR::ErrorCode fric_1_status = LibXR::ErrorCode::FAILED;
    LibXR::ErrorCode trig_status = LibXR::ErrorCode::FAILED;
  };
  Motors motors_;

  /* 模式层（event = 遥控/事件注入的摩擦轮档位；state = 状态机输出；
   * mode = 触发模式；last_mode = 上一周期的触发模式，用于沿检测） */
  struct State {
    LauncherEvent event = LauncherEvent::SET_FRICMODE_RELAX;
    LauncherState state = LauncherState::RELAX;
    TrigMode mode = TrigMode::RELAX;
    TrigMode last_mode = TrigMode::RELAX;
  };
  State state_;

  /* 拨弹盘子系统（角度均为输出轴弧度；freq 单位 Hz） */
  struct Trig {
    float freq = 0.0f; /* 热量门控输出的当前允许射频 */
    float expect_freq =
        15.0f;          /* 热控关闭时的固定射频，构造期置 max_frequency */
    float angle = 0.0f; /* 拨弹盘累计转角 */
    float target_angle = 0.0f;        /* 当前步进目标角 */
    float last_motor_angle = 0.0f;    /* 电机侧上一周期角度（算增量） */
    float first_shot_angle = 0.0f;    /* 标定锚点：首发对齐的拨弹盘角度 */
    float jam_target_angle = 0.0f;    /* 卡弹反转的往返中心角 */
    float progress = 0.0f;            /* 已拨过的整格进度（热量记账） */
    float last_angle = 0.0f;          /* 上一周期 angle 快照（热量/标定共用） */
    float fric_speed_peak = 0.0f;     /* 标定达速判定的摩擦轮峰值转速 */
    int32_t target_shot_index = 0;    /* 目标格序号（相对 first_shot_angle） */
    uint8_t shot_count = 1;           /* 单次触发发数（SINGLE=1 / BOOST_3=3） */
    bool step_active = false;         /* 步进进行中 */
    bool reverse = false;             /* 卡弹反转方向 */
    bool calibrated = false;          /* 拨弹盘相位标定完成 */
    bool calibration_pending = false; /* 标定流程挂起 */
    bool continue_mode = false;       /* 连发模式（右拨杆中档） */
    bool press_continue = false;      /* 长按升级为连发 */
    bool last_fire_notify = false;    /* 上一周期 isfire（上升沿检测） */
    LibXR::MillisecondTimestamp fire_press_time = 0;
    LibXR::MillisecondTimestamp last_trig_time = 0;
    LibXR::MillisecondTimestamp last_jam_time = 0;
  };
  Trig trig_;

  /* 摩擦轮子系统（rpm 为电机侧转速） */
  struct Fric {
    float target_rpm = 0.0f; /* 当前 PID 目标 */
    float expect_rpm = 0.0f; /* 弹速闭环微调后的期望转速，构造期置初值 */
    float last_bullet_speed = -1.0f; /* 上次参与微调的弹速（变化沿检测） */
  };
  Fric fric_;

  /* 热量子系统（limit 即原 HeatLimit：记账 + 门控；ctrl_ 为弹频调度器） */
  struct Heat {
    HeatLimit limit{
        .launched_num = 0.0f, .current_heat = 0.0f, .allow_fire = true};
    bool initialized = false; /* 首周期跳过热量增量 */
    bool data_valid = false;  /* online 热量字段是否有效 */
    LibXR::MillisecondTimestamp last_check_time = 0; /* 结算周期锚点 */
  };
  Heat heat_;
  launcher::HeatCtrl heat_ctrl_;

  /* 在线信息（Pldx::NavHostData::SentryInfoOnline）新鲜度与弹量 */
  struct OnlineInfo {
    bool valid = false; /* IsOnlineInfoFresh() 的第一个条件 */
    bool bullet_count_valid = false;
    uint16_t bullet_count = 0;
    LibXR::MillisecondTimestamp last_rx_ms = 0;
  };
  OnlineInfo online_;

  /* 裁判系统 UI 绘制状态（referee_ 非空时才活动） */
  struct Ui {
    bool layer_cleared = false;
    bool fric_text_initialized = false;
    bool fire_mode_text_initialized = false;
    bool shot_position_initialized = false;
    uint32_t refresh_tick = 0; /* 分时重发相位计数 */
  };
  Ui ui_;

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LibXR::Topic feedback_topic_;

  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  uint32_t last_feedback_publish_ms_ = 0U;

  CMD::LauncherCMD launcher_cmd_{};
  RefereeData ref_data_;

  Referee* referee_ = nullptr;
  LibXR::Event launcher_event;
  LibXR::Thread thread_;
  LibXR::Timer::TimerHandle timer_ui_{};
  LibXR::Mutex mutex_;

  void PublishFeedback() {
    LibXR::Mutex::LockGuard lock(mutex_);
    const auto now = LibXR::Timebase::GetMilliseconds();
    Pldx::NavHostData::GimbalFeedbackV1 feedback{};
    feedback.bullet_speed_mps = ref_data_.bullet_speed;
    feedback.bullet_count = online_.bullet_count;
    if (IsOnlineInfoFresh(now) && std::isfinite(feedback.bullet_speed_mps) &&
        feedback.bullet_speed_mps > 0.0F) {
      feedback.valid_flags |= Pldx::NavHostData::GIMBAL_BULLET_SPEED_VALID;
    }
    if (online_.bullet_count_valid && IsOnlineInfoFresh(now)) {
      feedback.valid_flags |= Pldx::NavHostData::GIMBAL_BULLET_COUNT_VALID;
    }
    feedback_topic_.Publish(feedback);
  }

  void UpdateLauncherState() {
    if (motors_.trig_fb.torque > launcher::param::JAM_TORQUE) {
      state_.state = LauncherState::JAMMED;
      return;
    }
    if (state_.event != LauncherEvent::SET_FRICMODE_READY) {
      state_.state = LauncherState::RELAX;
      return;
    }

    if (!heat_.limit.allow_fire) {
      state_.state = LauncherState::STOP;
      return;
    }

    state_.state =
        launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
  }

  void RunStateMachine() {
    auto now = LibXR::Timebase::GetMilliseconds();
    UpdateOnlineInfoFreshness(now);
    CurrentHeat(now);
    UpdateHeatControl(now);
    UpdateLauncherState();
    UpdateTriggerMode(now);
    UpdateTriggerSetpoint(now);

    trig_.last_fire_notify = launcher_cmd_.isfire;
  }

  bool IsOnlineInfoFresh(LibXR::MillisecondTimestamp now) const {
    return online_.valid && (now - online_.last_rx_ms).ToMillisecond() <=
                                launcher::param::ONLINE_INFO_TIMEOUT_MS;
  }

  void UpdateOnlineInfoFreshness(LibXR::MillisecondTimestamp now) {
    if (!HEAT_CONTROL_ENABLED) {
      heat_.limit.allow_fire = true;
      trig_.freq = trig_.expect_freq;
      return;
    }
    if (IsOnlineInfoFresh(now) && heat_.data_valid) {
      return;
    }

    online_.valid = false;
    heat_.data_valid = false;
    ref_data_ = RefereeData{};
    heat_ctrl_.Reset(); /* 裁判数据失效：清空弹频调度状态，恢复后重新开窗 */
    heat_.limit.allow_fire = false;
    trig_.freq = 0.0f;
    fric_.target_rpm = 0.0f;
    launcher_cmd_.isfire = false;
    trig_.press_continue = false;
    trig_.step_active = false;
    trig_.calibration_pending = false;
    trig_.reverse = false;
    trig_.progress = 0.0f;
    trig_.target_angle = trig_.angle;

    // 裁判热量或弹速数据失效时，强制回到已有安全摩擦轮模式。
    if (state_.event == LauncherEvent::SET_FRICMODE_READY) {
      state_.event = LauncherEvent::SET_FRICMODE_SAFE;
      ui_.fric_text_initialized = false;
      ui_.refresh_tick = UI_FRIC_TEXT_PHASE;
      pid_fric_0_.Reset();
      pid_fric_1_.Reset();
      pid_trig_angle_.Reset();
      pid_trig_sp_.Reset();
    }
  }

  void UpdateTriggerMode(LibXR::MillisecondTimestamp now) {
    switch (state_.state) {
      case LauncherState::RELAX:
        state_.mode = TrigMode::RELAX;
        trig_.press_continue = false;
        break;

      case LauncherState::STOP:
        state_.mode = TrigMode::SAFE;
        trig_.press_continue = false;
        break;

      case LauncherState::NORMAL:
        if (trig_.continue_mode) {
          trig_.press_continue = true;
          state_.mode = TrigMode::CONTINUE;
        } else if (!trig_.last_fire_notify) {
          trig_.fire_press_time = now;
          trig_.press_continue = false;
          state_.mode = TrigMode::SINGLE;
        } else {
          if (!trig_.press_continue &&
              (now - trig_.fire_press_time).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            trig_.press_continue = true;
          }
          state_.mode =
              trig_.press_continue ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        state_.mode = TrigMode::JAM;
        break;
    }
  }

  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    const float step = launcher::param::TRIG_STEP;
    const float ready_rpm =
        fric_.expect_rpm - launcher::param::FRIC_READY_RPM_MARGIN;
    const float fric_speed = (fabsf(motors_.fric_0_fb.velocity) +
                              fabsf(motors_.fric_1_fb.velocity)) *
                             0.5f;

    auto indexed_target = [&]() {
      return trig_.first_shot_angle +
             step * static_cast<float>(trig_.target_shot_index);
    };

    auto next_indexed_target = [&]() {
      trig_.target_shot_index = static_cast<int32_t>(
          ceilf((trig_.angle - trig_.first_shot_angle) / step));
      return indexed_target();
    };

    auto recover_from_jam = [&]() {
      trig_.target_angle =
          trig_.calibrated ? next_indexed_target() : trig_.jam_target_angle;
      trig_.reverse = false;
      trig_.step_active = true;
      trig_.last_trig_time = now;
    };

    if (trig_.step_active) {
      trig_.fric_speed_peak = std::max(trig_.fric_speed_peak, fric_speed);

      if (trig_.calibration_pending && !trig_.calibrated &&
          trig_.fric_speed_peak >= ready_rpm &&
          trig_.fric_speed_peak - fric_speed >=
              launcher::param::FRIC_DROP_RPM) {
        trig_.calibrated = true;
        trig_.calibration_pending = false;
        trig_.first_shot_angle = trig_.angle;
        trig_.target_angle = indexed_target();
        heat_.limit.current_heat =
            std::max(heat_.limit.current_heat, ref_data_.current_heat_17) +
            HEAT_CONFIG.single_heat;
        trig_.progress = 0.0f;
        trig_.last_angle = trig_.angle;
      }

      float angle_error = fabsf(trig_.target_angle - trig_.angle);
      if (angle_error <= launcher::param::TRIGGER_SETTLE_ANGLE) {
        trig_.step_active = false;
        if (trig_.calibration_pending && !trig_.calibrated) {
          trig_.calibration_pending = false;
        }
      }
    } else {
      trig_.fric_speed_peak = fric_speed;
    }

    auto start_shot = [&]() {
      /* 热量门控：UpdateHeatControl 已按 Fire_Ctrl 分档给出本周期允许射频 */
      if (HEAT_CONTROL_ENABLED && !heat_.limit.allow_fire) {
        return;
      }

      if (trig_.calibrated) {
        trig_.target_shot_index += static_cast<int32_t>(trig_.shot_count);
        trig_.target_angle = indexed_target();
      } else {
        trig_.calibration_pending = true;
        trig_.target_shot_index = static_cast<int32_t>(trig_.shot_count) - 1;
        trig_.fric_speed_peak = fric_speed;
        trig_.target_angle =
            trig_.angle + step * static_cast<float>(trig_.shot_count);
      }

      trig_.step_active = true;
      trig_.last_trig_time = now;
    };

    switch (state_.mode) {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        trig_.target_angle = trig_.angle;
        trig_.step_active = false;
        trig_.calibration_pending = false;
        trig_.reverse = false;
        break;

      case TrigMode::SINGLE:
        if (state_.last_mode == TrigMode::JAM) {
          recover_from_jam();
        } else if (state_.last_mode == TrigMode::SAFE ||
                   state_.last_mode == TrigMode::RELAX) {
          start_shot();
        }
        break;

      case TrigMode::CONTINUE: {
        float trig_freq = std::max(trig_.freq, 1e-3f);
        float interval_s = 1.0f / trig_freq;
        float since_last = (now - trig_.last_trig_time).ToSecondf();
        if (state_.last_mode == TrigMode::JAM) {
          recover_from_jam();
        } else if (!trig_.step_active && since_last >= interval_s) {
          start_shot();
        }
      } break;

      case TrigMode::JAM: {
        trig_.step_active = false;
        if (state_.last_mode != TrigMode::JAM) {
          trig_.jam_target_angle =
              trig_.calibrated ? indexed_target() : trig_.target_angle;
          trig_.reverse = false;
        }
        if (state_.last_mode != TrigMode::JAM ||
            (now - trig_.last_jam_time).ToSecondf() >=
                launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          trig_.target_angle = trig_.reverse ? trig_.jam_target_angle
                                             : trig_.angle - 0.3f * step;
          trig_.reverse = !trig_.reverse;
          trig_.last_jam_time = now;
        }
      } break;
    }

    state_.last_mode = state_.mode;
  }

  void SetFricTargetByEvent() {
    switch (state_.event) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        fric_.target_rpm = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY: {
        if (HEAT_CONTROL_ENABLED &&
            (!IsOnlineInfoFresh(LibXR::Timebase::GetMilliseconds()) ||
             !heat_.data_valid)) {
          fric_.target_rpm = 0.0f;
          break;
        }

        // 根据裁判系统回传弹速微调摩擦轮期望转速
        float bullet_speed = ref_data_.bullet_speed;
        if (bullet_speed <= 0.0f || bullet_speed > 30.0f) {
          bullet_speed =
              PARAM.target_bullet_speed - 2.0f * PARAM.bullet_speed_tolerance;
        }

        if (fric_.last_bullet_speed != bullet_speed) {
          if (bullet_speed >
              PARAM.target_bullet_speed - PARAM.bullet_speed_tolerance) {
            fric_.expect_rpm -= 70.0f;
          }

          if (bullet_speed <
              PARAM.target_bullet_speed - 2.2f * PARAM.bullet_speed_tolerance) {
            fric_.expect_rpm += 50.0f;
          }
          fric_.last_bullet_speed = bullet_speed;
        }

        fric_.target_rpm = fric_.expect_rpm;
        break;
      }
      case LauncherEvent::SET_SHOTMODE_SINGLE:
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        break;
    }
  }

  void UpdateHeatControl(LibXR::MillisecondTimestamp now) {
    if (!HEAT_CONTROL_ENABLED) {
      heat_.limit.allow_fire = true;
      trig_.freq = trig_.expect_freq;
      return;
    }
    const float current_heat =
        std::max(heat_.limit.current_heat, ref_data_.current_heat_17);
    const auto decision = heat_ctrl_.Update(
        HEAT_CONFIG,
        {ref_data_.heat_limit, current_heat, ref_data_.cooling_rate,
         heat_.data_valid && IsOnlineInfoFresh(now)});
    heat_.limit.allow_fire = decision.allow_fire;
    trig_.freq = decision.target_frequency;
  }

  void CurrentHeat(LibXR::MillisecondTimestamp now) {
    if (!heat_.initialized) {
      heat_.initialized = true;
      heat_.last_check_time = now;
      trig_.last_angle = trig_.angle;
      return;
    }

    heat_.limit.current_heat =
        std::max(heat_.limit.current_heat, ref_data_.current_heat_17);
    heat_.limit.launched_num = 0.0f;

    float delta_teeth =
        (trig_.angle - trig_.last_angle) / launcher::param::TRIG_STEP;
    trig_.last_angle = trig_.angle;

    if (state_.event == LauncherEvent::SET_FRICMODE_READY) {
      trig_.progress += delta_teeth;
      if (trig_.progress < 0.0f) {
        trig_.progress = 0.0f;
      }
    } else {
      trig_.progress = 0.0f;
    }

    if (trig_.calibrated &&
        trig_.progress >= 1.0f - launcher::param::SHOT_PROGRESS_EPSILON) {
      heat_.limit.launched_num = floorf(trig_.progress);
      trig_.progress -= heat_.limit.launched_num;
    }

    const auto elapsed_ms = (now - heat_.last_check_time).ToMillisecond();
    const uint32_t periods =
        elapsed_ms / launcher::param::HEAT_SETTLEMENT_PERIOD_MS;
    heat_.limit.current_heat = launcher::HeatCtrl::AdvanceBudget(
        HEAT_CONFIG, heat_.limit.current_heat, ref_data_.cooling_rate,
        static_cast<uint32_t>(heat_.limit.launched_num), periods);
    if (periods > 0U) {
      heat_.last_check_time = LibXR::MillisecondTimestamp(
          static_cast<uint32_t>(heat_.last_check_time) +
          periods * launcher::param::HEAT_SETTLEMENT_PERIOD_MS);
    }
  }

  static void DrawUI(InfantryLauncher* launcher) {
    if (launcher->referee_ == nullptr) {
      return;
    }

    const uint16_t ROBOT_ID = launcher->referee_->GetRobotID();
    if (ROBOT_ID == 0) {
      return;
    }
    const uint16_t CLIENT_ID = launcher->referee_->GetClientID(ROBOT_ID);

    launcher->mutex_.Lock();
    const uint32_t UI_TICK = launcher->ui_.refresh_tick++;
    const uint32_t UI_PHASE = UI_TICK % UI_REFRESH_PHASE_COUNT;
    const bool FORCE_TEXT_READD =
        (UI_TICK % UI_TEXT_READD_DIV) < UI_REFRESH_PHASE_COUNT;
    const bool FRIC_ENABLED =
        launcher->state_.event == LauncherEvent::SET_FRICMODE_READY;
    const uint8_t SHOT_COUNT = launcher->trig_.shot_count;
    const TrigMode TRIG_MODE = launcher->state_.mode;
    const bool CONTINUE_MODE = launcher->trig_.continue_mode;
    const bool PRESS_CONTINUE = launcher->trig_.press_continue;
    const bool UI_LAYER_CLEARED = launcher->ui_.layer_cleared;
    const bool UI_FRIC_TEXT_INITIALIZED = launcher->ui_.fric_text_initialized;
    const bool UI_FIRE_MODE_TEXT_INITIALIZED =
        launcher->ui_.fire_mode_text_initialized;
    const bool UI_SHOT_POSITION_INITIALIZED =
        launcher->ui_.shot_position_initialized;
    launcher->mutex_.Unlock();

    if (!UI_LAYER_CLEARED) {
      if (UI_PHASE != 0) {
        return;
      }

      Referee::UILayerDelete ui_del{};
      ui_del.delete_type =
          static_cast<uint8_t>(Referee::UIDeleteType::UI_DELETE_LAYER);
      ui_del.layer = UI_LAYER_LAUNCHER;
      if (launcher->referee_->SendUILayerDelete(ROBOT_ID, CLIENT_ID, ui_del) !=
          LibXR::ErrorCode::OK) {
        return;
      }

      launcher->mutex_.Lock();
      launcher->ui_.layer_cleared = true;
      launcher->mutex_.Unlock();
      return;
    }

    if (UI_PHASE == UI_SHOT_POSITION_PHASE) {
      const bool REBUILD_SHOT_POSITION =
          !UI_SHOT_POSITION_INITIALIZED || (UI_TICK % UI_FIGURE_READD_DIV) == 0;
      if (REBUILD_SHOT_POSITION) {
        Referee::UIFigure shot_position_fig{};
        // 绘制实际落点圆圈
        launcher->referee_->FillCircle(
            shot_position_fig, "BPT", Referee::UIFigureOp::UI_OP_ADD,
            UI_LAYER_LAUNCHER, Referee::UIColor::UI_COLOR_YELLOW,
            UI_SHOT_POSITION_WIDTH, UI_SHOT_POSITION_X, UI_SHOT_POSITION_Y,
            UI_SHOT_POSITION_RADIUS);
        if (launcher->referee_->SendUIFigure(ROBOT_ID, CLIENT_ID,
                                             shot_position_fig) ==
            LibXR::ErrorCode::OK) {
          launcher->mutex_.Lock();
          launcher->ui_.shot_position_initialized = true;
          launcher->mutex_.Unlock();
        }
      }
      return;
    }

    Referee::UICharacter char_fig{};
    if (UI_PHASE == UI_FRIC_TEXT_PHASE) {
      const bool REBUILD_FRIC_TEXT =
          !UI_FRIC_TEXT_INITIALIZED || FORCE_TEXT_READD;
      // 绘制发射机构的摩擦轮状态文字
      launcher->referee_->FillCharacter(
          char_fig, "FRC",
          REBUILD_FRIC_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                            : Referee::UIFigureOp::UI_OP_MODIFY,
          UI_LAYER_LAUNCHER,
          FRIC_ENABLED ? Referee::UIColor::UI_COLOR_GREEN
                       : Referee::UIColor::UI_COLOR_ORANGE,
          UI_FONT_SIZE, UI_CHAR_WIDTH, UI_FRIC_TEXT_X, UI_FRIC_TEXT_Y,
          FRIC_ENABLED ? "FRIC ON" : "FRIC OFF");
      if (launcher->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, char_fig) ==
          LibXR::ErrorCode::OK) {
        launcher->mutex_.Lock();
        launcher->ui_.fric_text_initialized = true;
        launcher->mutex_.Unlock();
      }
      return;
    }

    if (UI_PHASE == UI_FIRE_MODE_TEXT_PHASE) {
      const bool REBUILD_FIRE_MODE_TEXT =
          !UI_FIRE_MODE_TEXT_INITIALIZED || FORCE_TEXT_READD;
      // 绘制当前发射模式文字
      launcher->referee_->FillCharacter(
          char_fig, "FRM",
          REBUILD_FIRE_MODE_TEXT ? Referee::UIFigureOp::UI_OP_ADD
                                 : Referee::UIFigureOp::UI_OP_MODIFY,
          UI_LAYER_LAUNCHER,
          GetFireModeColor(SHOT_COUNT, TRIG_MODE, CONTINUE_MODE), UI_FONT_SIZE,
          UI_CHAR_WIDTH, UI_FIRE_MODE_TEXT_X, UI_FIRE_MODE_TEXT_Y,
          GetFireModeText(SHOT_COUNT, TRIG_MODE, CONTINUE_MODE,
                          PRESS_CONTINUE));
      if (launcher->referee_->SendUICharacter(ROBOT_ID, CLIENT_ID, char_fig) ==
          LibXR::ErrorCode::OK) {
        launcher->mutex_.Lock();
        launcher->ui_.fire_mode_text_initialized = true;
        launcher->mutex_.Unlock();
      }
      return;
    }
  }

  static const char* GetFireModeText(uint8_t shot_count, TrigMode trig_mode,
                                     bool continue_mode, bool press_continue) {
    if (trig_mode == TrigMode::CONTINUE || continue_mode || press_continue) {
      return "CONT";
    }
    if (shot_count >= 3) {
      return "BOOST_3";
    }
    return "SING";
  }

  static Referee::UIColor GetFireModeColor(uint8_t shot_count,
                                           TrigMode trig_mode,
                                           bool continue_mode) {
    if (trig_mode == TrigMode::CONTINUE || continue_mode) {
      return Referee::UIColor::UI_COLOR_CYAN;
    }
    if (shot_count >= 3) {
      return Referee::UIColor::UI_COLOR_YELLOW;
    }
    return Referee::UIColor::UI_COLOR_WHITE;
  }

  void TrigControl(float& out_trig, float target_trig_angle, float dt) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle, trig_.angle,
        motors_.trig_fb.omega / PARAM.trig_gear_ratio, dt);
    float omega_limit = static_cast<float>(1.5f * LibXR::TWO_PI * trig_.freq /
                                           PARAM.num_trig_tooth);
    float motor_omega_ref =
        std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(
        motor_omega_ref, motors_.trig_fb.omega / PARAM.trig_gear_ratio, dt);
  }

  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 =
        pid_fric_0_.Calculate(target_rpm, motors_.fric_0_fb.velocity, dt);
    out_fric_1 =
        pid_fric_1_.Calculate(target_rpm, motors_.fric_1_fb.velocity, dt);

    if (state_.event == LauncherEvent::SET_FRICMODE_SAFE) {
      out_fric_0 /= 50.0f;
      out_fric_1 /= 50.0f;
    }
  }
};
