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
  - burst_duration: 2.0
  - heat_margin: 10.0
  - cmd: '@&cmd'
  - referee: '@nullptr'
  - thread_priority: LibXR::Thread::Priority::HIGH
required_hardware:
  - dr16
  - can
depends:
  - pldx/CMD
  - pldx/RMMotor
  - pldx/NavLinkProtocol
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "NavLinkProtocol.hpp"
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
constexpr float PERIODS_PER_SECOND = 10.0f;

struct HeatFeedforwardConfig {
  float single_heat = 10.0f;
  float max_frequency = 15.0f;
  float burst_duration = 2.0f;
  float heat_margin = 10.0f;
};

struct HeatFeedforwardObservation {
  float heat_limit = 0.0f;
  float current_heat = 0.0f;
  float cooling_rate = 0.0f;
  bool data_valid = false;
};

struct HeatFeedforwardResult {
  bool allow_fire = false;
  float target_frequency = 0.0f;
};

// PDF model: each 100 ms period settles shots first, then cooling.
class HeatFeedforward {
 public:
  static float AdvanceBudget(const HeatFeedforwardConfig& config,
                             float current_heat, float cooling_rate,
                             unsigned int launched_shots,
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

  static HeatFeedforwardResult Calculate(
      const HeatFeedforwardConfig& config,
      const HeatFeedforwardObservation& observation,
      unsigned int shot_count = 1U) {
    const float d = config.single_heat;
    const float n =
        std::ceil(config.burst_duration / HEAT_SETTLEMENT_PERIOD_SEC);
    if (!observation.data_valid || !std::isfinite(d) || d <= 0.0f ||
        !std::isfinite(n) || n <= 0.0f ||
        !std::isfinite(config.max_frequency) || config.max_frequency <= 0.0f ||
        !std::isfinite(config.heat_margin) || config.heat_margin < 0.0f ||
        !std::isfinite(observation.heat_limit) ||
        !std::isfinite(observation.current_heat) ||
        !std::isfinite(observation.cooling_rate) ||
        observation.heat_limit <= 0.0f || observation.cooling_rate < 0.0f ||
        shot_count == 0U) {
      return {};
    }
    const float remaining =
        observation.heat_limit - observation.current_heat - config.heat_margin;
    if (remaining < d * static_cast<float>(shot_count)) {
      return {};
    }
    const float sustainable = observation.cooling_rate / d;
    const float burst =
        (PERIODS_PER_SECOND * remaining - observation.cooling_rate) / (d * n) +
        sustainable;
    const float lower_frequency = std::min(sustainable, config.max_frequency);
    return {true, std::clamp(burst, lower_frequency, config.max_frequency)};
  }
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
      float single_heat, float max_frequency, float burst_duration,
      float heat_margin, CMD* cmd, Referee* referee = nullptr,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : motor_fric_0_(motor_fric_0),
        motor_fric_1_(motor_fric_1),
        motor_trig_(motor_trig),
        feedback_topic_(
            LibXR::Topic::CreateTopic<Pldx::NavLink::GimbalFeedbackV1>(
                Pldx::NavLink::LAUNCHER_FEEDBACK_TOPIC, nullptr, true)),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_speed_0),
        pid_fric_1_(pid_param_fric_speed_1),
        param_(launcher_param),
        heat_control_enabled_(heat_control_enabled),
        heat_config_{single_heat, max_frequency, burst_duration, heat_margin},
        referee_(referee) {
    expect_trig_freq_ = max_frequency;
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
    LibXR::Topic::ASyncSubscriber<Pldx::NavLink::SentryInfoOnline>
        online_info_sub(Pldx::NavLink::ONLINE_INFO_TOPIC);
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
        self->online_info_valid_ = true;
        self->last_online_info_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();
        self->heat_data_valid_ = online.heat_limit > 0U;
        if (self->heat_data_valid_) {
          self->ref_data_.heat_limit = static_cast<float>(online.heat_limit);
          self->ref_data_.cooling_rate =
              static_cast<float>(online.cooling_value);
          self->ref_data_.current_heat_17 =
              static_cast<float>(online.current_heat);
        }
        self->bullet_count_ = online.bullets_remaining;
        self->bullet_count_valid_ = true;
        online_info_sub.StartWaiting();
      }
      self->mutex_.Lock();
      self->Update();
      if (!self->motor_fault_latched_) {
        self->RunStateMachine();
      }
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
    motor_fric_0_status_ = motor_fric_0_->Update();
    motor_fric_1_status_ = motor_fric_1_->Update();
    motor_trig_status_ = motor_trig_->Update();
    motors_online_ = motor_fric_0_status_ == LibXR::ErrorCode::OK &&
                     motor_fric_1_status_ == LibXR::ErrorCode::OK &&
                     motor_trig_status_ == LibXR::ErrorCode::OK;

    if (!motors_online_ && !motor_fault_latched_) {
      motor_fault_latched_ = true;
      pid_fric_0_.Reset();
      pid_fric_1_.Reset();
      pid_trig_angle_.Reset();
      pid_trig_sp_.Reset();
    }
    if (motor_fault_latched_) {
      ForceMotorFaultSafeState();
    }

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    float current_motor_angle = param_trig_.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle_);
    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle_ = current_motor_angle;

    UpdateLauncherState();
  }

  void Control() {
    if (!motors_online_ || motor_fault_latched_) {
      out_trig_ = 0.0f;
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    bool relax = false;

    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      relax = true;
    } else {
      if (trig_mode_ != TrigMode::RELAX) {
        TrigControl(out_trig_, target_trig_angle_, dt_);
      }
      FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
      trig_fb = param_trig_;
      fric_0_fb = param_fric_0_;
      fric_1_fb = param_fric_1_;
    }

    if (relax) {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    auto cmd_trig = Motor::MotorCmd{
        .mode = Motor::ControlMode::MODE_CURRENT,
        .reduction_ratio = param_.trig_gear_ratio,
        .velocity = out_trig_,
    };
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_1};

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

    motor_control(motor_trig_, trig_fb, cmd_trig);
    motor_control(motor_fric_0_, fric_0_fb, cmd_fric_0);
    motor_control(motor_fric_1_, fric_1_fb, cmd_fric_1);
  }

  void SetMode(uint32_t mode) {
    if (motor_fault_latched_) {
      if (!motors_online_) {
        return;
      }
      motor_fault_latched_ = false;
    }

    auto event = static_cast<LauncherEvent>(mode);
    switch (event) {
      case LauncherEvent::SET_SHOTMODE_SINGLE:
        shot_count_ = 1;
        continue_mode_ = false;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
        shot_count_ = 1;
        continue_mode_ = true;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        shot_count_ = 3;
        continue_mode_ = false;
        ui_fire_mode_text_initialized_ = false;
        ui_refresh_tick_ = UI_FIRE_MODE_TEXT_PHASE;
        return;
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
      case LauncherEvent::SET_FRICMODE_READY:
        launcher_event_ = event;
        ui_fric_text_initialized_ = false;
        ui_refresh_tick_ = UI_FRIC_TEXT_PHASE;
        if (event != LauncherEvent::SET_FRICMODE_READY) {
          calibrated_ = false;
          calibration_pending_ = false;
          target_shot_index_ = 0;
          is_reverse_ = false;
        }
        break;
    }

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  void LostCtrl() {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    press_continue_ = false;
    calibrated_ = false;
    calibration_pending_ = false;
    trigger_step_active_ = false;
    target_shot_index_ = 0;
    is_reverse_ = false;
    shot_progress_ = 0.0f;
    launcher_cmd_.isfire = false;
    ui_fric_text_initialized_ = false;
    ui_fire_mode_text_initialized_ = false;
    ui_shot_position_initialized_ = false;
    ui_refresh_tick_ = UI_FRIC_TEXT_PHASE;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

  LibXR::Event& GetEvent() { return launcher_event; }

  void OnMonitor() {}

  CMD::LauncherCMD launcher_cmd_{};  // NOLINT
  RefereeData ref_data_;

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

  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;
  LibXR::Topic feedback_topic_;
  float last_trig_angle_ = 0.0f;
  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};
  LibXR::ErrorCode motor_fric_0_status_ = LibXR::ErrorCode::FAILED;
  LibXR::ErrorCode motor_fric_1_status_ = LibXR::ErrorCode::FAILED;
  LibXR::ErrorCode motor_trig_status_ = LibXR::ErrorCode::FAILED;

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;
  bool heat_control_enabled_ = true;
  launcher::HeatFeedforwardConfig heat_config_{};
  Referee* referee_ = nullptr;
  LibXR::Event launcher_event;
  LibXR::Thread thread_;
  LibXR::Timer::TimerHandle timer_ui_{};

  float out_trig_ = 0.0f;

  float expect_trig_freq_ = 15.0f;
  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float expect_rpm_ = param_.fric1_setpoint_speed;
  float last_bullet_speed_ = -1.0f;
  float trig_freq_ = 0.0f;
  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;
  float first_shot_angle_ = 0.0f;
  float fric_speed_peak_ = 0.0f;
  float jam_target_angle_ = 0.0f;
  int32_t target_shot_index_ = 0;
  uint8_t shot_count_ = 1;

  bool last_fire_notify_ = false;
  bool continue_mode_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool heat_initialized_ = false;
  bool trigger_step_active_ = false;
  bool calibrated_ = false;
  bool calibration_pending_ = false;
  bool online_info_valid_ = false;
  bool heat_data_valid_ = false;
  bool ui_layer_cleared_ = false;
  bool ui_fric_text_initialized_ = false;
  bool ui_fire_mode_text_initialized_ = false;
  bool ui_shot_position_initialized_ = false;
  bool motors_online_ = false;
  bool motor_fault_latched_ = false;
  uint32_t ui_refresh_tick_ = 0;
  uint32_t last_feedback_publish_ms_ = 0U;
  uint16_t bullet_count_ = 0U;
  bool bullet_count_valid_ = false;

  float shot_progress_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_check_time_ = 0;
  LibXR::MillisecondTimestamp last_online_info_rx_time_ms_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  HeatLimit heat_limit_{
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .allow_fire = true,
  };
  LibXR::Mutex mutex_;

  void PublishFeedback() {
    LibXR::Mutex::LockGuard lock(mutex_);
    const auto now = LibXR::Timebase::GetMilliseconds();
    Pldx::NavLink::GimbalFeedbackV1 feedback{};
    feedback.bullet_speed_mps = ref_data_.bullet_speed;
    feedback.bullet_count = bullet_count_;
    if (IsOnlineInfoFresh(now) && std::isfinite(feedback.bullet_speed_mps) &&
        feedback.bullet_speed_mps > 0.0F) {
      feedback.valid_flags |= Pldx::NavLink::GIMBAL_BULLET_SPEED_VALID;
    }
    if (bullet_count_valid_ && IsOnlineInfoFresh(now)) {
      feedback.valid_flags |= Pldx::NavLink::GIMBAL_BULLET_COUNT_VALID;
    }
    feedback_topic_.Publish(feedback);
  }

  void ForceMotorFaultSafeState() {
    launcher_cmd_.isfire = false;
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;
    out_trig_ = 0.0f;
    target_rpm_ = 0.0f;
    target_trig_angle_ = trig_angle_;
    press_continue_ = false;
    trigger_step_active_ = false;
  }

  void UpdateLauncherState() {
    if (param_trig_.torque > launcher::param::JAM_TORQUE) {
      launcher_state_ = LauncherState::JAMMED;
      return;
    }
    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcher_state_ = LauncherState::RELAX;
      return;
    }

    if (!heat_limit_.allow_fire) {
      launcher_state_ = LauncherState::STOP;
      return;
    }

    launcher_state_ =
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

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  bool IsOnlineInfoFresh(LibXR::MillisecondTimestamp now) const {
    return online_info_valid_ &&
           (now - last_online_info_rx_time_ms_).ToMillisecond() <=
               launcher::param::ONLINE_INFO_TIMEOUT_MS;
  }

  void UpdateOnlineInfoFreshness(LibXR::MillisecondTimestamp now) {
    if (!heat_control_enabled_) {
      heat_limit_.allow_fire = true;
      trig_freq_ = expect_trig_freq_;
      return;
    }
    if (IsOnlineInfoFresh(now) && heat_data_valid_) {
      return;
    }

    online_info_valid_ = false;
    heat_data_valid_ = false;
    ref_data_ = RefereeData{};
    heat_limit_.allow_fire = false;
    trig_freq_ = 0.0f;
    target_rpm_ = 0.0f;
    out_trig_ = 0.0f;
    launcher_cmd_.isfire = false;
    press_continue_ = false;
    trigger_step_active_ = false;
    calibration_pending_ = false;
    is_reverse_ = false;
    shot_progress_ = 0.0f;
    target_trig_angle_ = trig_angle_;

    // 裁判热量或弹速数据失效时，强制回到已有安全摩擦轮模式。
    if (launcher_event_ == LauncherEvent::SET_FRICMODE_READY) {
      launcher_event_ = LauncherEvent::SET_FRICMODE_SAFE;
      ui_fric_text_initialized_ = false;
      ui_refresh_tick_ = UI_FRIC_TEXT_PHASE;
      pid_fric_0_.Reset();
      pid_fric_1_.Reset();
      pid_trig_angle_.Reset();
      pid_trig_sp_.Reset();
    }
  }

  void UpdateTriggerMode(LibXR::MillisecondTimestamp now) {
    switch (launcher_state_) {
      case LauncherState::RELAX:
        trig_mode_ = TrigMode::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (continue_mode_) {
          press_continue_ = true;
          trig_mode_ = TrigMode::CONTINUE;
        } else if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TrigMode::SINGLE;
        } else {
          if (!press_continue_ &&
              (now - fire_press_time_).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TrigMode::JAM;
        break;
    }
  }

  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    const float step = launcher::param::TRIG_STEP;
    const float ready_rpm =
        expect_rpm_ - launcher::param::FRIC_READY_RPM_MARGIN;
    const float fric_speed =
        (fabsf(param_fric_0_.velocity) + fabsf(param_fric_1_.velocity)) * 0.5f;
    const bool fric_ready = fabsf(param_fric_0_.velocity) >= ready_rpm &&
                            fabsf(param_fric_1_.velocity) >= ready_rpm;

    auto indexed_target = [&]() {
      return first_shot_angle_ + step * static_cast<float>(target_shot_index_);
    };

    auto next_indexed_target = [&]() {
      target_shot_index_ =
          static_cast<int32_t>(ceilf((trig_angle_ - first_shot_angle_) / step));
      return indexed_target();
    };

    auto recover_from_jam = [&]() {
      target_trig_angle_ =
          calibrated_ ? next_indexed_target() : jam_target_angle_;
      is_reverse_ = false;
      trigger_step_active_ = true;
      last_trig_time_ = now;
    };

    if (trigger_step_active_) {
      fric_speed_peak_ = std::max(fric_speed_peak_, fric_speed);

      if (calibration_pending_ && !calibrated_ &&
          fric_speed_peak_ >= ready_rpm &&
          fric_speed_peak_ - fric_speed >= launcher::param::FRIC_DROP_RPM) {
        calibrated_ = true;
        calibration_pending_ = false;
        first_shot_angle_ = trig_angle_;
        target_trig_angle_ = indexed_target();
        heat_limit_.current_heat =
            std::max(heat_limit_.current_heat, ref_data_.current_heat_17) +
            heat_config_.single_heat;
        shot_progress_ = 0.0f;
        last_trig_angle_ = trig_angle_;
      }

      float angle_error = fabsf(target_trig_angle_ - trig_angle_);
      if (angle_error <= launcher::param::TRIGGER_SETTLE_ANGLE) {
        trigger_step_active_ = false;
        if (calibration_pending_ && !calibrated_) {
          calibration_pending_ = false;
        }
      }
    } else {
      fric_speed_peak_ = fric_speed;
    }

    auto start_shot = [&]() {
      if (!fric_ready) {
        return;
      }

      const float current_heat =
          std::max(heat_limit_.current_heat, ref_data_.current_heat_17);
      const auto heat_decision = launcher::HeatFeedforward::Calculate(
          heat_config_,
          {ref_data_.heat_limit, current_heat, ref_data_.cooling_rate,
           heat_data_valid_ && IsOnlineInfoFresh(now)},
          shot_count_);
      if (heat_control_enabled_ && !heat_decision.allow_fire) {
        return;
      }

      if (calibrated_) {
        target_shot_index_ += static_cast<int32_t>(shot_count_);
        target_trig_angle_ = indexed_target();
      } else {
        calibration_pending_ = true;
        target_shot_index_ = static_cast<int32_t>(shot_count_) - 1;
        fric_speed_peak_ = fric_speed;
        target_trig_angle_ =
            trig_angle_ + step * static_cast<float>(shot_count_);
      }

      trigger_step_active_ = true;
      last_trig_time_ = now;
    };

    switch (trig_mode_) {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        target_trig_angle_ = trig_angle_;
        trigger_step_active_ = false;
        calibration_pending_ = false;
        is_reverse_ = false;
        break;

      case TrigMode::SINGLE:
        if (last_trig_mode_ == TrigMode::JAM) {
          recover_from_jam();
        } else if (last_trig_mode_ == TrigMode::SAFE ||
                   last_trig_mode_ == TrigMode::RELAX) {
          start_shot();
        }
        break;

      case TrigMode::CONTINUE: {
        float trig_freq = std::max(trig_freq_, 1e-3f);
        float interval_s = 1.0f / trig_freq;
        float since_last = (now - last_trig_time_).ToSecondf();
        if (last_trig_mode_ == TrigMode::JAM) {
          recover_from_jam();
        } else if (!trigger_step_active_ && since_last >= interval_s) {
          start_shot();
        }
      } break;

      case TrigMode::JAM: {
        trigger_step_active_ = false;
        if (last_trig_mode_ != TrigMode::JAM) {
          jam_target_angle_ =
              calibrated_ ? indexed_target() : target_trig_angle_;
          is_reverse_ = false;
        }
        if (last_trig_mode_ != TrigMode::JAM ||
            (now - last_jam_time_).ToSecondf() >=
                launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          target_trig_angle_ =
              is_reverse_ ? jam_target_angle_ : trig_angle_ - 0.3f * step;
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;
    }

    last_trig_mode_ = trig_mode_;
  }

  void SetFricTargetByEvent() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        target_rpm_ = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY: {
        if (heat_control_enabled_ &&
            (!IsOnlineInfoFresh(LibXR::Timebase::GetMilliseconds()) ||
             !heat_data_valid_)) {
          target_rpm_ = 0.0f;
          break;
        }

        // 根据裁判系统回传弹速微调摩擦轮期望转速
        float bullet_speed = ref_data_.bullet_speed;
        if (bullet_speed <= 0.0f || bullet_speed > 30.0f) {
          bullet_speed =
              param_.target_bullet_speed - 2.0f * param_.bullet_speed_tolerance;
        }

        if (last_bullet_speed_ != bullet_speed) {
          if (bullet_speed >
              param_.target_bullet_speed - param_.bullet_speed_tolerance) {
            expect_rpm_ -= 70.0f;
          }

          if (bullet_speed < param_.target_bullet_speed -
                                 2.2f * param_.bullet_speed_tolerance) {
            expect_rpm_ += 50.0f;
          }
          last_bullet_speed_ = bullet_speed;
        }

        target_rpm_ = expect_rpm_;
        break;
      }
      case LauncherEvent::SET_SHOTMODE_SINGLE:
      case LauncherEvent::SET_SHOTMODE_CONTINUE:
      case LauncherEvent::SET_SHOTMODE_BOOST_3:
        break;
    }
  }

  void UpdateHeatControl(LibXR::MillisecondTimestamp now) {
    if (!heat_control_enabled_) {
      heat_limit_.allow_fire = true;
      trig_freq_ = expect_trig_freq_;
      return;
    }
    const float current_heat =
        std::max(heat_limit_.current_heat, ref_data_.current_heat_17);
    const auto decision = launcher::HeatFeedforward::Calculate(
        heat_config_,
        {ref_data_.heat_limit, current_heat, ref_data_.cooling_rate,
         heat_data_valid_ && IsOnlineInfoFresh(now)});
    heat_limit_.allow_fire = decision.allow_fire;
    trig_freq_ = decision.target_frequency;
  }

  void CurrentHeat(LibXR::MillisecondTimestamp now) {
    if (!heat_initialized_) {
      heat_initialized_ = true;
      last_check_time_ = now;
      last_trig_angle_ = trig_angle_;
      return;
    }

    heat_limit_.current_heat =
        std::max(heat_limit_.current_heat, ref_data_.current_heat_17);
    heat_limit_.launched_num = 0.0f;

    float delta_teeth =
        (trig_angle_ - last_trig_angle_) / launcher::param::TRIG_STEP;
    last_trig_angle_ = trig_angle_;

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_READY) {
      shot_progress_ += delta_teeth;
      if (shot_progress_ < 0.0f) {
        shot_progress_ = 0.0f;
      }
    } else {
      shot_progress_ = 0.0f;
    }

    if (calibrated_ &&
        shot_progress_ >= 1.0f - launcher::param::SHOT_PROGRESS_EPSILON) {
      heat_limit_.launched_num = floorf(shot_progress_);
      shot_progress_ -= heat_limit_.launched_num;
    }

    const auto elapsed_ms = (now - last_check_time_).ToMillisecond();
    const uint32_t periods =
        elapsed_ms / launcher::param::HEAT_SETTLEMENT_PERIOD_MS;
    heat_limit_.current_heat = launcher::HeatFeedforward::AdvanceBudget(
        heat_config_, heat_limit_.current_heat, ref_data_.cooling_rate,
        static_cast<uint32_t>(heat_limit_.launched_num), periods);
    if (periods > 0U) {
      last_check_time_ = LibXR::MillisecondTimestamp(
          static_cast<uint32_t>(last_check_time_) +
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
    const uint32_t UI_TICK = launcher->ui_refresh_tick_++;
    const uint32_t UI_PHASE = UI_TICK % UI_REFRESH_PHASE_COUNT;
    const bool FORCE_TEXT_READD =
        (UI_TICK % UI_TEXT_READD_DIV) < UI_REFRESH_PHASE_COUNT;
    const bool FRIC_ENABLED =
        launcher->launcher_event_ == LauncherEvent::SET_FRICMODE_READY;
    const uint8_t SHOT_COUNT = launcher->shot_count_;
    const TrigMode TRIG_MODE = launcher->trig_mode_;
    const bool CONTINUE_MODE = launcher->continue_mode_;
    const bool PRESS_CONTINUE = launcher->press_continue_;
    const bool UI_LAYER_CLEARED = launcher->ui_layer_cleared_;
    const bool UI_FRIC_TEXT_INITIALIZED = launcher->ui_fric_text_initialized_;
    const bool UI_FIRE_MODE_TEXT_INITIALIZED =
        launcher->ui_fire_mode_text_initialized_;
    const bool UI_SHOT_POSITION_INITIALIZED =
        launcher->ui_shot_position_initialized_;
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
      launcher->ui_layer_cleared_ = true;
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
          launcher->ui_shot_position_initialized_ = true;
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
        launcher->ui_fric_text_initialized_ = true;
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
        launcher->ui_fire_mode_text_initialized_ = true;
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
        target_trig_angle, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt);
    float omega_limit = static_cast<float>(1.5f * LibXR::TWO_PI * trig_freq_ /
                                           param_.num_trig_tooth);
    float motor_omega_ref =
        std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt);
  }

  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      out_fric_0 /= 50.0f;
      out_fric_1 /= 50.0f;
    }
  }
};
