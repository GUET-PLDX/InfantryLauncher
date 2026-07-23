#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HEADER="${SCRIPT_DIR}/../InfantryLauncher.hpp"

python3 - "${HEADER}" <<'PY'
import pathlib
import re
import sys


class ContractError(Exception):
    pass


def compact(text):
    return re.sub(r"\s+", " ", text).strip()


def function_body(source, name):
    match = re.search(rf"\bvoid\s+{name}\s*\([^)]*\)\s*\{{", source)
    if match is None:
        raise ContractError(f"{name}() not found")

    opening = source.find("{", match.start())
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return compact(source[opening + 1 : index])
    raise ContractError(f"{name}() body is incomplete")


def require(description, expected, body):
    if expected not in body:
        raise ContractError(f"missing: {description}")


def validate(source):
    update = function_body(source, "Update")
    thread = function_body(source, "ThreadFunc")
    control = function_body(source, "Control")
    set_mode = function_body(source, "SetMode")
    safe_state = function_body(source, "ForceMotorFaultSafeState")

    status_updates = (
        "motor_fric_0_status_ = motor_fric_0_->Update();",
        "motor_fric_1_status_ = motor_fric_1_->Update();",
        "motor_trig_status_ = motor_trig_->Update();",
    )
    for status_update in status_updates:
        require("persistent per-motor ErrorCode", status_update, update)

    require(
        "all-motor freshness conjunction",
        "motors_online_ = motor_fric_0_status_ == LibXR::ErrorCode::OK && "
        "motor_fric_1_status_ == LibXR::ErrorCode::OK && "
        "motor_trig_status_ == LibXR::ErrorCode::OK;",
        update,
    )
    require(
        "fault transition latch",
        "if (!motors_online_ && !motor_fault_latched_) { "
        "motor_fault_latched_ = true;",
        update,
    )
    require(
        "latched safe-state enforcement",
        "if (motor_fault_latched_) { ForceMotorFaultSafeState(); }",
        update,
    )
    if "XR_LOG_" in update:
        raise ContractError("periodic Update() diagnostics are forbidden")

    require(
        "state-machine suppression while fault is latched",
        "if (!self->motor_fault_latched_) { self->RunStateMachine(); }",
        thread,
    )

    guard = re.match(
        r"if \(!motors_online_ \|\| motor_fault_latched_\) \{(.*?)\}",
        control,
    )
    if guard is None:
        raise ContractError("Control() must begin with the online and latch guard")
    expected_guard = (
        "out_trig_ = 0.0f; motor_trig_->Relax(); motor_fric_0_->Relax(); "
        "motor_fric_1_->Relax(); return;"
    )
    if compact(guard.group(1)) != expected_guard:
        raise ContractError("fault guard must relax all motors and return")

    require(
        "offline SetMode rejection",
        "if (motor_fault_latched_) { if (!motors_online_) { return; } "
        "motor_fault_latched_ = false; }",
        set_mode,
    )
    if set_mode.index("motor_fault_latched_ = false;") > set_mode.index(
        "auto event = static_cast<LauncherEvent>(mode);"
    ):
        raise ContractError("fresh SetMode must clear the latch before mode handling")

    for safe_write in (
        "launcher_cmd_.isfire = false;",
        "launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;",
        "launcher_state_ = LauncherState::RELAX;",
        "trig_mode_ = TrigMode::RELAX;",
        "out_trig_ = 0.0f;",
        "target_rpm_ = 0.0f;",
    ):
        require("fault safe-state write", safe_write, safe_state)

    for status in (
        "motor_fric_0_status_",
        "motor_fric_1_status_",
        "motor_trig_status_",
    ):
        require(
            "diagnostic ErrorCode member",
            f"LibXR::ErrorCode {status} = LibXR::ErrorCode::FAILED;",
            compact(source),
        )
    require(
        "fault latch member",
        "bool motor_fault_latched_ = false;",
        compact(source),
    )


source = pathlib.Path(sys.argv[1]).read_text()
validate(source)

mutations = (
    (
        "automatic online latch clear",
        "if (motor_fault_latched_) {\n      ForceMotorFaultSafeState();\n    }",
        "if (motors_online_) {\n      motor_fault_latched_ = false;\n    }",
    ),
    (
        "offline SetMode latch clear",
        "if (!motors_online_) {\n        return;\n      }",
        "if (motors_online_) {\n        return;\n      }",
    ),
    (
        "fresh SetMode does not clear latch",
        "motor_fault_latched_ = false;\n    }\n\n    auto event",
        "motor_fault_latched_ = true;\n    }\n\n    auto event",
    ),
    (
        "state machine runs while faulted",
        "if (!self->motor_fault_latched_) {\n        self->RunStateMachine();\n      }",
        "self->RunStateMachine();",
    ),
    (
        "Control ignores latched fault",
        "if (!motors_online_ || motor_fault_latched_)",
        "if (!motors_online_)",
    ),
    (
        "trigger diagnostic status discarded",
        "motor_trig_status_ = motor_trig_->Update();",
        "motor_trig_->Update();",
    ),
)

for description, old, new in mutations:
    if source.count(old) != 1:
        raise ContractError(f"mutation fixture mismatch: {description}")
    try:
        validate(source.replace(old, new, 1))
    except ContractError:
        continue
    raise ContractError(f"mutation survived: {description}")

print("PASS: InfantryLauncher motor freshness latch contracts and mutations")
PY
