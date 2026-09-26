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

    status_updates = (
        "motors_.fric_0_status = motors_.fric_0->Update();",
        "motors_.fric_1_status = motors_.fric_1->Update();",
        "motors_.trig_status = motors_.trig->Update();",
    )
    for status_update in status_updates:
        require("persistent per-motor ErrorCode", status_update, update)

    require(
        "trigger output re-derived every cycle",
        "float out_trig = 0.0f;",
        control,
    )
    require(
        "trigger activation derived from mode, freshness and state",
        "const bool TRIG_ACTIVE = !relax && TRIG_ONLINE && "
        "state_.mode != TrigMode::RELAX;",
        control,
    )
    require(
        "trigger law runs only when active",
        "if (TRIG_ACTIVE) { TrigControl(out_trig, trig_.target_angle, dt_); }",
        control,
    )
    require(
        "inactive trigger stays relaxed",
        "if (!TRIG_ACTIVE) { motors_.trig->Relax(); }",
        control,
    )
    require(
        "offline friction wheels stay relaxed",
        "if (!FRIC_0_ONLINE || !FRIC_1_ONLINE) "
        "{ motors_.fric_0->Relax(); motors_.fric_1->Relax(); }",
        control,
    )
    require(
        "state machine runs unconditionally",
        "self->RunStateMachine();",
        thread,
    )
    require(
        "SetMode keeps mode handling",
        "auto event = static_cast<LauncherEvent>(mode);",
        set_mode,
    )

    for banned in (
        "motor_fault_latched_",
        "motors_online_",
        "ForceMotorFaultSafeState",
        "out_trig_",
    ):
        if banned in source:
            raise ContractError(f"stale-output symbol reappeared: {banned}")


source = pathlib.Path(sys.argv[1]).read_text()
validate(source)

mutations = (
    (
        "per-motor trigger relax dropped",
        "    if (!TRIG_ACTIVE) {\n      motors_.trig->Relax();\n    }",
        "    motors_.trig->Relax();",
    ),
    (
        "trigger activation ignores mode and freshness",
        "    const bool TRIG_ACTIVE =\n        !relax && TRIG_ONLINE && state_.mode != TrigMode::RELAX;",
        "    const bool TRIG_ACTIVE = !relax && TRIG_ONLINE;",
    ),
    (
        "trigger law runs while inactive",
        "      if (TRIG_ACTIVE) {\n        TrigControl(out_trig, trig_.target_angle, dt_);\n      }",
        "      TrigControl(out_trig, trig_.target_angle, dt_);",
    ),
    (
        "stale member output reintroduced",
        "    float out_trig = 0.0f;",
        "    out_trig_ = 0.0f;",
    ),
    (
        "offline friction wheels commanded",
        "    if (!FRIC_0_ONLINE || !FRIC_1_ONLINE) {",
        "    if (false) {",
    ),
    (
        "all-motor latch reintroduced",
        "    motors_.trig_status = motors_.trig->Update();",
        "    motors_.trig_status = motors_.trig->Update();\n\n"
        "    if (!motors_online_ && !motor_fault_latched_) {\n"
        "      motor_fault_latched_ = true;\n    }",
    ),
    (
        "state machine suppressed while a motor is offline",
        "      self->RunStateMachine();",
        "      if (!self->motors_online_) {\n        self->RunStateMachine();\n      }",
    ),
    (
        "SetMode offline rejection reintroduced",
        "  void SetMode(uint32_t mode) {\n    auto event",
        "  void SetMode(uint32_t mode) {\n    if (!motors_online_) {\n      return;\n    }\n\n    auto event",
    ),
    (
        "trigger diagnostic status discarded",
        "motors_.trig_status = motors_.trig->Update();",
        "motors_.trig->Update();",
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

print("PASS: InfantryLauncher per-cycle output derivation contracts and mutations")
PY
