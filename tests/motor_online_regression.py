import pathlib
import re


HEADER = pathlib.Path(__file__).resolve().parents[1] / "InfantryLauncher.hpp"
SOURCE = HEADER.read_text()


def function_body(name):
    match = re.search(rf"\bvoid\s+{name}\s*\(\s*\)\s*\{{", SOURCE)
    if match is None:
        raise AssertionError(f"{name}() not found")

    opening = SOURCE.find("{", match.start())
    depth = 0
    for index in range(opening, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[opening + 1 : index]
    raise AssertionError(f"{name}() body is incomplete")


def compact(text):
    return re.sub(r"\s+", " ", text).strip()


errors = []
update = compact(function_body("Update"))
control = compact(function_body("Control"))

updates = (
    ("motor_fric_0_", "FRIC_0_STATUS"),
    ("motor_fric_1_", "FRIC_1_STATUS"),
    ("motor_trig_", "TRIG_STATUS"),
)
for motor, status in updates:
    if re.search(
        rf"\bconst auto {status} = {motor}->Update\(\);", update
    ) is None:
        errors.append(f"discarded update status: {motor}")

if re.search(r"^[ \t]*motor_\w*_->Update\(\);[ \t]*$", SOURCE, re.MULTILINE):
    errors.append("bare motor Update() statement remains")

online_assignment = (
    "motors_online_ = FRIC_0_STATUS == LibXR::ErrorCode::OK && "
    "FRIC_1_STATUS == LibXR::ErrorCode::OK && "
    "TRIG_STATUS == LibXR::ErrorCode::OK;"
)
if online_assignment not in update:
    errors.append("motors_online_ does not require all three successful updates")

if re.search(r"\bbool motors_online_ = false;", SOURCE) is None:
    errors.append("motors_online_ member is missing or not fail-safe initialized")

guard = re.match(r"if \(!motors_online_\) \{(.*?)\}", control)
if guard is None:
    errors.append("Control() must begin with the offline guard")
else:
    required_guard_body = (
        "out_trig_ = 0.0f; motor_trig_->Relax(); motor_fric_0_->Relax(); "
        "motor_fric_1_->Relax(); return;"
    )
    if compact(guard.group(1)) != required_guard_body:
        errors.append(
            "offline guard must zero trigger output, relax every motor, and return"
        )

if errors:
    raise SystemExit("FAIL:\n- " + "\n- ".join(errors))

print("PASS: InfantryLauncher requires every motor online before control")
