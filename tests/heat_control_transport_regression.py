import pathlib


HEADER = pathlib.Path(__file__).resolve().parents[1] / "InfantryLauncher.hpp"
SOURCE = HEADER.read_text(encoding="utf-8")


required = (
    "bool heat_control_enabled",
    "heat_control_enabled_(heat_control_enabled)",
    "Pldx::NavLink::SentryInfoOnline",
    "Pldx::NavLink::ONLINE_INFO_TOPIC",
    "launcher::param::ONLINE_INFO_TIMEOUT_MS",
    "online.heat_limit",
    "online.cooling_value",
    "online.current_heat",
    "online.bullets_remaining",
    "void UpdateOnlineInfoFreshness",
    "if (!heat_control_enabled_)",
    "heat_limit_.allow_fire = true;",
    "trig_freq_ = expect_trig_freq_;",
    "if (heat_control_enabled_ && !heat_decision.allow_fire)",
    "if (calibrated_ &&",
)

missing = [item for item in required if item not in SOURCE]
if missing:
    raise SystemExit("FAIL: missing heat-control contracts: " + ", ".join(missing))

for forbidden in (
    'ASyncSubscriber<Referee::LauncherPack>',
    '"launcher_ref"',
    "ref_pack.bullet_remain",
    "UpdateLauncherRefFreshness",
    "RefereeStateV1",
    "REFEREE_STATE_TOPIC",
    "HeaderCompatible",
    "SHOOT_MODE_PRIMARY",
):
    if forbidden in SOURCE:
        raise SystemExit(f"FAIL: obsolete local launcher referee input remains: {forbidden}")

print("PASS: launcher heat control consumes online_info and is configurable")
