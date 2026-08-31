"""Per-gripper physical constants. Every value that differed between the old
copies (closed_only, single_touch, span, torque thresholds) is a profile
field, never a module constant. Stored inside the cal entry under "profile"."""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields

TMAX_CAP = 2.0   # Nm: documented worm ceiling; >2 Nm transients snapped the v1 worm


@dataclass
class GripperProfile:
    closed_only: bool = False
    single_touch: bool = False
    span_from_closed: float = 30.0     # rad; conservative UNDERestimate of travel
    contact_torque: float = 0.30       # Nm; must exceed the unit's free-run p95
    probe_tmax: float = 0.5            # Nm cap during probing
    probe_vel: float = 0.8             # rad/s slow datum touch
    seek_vel: float = 2.5              # rad/s fast seek
    margin: float = 0.75               # rad inside the physical stops
    backoff: float = 2.0               # rad between touches
    touch_tol: float = 0.2             # rad double-touch agreement
    tmax_nm: float = 2.0               # hard grip-torque cap
    vmax: float = 24.0                 # rad/s software velocity limit
    sw_kp: float = 24.0                # software position-loop gain (1/s)
    stall_torque_frac: float = 0.75    # stall when |torque| > frac * tmax_nm ...
    stall_time_s: float = 0.4          # ... for this long
    park_tolerance_rad: float = 0.35   # PROVISIONAL: boot wrapped vs last_wrapped (spec §10)
    watchdog_ms: int = 8000            # RID 9 raw value; unit under investigation (2026-08-30)
    close_dir: int = 1                 # velocity sign that closes the jaws

    def validate(self) -> None:
        if self.tmax_nm > TMAX_CAP:
            raise ValueError(f"profile tmax_nm {self.tmax_nm} exceeds the {TMAX_CAP} Nm cap")
        if self.close_dir not in (1, -1):
            raise ValueError("close_dir must be +1 or -1")
        if self.span_from_closed <= 0 or self.vmax <= 0 or self.sw_kp <= 0:
            raise ValueError("span_from_closed, vmax, sw_kp must be positive")

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "GripperProfile":
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})


PRESETS = {
    "openarm": GripperProfile(),
    "yam": GripperProfile(span_from_closed=33.5),
    "ur": GripperProfile(closed_only=True, single_touch=True, span_from_closed=30.0),
}
