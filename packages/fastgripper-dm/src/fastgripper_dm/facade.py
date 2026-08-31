"""FastGripper: the one object a user drives. Owns a MotorPort, a
GripperController, and the cal entry's park lifecycle."""

from __future__ import annotations

import time

from .calstore import default_cal_path, entry_profile, get_entry, load_store, save_store
from .controller import GripperController
from .port import MitCommand, MotorPort, PortError, SPAN
from .tracker import MultiTurnTracker


class HomingError(RuntimeError):
    pass


def _wrapped_dist(a: float, b: float) -> float:
    d = abs(a - b) % SPAN
    return min(d, SPAN - d)


class FastGripper:
    def __init__(self, port: MotorPort | None = None, interface: str | None = None,
                 channel: str | None = None, gripper: str | None = None,
                 cal_path: str | None = None, home: str = "auto", auto_fallback: str = "stall"):
        self._given_port = port
        self._interface, self._channel = interface, channel
        self._gripper_name = gripper
        self._cal_path = cal_path or default_cal_path()
        self._home_mode = home
        self._auto_fallback = auto_fallback
        self.port: MotorPort | None = port
        self.ctrl: GripperController | None = None
        self._store = None
        self._entry = None
        self._connected = False

    @classmethod
    def standalone(cls, interface=None, channel=None, gripper=None, cal_path=None, home="auto"):
        from .cli import _load_config

        cfg = _load_config()
        return cls(interface=interface or cfg.get("interface", "auto"),
                   channel=channel or cfg.get("channel"),
                   gripper=gripper, cal_path=cal_path, home=home)

    # --- lifecycle ---
    def connect(self) -> None:
        self._store = load_store(self._cal_path)
        name, self._entry = get_entry(self._store, self._gripper_name)
        self._gripper_name = name
        profile = entry_profile(self._entry)
        self.ctrl = GripperController(self._entry, profile)
        if self.port is None:
            from .cli import _load_config
            from .damiao.canbus import open_bus

            cfg = _load_config()
            bus = open_bus(self._interface, self._channel)
            from .damiao.can_port import DamiaoCanPort

            self.port = DamiaoCanPort(bus, int(self._entry.get("motor_id", cfg.get("motor_id", 0x01))),
                                      int(self._entry.get("master_id", cfg.get("master_id", 0x00))))
        self.port.enable()
        try:
            boot = self.port.read()
            mode = self._home_mode
            if mode == "auto":
                lw = self._entry.get("last_wrapped")
                if lw is not None and _wrapped_dist(boot.position, lw) <= profile.park_tolerance_rad:
                    self.ctrl.adopt_park(self._entry["last_position"])
                    self.ctrl.tick(boot, 0.0)
                elif self._auto_fallback == "stall":
                    self.home_against_stop()
                else:
                    raise HomingError(
                        f"gripper is not where it was parked (wrapped {boot.position:+.2f} vs "
                        f"saved {lw}) -- run home_against_stop() / `autocal home`, or connect "
                        f"with home='assume_closed' after closing the jaws by hand")
            elif mode == "assume_closed":
                self.ctrl.tick(boot, 0.0)
                self.ctrl.anchor(self._entry["closed"])
            elif mode == "stall":
                self.home_against_stop()
            elif mode == "off":
                self.ctrl.tick(boot, 0.0)
            else:
                raise ValueError(f"unknown home mode '{mode}'")
            self.ctrl.hold()
        except Exception:
            # No failure path may leave the motor enabled (spec §4): once the
            # port is enabled, any exception during boot/homing/restore must
            # disable and close it before propagating.
            self._safe_disable_close()
            raise
        self._connected = True

    def _safe_disable_close(self) -> None:
        try:
            self.port.disable()
        except Exception:
            pass
        try:
            self.port.close()
        except Exception:
            pass

    def home_against_stop(self) -> None:
        """Probe toward the closed stop under the profile's probe caps, anchor
        against the recorded stop_closed datum, with the URtest sanity guards."""
        if "stop_closed" not in self._entry:
            raise HomingError("entry has no stop_closed datum -- run `autocal full` once")
        p = self.ctrl.profile
        d = p.close_dir
        tracker = MultiTurnTracker()
        fb = self.port.read()
        tracker.update(fb.position)
        start = tracker.position
        t_end = time.monotonic() + 90.0
        contact_since = None
        # Probe-loop damping gain. This is a LOCAL timing/gain constant for the
        # probe move only (not the profile's tmax_nm/vmax-derived main-loop kd,
        # and not a profile field) -- tuned against SimulatedWormGripper's
        # tiny inertia (0.01) at dt=0.02s: kd=1.0 sits past that sim's
        # kd*dt/inertia stability boundary and blows up into a divergent
        # oscillation within a few ticks; kd<=~0.35 settles the ramp-up below
        # the 0.1 rad/s low-velocity contact threshold for longer than the
        # 0.3 s debounce and false-triggers "contact" a few cm off the start
        # position. kd=0.6 is comfortably inside the stable band and clears
        # the ramp-up fast enough to reach the real stop cleanly.
        kd = 0.6
        while True:
            now = time.monotonic()
            if now > t_end:
                raise HomingError("no contact within 90 s")
            v = d * p.probe_vel
            v = min(fb.velocity + p.probe_tmax / kd, max(fb.velocity - p.probe_tmax / kd, v))
            fb = self.port.command(MitCommand(vel=v, kd=kd))
            tracker.update(fb.position)
            if abs(tracker.position - start) > 40.0:
                raise HomingError("traveled 40 rad without contact -- wrong close_dir or no stop")
            contact = abs(fb.torque) > p.contact_torque or abs(fb.velocity) < 0.1
            if contact:
                contact_since = contact_since or now
                if now - contact_since > 0.3:
                    break
            else:
                contact_since = None
            time.sleep(0.02)
        stop_here = tracker.position
        offset = self._entry["stop_closed"] - stop_here
        if abs(offset) > 3 * 6.283185 + 1.0:
            raise HomingError(f"re-anchor offset {offset:+.2f} rad exceeds ~3 turns -- probe "
                              f"likely triggered on friction; raise contact_torque and retry")
        anchored = stop_here + offset
        lo = min(self._entry["open"], self._entry["closed"]) - 2.0
        hi = max(self._entry["open"], self._entry["closed"]) + 2.0
        if not (lo <= anchored <= hi):
            raise HomingError(f"homed position {anchored:+.2f} outside calibrated range -- recalibrate")
        # back off the stop, then hand the anchored frame to the main controller
        back_goal = anchored - d * p.margin
        self.ctrl.adopt_park(anchored)
        self.ctrl.tick(fb, 0.0)
        self.ctrl.goto_rad(back_goal)
        self.wait(timeout=10.0)

    # --- motion ---
    def _run_until(self, predicate, timeout: float) -> None:
        fb = self.port.read()
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            t0 = time.monotonic()
            fb = self.port.command(self.ctrl.tick(fb, 0.02))
            if predicate():
                return
            time.sleep(max(0.0, 0.02 - (time.monotonic() - t0)))
        raise TimeoutError("gripper did not reach the goal in time")

    def wait(self, timeout: float = 10.0) -> None:
        self._run_until(lambda: self.ctrl.at_goal or self.ctrl.stalled, timeout)

    def goto(self, frac: float) -> None:
        self.ctrl.goto_frac(frac)

    def open(self) -> None:
        self.ctrl.open()

    def close(self) -> None:
        self.ctrl.close()

    @property
    def position(self):
        return self.ctrl.position if self.ctrl else None

    @property
    def stalled(self) -> bool:
        return bool(self.ctrl and self.ctrl.stalled)

    # --- park / teardown ---
    def park(self) -> None:
        if self.ctrl and self.ctrl.tracker.seen:
            self._entry.update(self.ctrl.park_fields())
            self._entry["parked_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
            save_store(self._cal_path, self._store)

    def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False
        try:
            if self.ctrl:
                self.ctrl.hold()
            self.park()
        finally:
            try:
                self.port.close()
            except PortError:
                pass

    def __enter__(self):
        if not self._connected:
            self.connect()
        return self

    def __exit__(self, *exc):
        self.disconnect()
