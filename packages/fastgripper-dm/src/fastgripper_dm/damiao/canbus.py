"""Shared CAN bus opening: gs_usb / slcan (macOS adapters) or socketcan (Linux).

socketcan expects the interface to already be up at the right bitrate, e.g.:
    sudo ip link set can0 up type can bitrate 1000000
"""

import glob
import time

import can


class BusDead(SystemExit):
    """The bus opened but passes no frames (nothing ACKs). Subclasses SystemExit so
    the CLI wrapper prints the recovery text; library callers can catch it."""


def open_bus(interface: str = "auto", channel: str | None = None) -> can.BusABC:
    if interface == "auto":
        # Linux with a native CAN netdev (e.g. Jetson): use socketcan.
        candevs = sorted(glob.glob("/sys/class/net/can*"))
        if not channel and candevs:
            interface, channel = "socketcan", candevs[0].rsplit("/", 1)[-1]
            print(f"(auto: socketcan on {channel})")
            return can.Bus(interface="socketcan", channel=channel)
        # slcan-flashed adapter shows up as a serial port; prefer it (it's the
        # reliable one). Fall back to a candlelight/gs_usb device.
        ports = sorted(glob.glob("/dev/tty.usbmodem*"))
        if channel or ports:
            interface, channel = "slcan", channel or ports[0]
            print(f"(auto: slcan on {channel})")
        else:
            interface = "gs_usb"
            print("(auto: no serial adapter found, trying gs_usb)")

    if interface == "slcan":
        if not channel:
            raise SystemExit("slcan needs a channel, e.g. /dev/tty.usbmodem2101")
        return can.Bus(interface="slcan", channel=channel, bitrate=1_000_000)

    if interface == "socketcan":
        # bitrate is set on the interface itself (ip link), not here
        if not channel:
            raise SystemExit("socketcan needs a channel, e.g. can0")
        return can.Bus(interface="socketcan", channel=channel)

    if interface == "gs_usb":
        # macOS: no kernel drivers to detach; gs_usb's calls raise USBError.
        import usb.core
        import usb.util

        _orig_detach = usb.core.Device.detach_kernel_driver
        _orig_active = usb.core.Device.is_kernel_driver_active

        def _safe_detach(self, intf):
            try:
                _orig_detach(self, intf)
            except usb.core.USBError:
                pass

        def _safe_active(self, intf):
            try:
                return _orig_active(self, intf)
            except usb.core.USBError:
                return False

        usb.core.Device.detach_kernel_driver = _safe_detach
        usb.core.Device.is_kernel_driver_active = _safe_active

        def _hard_reset() -> None:
            """Last-resort clear for a wedged adapter. NOTE: a software reset
            leaves the device half-re-enumerated on macOS -- the session that
            performs it usually works, but the NEXT process often inherits a
            flaky open (this was the source of the standalone->i2rt failure
            cascade). So: reset only on evidence the bus is dead, never
            preemptively."""
            for vid, pid in [(0x1D50, 0x606F), (0x1209, 0x2323)]:
                dev = usb.core.find(idVendor=vid, idProduct=pid)
                if dev is not None:
                    try:
                        dev.reset()
                    except usb.core.USBError:
                        pass
                    usb.util.dispose_resources(dev)
                    time.sleep(1.5)
                    break

        def _open():
            return can.Bus(interface="gs_usb", channel=channel or 0, index=0, bitrate=1_000_000)

        def _bus_alive(bus) -> bool:
            """Send a harmless frame to an unused ID; our own TX echo coming
            back means it was ACKed on the wire -- the pipeline works."""
            try:
                bus.send(can.Message(arbitration_id=0x40, data=[0xFF] * 7 + [0xFD],
                                     is_extended_id=False))
            except can.CanError:
                return False
            t0 = time.time()
            while time.time() - t0 < 0.5:
                m = bus.recv(timeout=0.05)
                if m is not None and not m.is_rx:
                    return True
            return False

        def _arm_drain_on_shutdown(bus) -> None:
            """Read the device dry before stopping it. Closing the channel
            with unread frames in the adapter's USB pipeline wedges the
            candlelight TX path until a physical replug (the autocal->next-
            session poison of 2026-07-21). Clean sessions drain by habit;
            make it guaranteed."""
            orig_shutdown = bus.shutdown

            def drain_and_shutdown(*a, **kw):
                try:
                    t0 = time.time()
                    while time.time() - t0 < 1.0:
                        if bus.recv(timeout=0.05) is None:
                            break  # quiet: pipeline drained
                except Exception:
                    pass
                orig_shutdown(*a, **kw)
                # release the USB handle NOW (os._exit skips finalization;
                # kernel-side async reclaim races the next session's open)
                try:
                    usb.util.dispose_resources(bus.gs_usb.gs_usb)
                except Exception:
                    pass

            bus.shutdown = drain_and_shutdown

        # Open WITHOUT resetting; verify frames actually flow; reset + reopen
        # only if they don't.
        try:
            bus = _open()
        except Exception as e:
            # Say WHY: a missing `gs_usb` package or an unpatched pyusb call
            # looks identical to a dead bus otherwise (seen live 2026-08-30).
            raise BusDead(
                f"could not open the gs_usb adapter: {type(e).__name__}: {e}\n"
                "  - is the adapter plugged in (VID 0x1D50 / PID 0x606F)?\n"
                "  - is the `gs_usb` package installed in this environment?\n"
                "  - macOS: run patches/setup-mac.sh <venv> once") from e
        if _bus_alive(bus):
            _arm_drain_on_shutdown(bus)
            return bus
        try:
            bus.shutdown()
        except Exception:
            pass
        # Do NOT software-reset: on this Mac dev.reset() knocks the adapter
        # off the USB bus entirely ([Errno 19] until physical replug) --
        # proven 2026-07-21. Report honestly and let the human replug.
        raise BusDead(
            "bus opened but passes NO frames (no TX echo -- nothing ACKs).\n"
            "Physical recovery required:\n"
            "  1. unplug/replug the adapter USB (LED green at rest)\n"
            "  2. if motors show flashing LEDs, power-cycle the 24V PSU\n"
            "then rerun.")

    raise SystemExit(f"unsupported interface: {interface}")


def add_bus_args(parser) -> None:
    parser.add_argument("--interface", choices=["auto", "gs_usb", "slcan", "socketcan"], default="auto")
    parser.add_argument("--channel", default=None, help="serial port for slcan / netdev for socketcan")
    parser.add_argument("--motor_id", type=lambda s: int(s, 0), default=0x01,
                        help="motor CAN ID (default 0x01 = factory; the cal entry's id wins when set)")
    parser.add_argument("--master_id", type=lambda s: int(s, 0), default=0x00,
                        help="motor feedback/master ID (default 0x00 = factory)")
