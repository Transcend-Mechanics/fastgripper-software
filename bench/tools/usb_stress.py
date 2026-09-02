"""USB link stress tools for the two-rig bench (no motion).

  soak   read every serial device at 60 Hz and ping the CAN bus concurrently
         for --seconds; reports per-link success/failure and any device that
         vanished. Ports must be free (no teleop running).
  watch  poll /dev for the serial nodes and the CAN adapter every 0.2 s and
         log every disappearance/return with a timestamp. Opens nothing, so
         run it DURING teleop to catch hub drops while the rigs are working.

Ports come from bench/local.toml (git-ignored, per-bench device paths) --
see bench/local.toml.example for the format.

Usage:
  I2RT_CAN_BUSTYPE=gs_usb .venv/bin/python bench/tools/usb_stress.py soak --seconds 120
  .venv/bin/python bench/tools/usb_stress.py watch            # Ctrl-C to stop
"""

import argparse
import os
import sys
import threading
import time

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL_TOML = os.path.join(BENCH_DIR, "local.toml")
LOCAL_TOML_EXAMPLE = os.path.join(BENCH_DIR, "local.toml.example")


def load_ports() -> dict[str, str]:
    # Lazy: tomllib is stdlib on 3.11+ only, and this module must still
    # `import` cleanly on the package's 3.10 venv (see the import smoke
    # check in bench/README.md) even though nothing there calls load_ports().
    try:
        import tomllib
    except ImportError:  # Python < 3.11
        import tomli as tomllib
    try:
        with open(LOCAL_TOML, "rb") as f:
            data = tomllib.load(f)
    except OSError:
        raise SystemExit(
            f"{LOCAL_TOML} not found -- copy {LOCAL_TOML_EXAMPLE} to "
            f"{LOCAL_TOML} and fill in this bench's device paths "
            "(bench/local.toml is git-ignored).")
    ports = data.get("ports")
    if not ports:
        raise SystemExit(
            f"{LOCAL_TOML} has no [ports] table -- see {LOCAL_TOML_EXAMPLE}")
    return ports


def can_present() -> bool:
    try:
        import usb.core
        return usb.core.find(idVendor=0x1D50, idProduct=0x606F) is not None
    except Exception:
        return False


def soak(seconds: float) -> int:
    # scservo_sdk is not installed in every environment this module gets
    # imported into (e.g. the package venv's import smoke test) -- keep it
    # lazy so `import usb_stress` alone never requires it.
    import can
    import scservo_sdk as scs
    from fastgripper_dm.damiao.canbus import open_bus

    ports = load_ports()
    res: dict[str, tuple[int, int, str]] = {}

    def serial_soak(name: str, port: str) -> None:
        if not os.path.exists(port):
            res[name] = (0, 0, "ABSENT at start")
            return
        ph = scs.PortHandler(port)
        ok = fail = 0
        note = ""
        t0 = time.time()
        try:
            if not ph.openPort() or not ph.setBaudRate(1_000_000):
                res[name] = (0, 0, "cannot open")
                return
            pk = scs.PacketHandler(0)
            rd = scs.GroupSyncRead(ph, pk, 56, 2)
            for i in range(1, 7):
                rd.addParam(i)
            while time.time() - t0 < seconds:
                if rd.txRxPacket() == scs.COMM_SUCCESS:
                    ok += 1
                else:
                    fail += 1
                    if not os.path.exists(port):
                        note = f"VANISHED at {time.time() - t0:.1f}s"
                        break
                time.sleep(1 / 60)
        except Exception as e:
            note = f"EXC at {time.time() - t0:.1f}s: {str(e).splitlines()[0][:60]}"
        finally:
            try:
                ph.closePort()
            except Exception:
                pass
        res[name] = (ok, fail, note)

    def can_soak() -> None:
        ok = fail = 0
        note = ""
        t0 = time.time()
        try:
            with open_bus("gs_usb", "0") as bus:
                while time.time() - t0 < seconds:
                    bus.send(can.Message(arbitration_id=1, data=[0xFF] * 7 + [0xFD], is_extended_id=False))
                    got = False
                    t1 = time.monotonic()
                    while time.monotonic() - t1 < 0.1:
                        m = bus.recv(timeout=0.02)
                        if m is not None and m.is_rx and not m.is_error_frame and (m.data[0] & 0x0F) == 1:
                            got = True
                            break
                    ok += got
                    fail += not got
                    time.sleep(0.05)
        except BaseException as e:  # open_bus raises SystemExit on a dead bus
            note = f"EXC at {time.time() - t0:.1f}s: {str(e).splitlines()[0][:60]}"
        res["CAN(motor 1)"] = (ok, fail, note)

    threads = [threading.Thread(target=serial_soak, args=(n, p)) for n, p in ports.items()]
    threads.append(threading.Thread(target=can_soak))
    print(f"soaking {len(threads)} links for {seconds:.0f}s (no motion)...", flush=True)
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    bad = 0
    for name, (ok, fail, note) in res.items():
        flag = "FAIL" if note or (ok == 0) else ("WARN" if fail > ok * 0.01 else "OK  ")
        bad += flag == "FAIL"
        print(f"  [{flag}] {name:>13}: ok={ok:5d} fail={fail:3d} {note}", flush=True)
    print("soak:", "PASS" if not bad else "FAIL", flush=True)
    return 0 if not bad else 1


def watch() -> None:
    ports = load_ports()
    state = {n: os.path.exists(p) for n, p in ports.items()}
    state["CAN"] = can_present()
    t0 = time.time()
    print("watching " + ", ".join(f"{n}={'up' if v else 'DOWN'}" for n, v in state.items()) + "  (Ctrl-C to stop)", flush=True)
    events = 0
    try:
        while True:
            time.sleep(0.2)
            now = {n: os.path.exists(p) for n, p in ports.items()}
            now["CAN"] = can_present()
            for n, v in now.items():
                if v != state[n]:
                    events += 1
                    print(f"{time.strftime('%H:%M:%S')} +{time.time() - t0:7.1f}s  {n}: {'RETURNED' if v else 'VANISHED'}", flush=True)
                    state[n] = v
    except KeyboardInterrupt:
        pass
    print(f"\nwatched {time.time() - t0:.0f}s, {events} drop/return events", flush=True)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("soak")
    s.add_argument("--seconds", type=float, default=120)
    sub.add_parser("watch")
    a = p.parse_args()
    if a.cmd == "soak":
        rc = soak(a.seconds)
        sys.stdout.flush()
        os._exit(rc)   # libusb finalizer SIGABRT on macOS
    else:
        watch()


if __name__ == "__main__":
    main()
