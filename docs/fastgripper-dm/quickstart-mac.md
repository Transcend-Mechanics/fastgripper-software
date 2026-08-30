# Quickstart — macOS + gs_usb (bench use)

Same commands as the Linux quickstart with `--interface gs_usb --channel 0`
(candlelight firmware, VID 0x1D50 / PID 0x606F; no serial port appears) or
`--interface slcan --channel /dev/tty.usbmodemXXXX` for an slcan-flashed adapter.

Apply the macOS gs_usb fix into your venv once:

```sh
patches/setup-mac.sh ~/fg
```

Known macOS behaviours (all handled by the tools, listed so they don't surprise you):

- The adapter wedges if a session dies without draining the bus, or after
  sitting idle overnight. Symptom: `preflight` says "no TX echo". Fix: unplug
  and replug the adapter's USB — never a software reset (that knocks it off
  USB entirely).
- Tools exit through `os._exit` after their own cleanup: Python's
  interpreter finalization crashes inside libusb on macOS.
- Long-running tools hold the Mac awake; an idle sleep freezes the loop and
  the motor's watchdog will (correctly) drop torque.
