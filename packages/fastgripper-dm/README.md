# fastgripper-dm

Driver, calibration and CLI for the FastGripper worm-gear parallel gripper
when its actuator is a Damiao DM-J4310 on a **dedicated CAN channel**
(standalone mode). Linux + SocketCAN is the shipped path; macOS + gs_usb is
supported for bench use.

```sh
pip install fastgripper-dm

# one-time: give the motor its identity (motor ALONE on the bus) and its watchdog
fastgripper-dm id --interface socketcan --channel can0 --new_id 0x07
fastgripper-dm setup --interface socketcan --channel can0 --motor_id 0x07 --master_id 0x17

# every session
fastgripper-dm preflight            # no motion: bus, motor, fault, watchdog, calibration
fastgripper-dm calibrate            # keyboard jog, mark open/closed (once per unit)
fastgripper-dm autocal home         # re-anchor against the closed stop (~20 s)
fastgripper-dm drive 40             # 40 % and exit
```

Docs: `docs/fastgripper-dm/` at the repo root (quickstart-linux, quickstart-mac,
troubleshooting). Driving the gripper on an i2rt YAM arm instead of a
dedicated CAN channel? See `docs/fastgripper-dm/yam.md`.

Safety: the `setup` step writes a comm-loss watchdog into the motor so a
crashed host cannot leave the worm pushing into a stop. `preflight` refuses to
pass with the watchdog disabled.
