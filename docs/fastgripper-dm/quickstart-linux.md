# Quickstart — Linux + SocketCAN (standalone gripper)

Hardware: the gripper's DM-J4310 on its own CAN channel via a USB-CAN adapter
(candlelight/gs_usb or any SocketCAN device), 24 V to the motor, 120 Ω
termination at both ends. Classic CAN 2.0, 1 Mbit/s, 11-bit IDs.

```sh
sudo ip link set can0 up type can bitrate 1000000     # after every reboot
python3 -m venv ~/fg && source ~/fg/bin/activate
pip install fastgripper-dm
```

## 1. Identity (once per motor, motor ALONE on the bus)

A factory motor answers at 0x01 and replies on 0x00. Give it the gripper
identity (the tool refuses if any other motor answers):

```sh
fastgripper-dm id --interface socketcan --channel can0 --old_id 0x01 --new_id 0x07
fastgripper-dm id --interface socketcan --channel can0 --new_id 0x07 --verify   # after a power cycle
```

## 2. Setup (once per computer) — saves settings and writes the watchdog

```sh
fastgripper-dm setup --interface socketcan --channel can0 --motor_id 0x07 --master_id 0x17
```

## 3. Every session

```sh
fastgripper-dm preflight     # expect: motor answers · watchdog 500 ms · cal … → GO
```

## 4. Calibrate (once per unit; jaws empty)

Keyboard jog: hold `a`/`d`, `f` fast, `o` mark OPEN, `c` mark CLOSED, `q` save.

```sh
fastgripper-dm calibrate
```

Or, if the gripper has a qualified closed hard stop, let it find the stop:

```sh
fastgripper-dm autocal full --closed_only --span_from_closed 30.0 --yes
```

`--span_from_closed` MUST be a conservative underestimate of the real travel
(rad of shaft rotation from the closed stop to fully open).

## 5. Move

```sh
fastgripper-dm drive 0      # closed
fastgripper-dm drive 100    # open
fastgripper-dm autocal home # re-anchor after any unclean exit (~20 s)
```

Stuck? `fastgripper-dm doctor` then `docs/fastgripper-dm/troubleshooting.md`.
