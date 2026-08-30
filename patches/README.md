# patches/

Reproducible copies of every out-of-tree fix the bench depends on.

| what | where | applied by |
| --- | --- | --- |
| i2rt bench patches (gs_usb backend, drain before bring-up, response-id matching, `I2RT_CAN_RESPONSE_TIMEOUT`, exception-safe recovery, config-tool fixes) | branch `fastgripper` in the local i2rt clone (`YAM Test/i2rt`, commits `f732e4f`, `4595129` on top of upstream main); pushed to a `Transcend-Mechanics/i2rt` fork in Plan 2 | `pip install "i2rt @ git+https://github.com/Transcend-Mechanics/i2rt@fastgripper"` (after the fork exists) |
| gs_usb 0.3.1 darwin guard (`is_kernel_driver_active` is Linux-only) | `gs_usb-darwin.patch` | `patches/setup-mac.sh <venv>` |
| ruckig 0.15.3 macOS build | `ruckig-build.md` | manual, i2rt bench only |

Linux needs none of these. `fastgripper-dm`'s own `canbus.py` also monkeypatches the two pyusb calls at runtime, so the standalone tools work on macOS even without the patch.
