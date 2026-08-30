"""Console-entry wrapper: run a tool's main() and exit via os._exit.

On macOS, Python interpreter finalization can crash inside libusb (SIGABRT
during GC of gs_usb objects), and a process that dies that way can leave the
USB adapter in a bad state for the next session. All hardware cleanup
(motor disable, bus shutdown/drain) happens inside each tool's main() before
this fires -- finalization has nothing left to do, so skip it.
"""

import os
import sys


def run(main) -> None:
    code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)
            code = 1
        else:
            code = e.code if e.code is not None else 0
    except KeyboardInterrupt:
        code = 130
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
