"""Console-entry wrapper: run a tool's main() and exit via os._exit.

On macOS, Python interpreter finalization can crash inside libusb (SIGABRT
during GC of gs_usb objects), and a process that dies that way can leave the
USB adapter in a bad state for the next session. All hardware cleanup
(motor disable, bus shutdown/drain) happens inside each tool's main() before
this fires -- finalization has nothing left to do, so skip it.
"""

import os
import sys

from ..facade import HomingError
from ..port import PortError


def run(main) -> None:
    code = 0
    try:
        main()
    except (PortError, HomingError) as e:
        # A HomingError that escaped here used to go through normal interpreter
        # finalization, which is exactly the libusb-abort path this wrapper exists
        # to avoid; report it and take the os._exit route like any other tool error.
        print(str(e), file=sys.stderr)
        code = 1
    except TimeoutError as e:
        print(str(e), file=sys.stderr)
        code = 1
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
