#!/usr/bin/env bash
# Apply the macOS-only fix so gs_usb (candlelight adapters) works on Darwin.
# Usage: patches/setup-mac.sh /path/to/.venv
set -euo pipefail
VENV="${1:?venv path}"
SITE="$("$VENV/bin/python" -c 'import gs_usb, os; print(os.path.dirname(gs_usb.__file__))')"
patch -p1 -N -d "$(dirname "$SITE")" < "$(dirname "$0")/gs_usb-darwin.patch" || echo "gs_usb patch already applied"
echo "gs_usb patched in $SITE"
