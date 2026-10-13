#!/bin/bash
# Capture CYD boot log via reconnecting USB-CDC reader (survives re-enumeration).
# Uses the repo-local reset_and_capture.py (pass --no-reset semantics by
# pre-resetting the board manually; plain capture only needs the reader part).
# Falls back to the IDF python env for pyserial.
exec ~/.espressif/python_env/idf5.3_py3.12_env/bin/python \
  "$(dirname "$0")/reset_and_capture.py" "${1:-25}"
