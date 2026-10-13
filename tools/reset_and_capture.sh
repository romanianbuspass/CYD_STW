#!/bin/bash
exec ~/.espressif/python_env/idf5.3_py3.12_env/bin/python \
  "$(dirname "$0")/reset_and_capture.py" "${1:-20}"
