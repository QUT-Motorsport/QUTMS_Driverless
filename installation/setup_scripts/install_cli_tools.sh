#!/bin/bash

cd ~/QUTMS/QUTMS_Driverless/installation/cli_test

pip install build
python -m build
pip install dist/cli_test-1.0.0-py3-none-any.whl
