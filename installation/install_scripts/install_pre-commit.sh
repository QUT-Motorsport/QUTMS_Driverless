#!/bin/bash

echo ""
echo "---Installing pre-commit---"
echo ""
sleep 3
sudo apt install pre-commit -y
cd ~/QUTMS/QUTMS_Driverless
pre-commit install
