#!/bin/bash

# purge old workspace
rm -rf ~/EUFS/build/ ~/EUFS/log/ ~/EUFS/install/
rm -rf ~/QUTMS_Driverless/build/ ~/QUTMS_Driverless/log/ ~/QUTMS_Driverless/install/

cd ~
mkdir QUTMS
cd QUTMS

mkdir bags/

cp -r ~/QUTMS_Driverless ~/QUTMS/QUTMS_Driverless
cp -r ~/EUFS/eufs_sim ~/QUTMS/eufs_sim
cp -r ~/EUFS/eufs_msgs ~/QUTMS/eufs_msgs
cp -r ~/EUFS/eufs_rviz_plugins ~/QUTMS/eufs_rviz_plugins

rcFile="~/.bashrc"
# export variable to delete
prop="EUFS_MASTER"   
if grep -q "^export $prop=" "$rcFile"; then
  sed -i "/^export $prop=.*$/d" "$rcFile" &&
  echo "[deleted] export $prop"
else
  echo "[not found] export $prop"
fi

# new export variables
echo "export EUFS_MASTER='~/QUTMS'" >> ~/.bashrc
echo "export QUTMS_WS='~/QUTMS'" >> ~/.bashrc
