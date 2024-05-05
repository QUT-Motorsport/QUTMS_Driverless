#!/bin/bash

# download this script into your home user folder
# eg. C:\Users\YOURNAME\
# chmod +x ssh_connection.sh
# to run it use "./ssh_connection.sh"

# addresses and usernames
roscube='qutms@192.168.3.2'
panda='qev3d@192.168.3.4'
jetson='qutms@192.168.3.3'
rasbipi='qutms@192.168.3.20'

servername=''
# if no opts
if [ $# -eq 0 ]; then
  echo "No arguments supplied"
  exit 1
fi

# get args
# format is -s servername
while getopts 's:' flag; do
  case "${flag}" in
    s) servername="${OPTARG}" ;;
    *) echo "Unexpected option ${flag}" ;;
  esac
done

# if servername is empty
if [ -z "$servername" ]; then
  echo "Servername is empty"
  exit 1
fi

# if servername is not in the list
if [ "$servername" != "roscube" ] && [ "$servername" != "jetson" ] && [ "$servername" != "rasbipi" ] && [ "$servername" != "panda" ]; then
  echo "Servername is not in the list"
  exit 1
fi

# if servername is in the list
if [ "$servername" == "roscube" ]; then
  servername=$roscube
fi
if [ "$servername" == "jetson" ]; then
  servername=$jetson
fi
if [ "$servername" == "rasbipi" ]; then
  servername=$rasbipi
fi
if [ "$servername" == "panda" ]; then
  servername=$panda
fi

echo "Connecting to $servername..."

until ssh ${servername}; do
    echo "Server is down, retrying..."
    sleep 3
done
