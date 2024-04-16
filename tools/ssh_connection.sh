#!/bin/bash
# ^idk what this is i just keep seeing it - it's a shebang

# chmod +x ssh_connection.sh
# to run it use "bash ssh_connection.sh"


roscube='192.168.3.2'
jetson='192.168.3.3'
rasbipi='192.168.3.20'
# idk if this does what i think it does :)

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
if [ "$servername" != "roscube" ] && [ "$servername" != "jetson" ] && [ "$servername" != "rasbipi" ]; then
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

echo "Connecting to $servername..."

until ssh ${servername}; do
    echo "Server is down, retrying in 5 seconds"
    sleep 5
done
