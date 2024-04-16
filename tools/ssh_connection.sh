#!/bin/bash
# ^idk what this is i just keep seeing it

# chmod +x remote_ease_of_access.sh
# to run it use "bash remote_ease_of_access.sh"


roscube='192.168.3.2'
jetson='192.168.3.3'
rasbipi='192.168.3.20'
# idk if this does what i think it does :)


repeat()
{
read -p "Enter the hostname or IP of your server :" servername
until ssh ${!servername}; do
    sleep 5
done
}
