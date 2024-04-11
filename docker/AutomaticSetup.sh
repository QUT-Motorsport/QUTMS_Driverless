#Verifying User Decision
read -p "This script will install and configure the QUTMS Driverless software. Please only run this on a fresh Ubuntu installation. Do not run this to rebuild. Proceed? (y/n) " choice
case "$choice" in 
  y|Y ) echo "Proceeding with installation..." ;;
  n|N ) echo "Cancelled." 
        exit 0;;
  * ) echo "Invalid input. Please enter y or n.";;
esac
# Install packages
sudo apt update -y && sudo apt upgrade -y
sudo apt install -y curl systemd cmake git
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
# Docker Install Procedure
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null  
sudo apt-get update -y
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt install -y docker-compose
# Create directory 
mkdir -p /home/ros2/dev/repos/
cd /home/ros2/dev/repos/
# Clone the repository
git clone https://github.com/QUT-Motorsport/QUTMS_Driverless.git --recurse-submodules
cd /home/ros2/dev/repos/QUTMS_Driverless/docker

# Permissions Fix
sudo usermod -aG docker ${USER}
echo "Re-enter password to Assign Permissions"
su -s ${USER}
sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
sudo chmod 666 /var/run/docker.sock