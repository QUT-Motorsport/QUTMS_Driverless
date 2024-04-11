#Verifying User Decision
read -p "This script will install and configure the QUTMS Driverless software. Please only run this on a fresh Ubuntu installation. Do not run this to rebuild. Proceed? (y/n) " choice
case "$choice" in 
  y|Y ) echo "Proceeding with installation..." ;;
  n|N ) echo "Cancelled." 
        exit 0;;
  * ) echo "Invalid input. Please enter y or n.";;
esac
#Checking for prior installation
package_check_apt="docker-compose"

if hash apt 2>/dev/null || hash apt-get 2>/dev/null; then

    if dpkg-query -W -f='${status}' $package_check_apt | grep -q "ok installed"; then  # Fixed grep
        read -p "A previous Docker Compose installation is detected. Are you sure you want to proceed? (y/n) " choice

        case "$choice" in
        y|Y ) echo "Proceeding with installation..." ;;
        n|N ) echo "Cancelled."
              exit 0;;
        esac

    else
        echo "Proceeding with installation..."
    fi

else
    echo "Previous installation not detected..."
fi
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
mkdir -p /home/$USER/dev/repos/
cd /home/$USER/dev/repos/
# Clone the repository
git clone https://github.com/QUT-Motorsport/QUTMS_Driverless.git --recurse-submodules
cd /home/$USER/dev/repos/QUTMS_Driverless/docker
# Permissions Fix
sudo usermod -aG docker ${USER}
echo "Re-enter password to Assign Permissions"
su -s ${USER}
sudo chown "$USER":"$USER" /home/"$USER"/.docker -R
sudo chmod 666 /var/run/docker.sock 
# making
cd /home/$USER/dev/repos/QUTMS_Driverless/docker
make build_docker_base_roscube
make build target=roscube
