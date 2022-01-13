#!/bin/bash
set -e

echo "Set up the timezone"
echo "==================="

sudo timedatectl set-timezone Asia/Jakarta

echo "Set up the repository"
echo "====================="
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg lsb-release -y
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list

echo "Install Docker Engine"
echo "====================="
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io -y

echo "Manage Docker as a non-root user"
echo "================================"
sudo usermod -aG docker $USER

echo "Manage Dialout as a non-root user"
echo "================================="
sudo usermod -aG dialout $USER

echo "Configure Docker to start on boot"
echo "================================="
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

echo "Install Docker Compose"
echo "======================"
#sudo curl -L "https://github.com/docker/compose/releases/download/1.28.6/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
#sudo chmod +x /usr/local/bin/docker-compose
echo "Python package manager --->>"
sudo apt -y install python3-pip

echo "Install Docker Compose --->>"
sudo pip3 install docker-compose 


echo "Install Lazydocker"
echo "=================="
curl https://raw.githubusercontent.com/jesseduffield/lazydocker/master/scripts/install_update_linux.sh | bash
sudo mv $HOME/lazydocker /usr/local/bin


echo "Modify serial port permission rules"
echo "==================================="
echo -e "KERNEL==\"ttyS*\",MODE=\"0666\"\nKERNEL==\"ttyACM*\",MODE=\"0666\"\nKERNEL==\"ttyUSB*\",MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-serial.rules

echo "Modify docker bip and default address pools"
echo "==========================================="
echo -e "{\n\t\"bip\": \"172.0.0.1/16\",\n\t\"default-address-pools\": [\n\t\t{\n\t\t\t\"base\": \"172.1.0.0/8\",\n\t\t\t\"size\": 16\n\t\t}\n\t]\n}\n" | sudo tee /etc/docker/daemon.json

echo ">>>> Prepare Complete & Please reboot <<<<"
