#!/bin/bash
set -e

# Set up the timezone
# ===================
sudo timedatectl set-timezone Asia/Jakarta

# Set up the repository
# =====================
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg lsb-release -y
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list

# Install Docker Engine
# =====================
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io -y

# Manage Docker as a non-root user
# ================================
sudo usermod -aG docker $USER

# Manage Dialout as a non-root user
# =================================
sudo usermod -aG dialout $USER

# Configure Docker to start on boot
# =================================
sudo systemctl enable docker.service
sudo systemctl enable containerd.service

# Install Docker Compose
# ======================
sudo curl -L "https://github.com/docker/compose/releases/download/1.28.6/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Install Lazydocker
# ==================
curl https://raw.githubusercontent.com/jesseduffield/lazydocker/master/scripts/install_update_linux.sh | bash

# Modify serial port permission rules
# ===================================
echo -e "KERNEL==\"ttyS*\",MODE=\"0666\"\nKERNEL==\"ttyACM*\",MODE=\"0666\"\nKERNEL==\"ttyUSB*\",MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-serial.rules

# Modify docker bip and default address pools
# ===========================================
echo -e "{\n\t\"bip\": \"172.0.0.1/16\",\n\t\"default-address-pools\": [\n\t\t{\n\t\t\t\"base\": \"172.1.0.0/8\",\n\t\t\t\"size\": 16\n\t\t}\n\t]\n}\n" | sudo tee /etc/docker/daemon.json