#!/bin/bash

# Set hostname for CD2
echo -e "\e[1;34mSetting hostname to cd2-raspberry...\e[0m"
sudo hostnamectl set-hostname cd2-raspberry

# Update hosts file
echo -e "\e[1;33mUpdating /etc/hosts...\e[0m"
sudo bash -c 'cat > /etc/hosts' << EOF
127.0.0.1       localhost
127.0.1.1       cd2-raspberry
EOF

# Update and upgrade the system
echo -e "\e[1;32mUpdating and upgrading the system...\e[0m"
sudo apt update && sudo apt upgrade -y

# Install tmux
echo -e "\e[1;35mInstalling tmux...\e[0m"
sudo apt install -y tmux

# Install pip3
echo -e "\e[1;36mInstalling pip3...\e[0m"
sudo apt install -y python3-pip

# Install lxml and numpy using pip3
echo -e "\e[1;37mInstalling lxml and numpy...\e[0m"
sudo apt install -y python3-lxml python3-numpy

# Change directory to rpz
cd rpz || { echo -e "\e[1;31mFailed to change directory to rpz\e[0m"; exit 1; }

# Install dependencies from requirements.txt
echo -e "\e[1;34mInstalling dependencies from requirements.txt...\e[0m"
sudo pip3 install -r req.txt

# Go back to the parent directory
cd ..

# Change directory to /home/oxi3
cd /home/oxi3 || { echo -e "\e[1;31mFailed to change directory to /home/oxi3\e[0m"; exit 1; }

# Create mav.sh and add the socat command
echo -e "\e[1;33mCreating mav.sh and adding the socat command...\e[0m"
cat <<EOF | sudo tee mav.sh > /dev/null
#!/bin/bash
# Kill all existing tmux sessions
sudo tmux kill-session
sudo tmux new-session -d -s mav 'socat UDP4-DATAGRAM:192.168.22.161:14552 /dev/serial0,b115200,raw,echo=0'
echo -e "\e[1;32mRunning mav.sh and adding the socat command...\e[0m"
EOF

# Make mav.sh executable
echo -e "\e[1;35mMaking mav.sh executable...\e[0m"
sudo chmod +x mav.sh

# Create run.sh to run rpz/run.py in tmux
echo -e "\e[1;36mCreating run.sh to run rpz/run.py in tmux...\e[0m"
cat <<EOF | sudo tee run.sh > /dev/null
#!/bin/bash
sudo tmux kill-session
sudo tmux new-session -d -s run '/usr/bin/python3 /home/oxi3/rpz/run.py'
echo -e "\e[1;37mRPZ is running in tmux session 'run'\e[0m"
EOF

# Make run.sh executable
echo -e "\e[1;34mMaking run.sh executable...\e[0m"
sudo chmod +x run.sh

# Add run.sh to crontab for automatic execution on reboot
echo -e "\e[1;33mAdding run.sh to crontab for automatic execution on reboot...\e[0m"
(crontab -l 2>/dev/null; echo "@reboot /home/oxi3/run.sh") | crontab -

# Display success message
echo -e "\e[1;32mCD2 Setup completed successfully!\e[0m"
echo -e "\e[1;35mSystem will reboot in 5 seconds...\e[0m"

echo -e "\e[1;36mDo ssh into the CD2 using: ssh oxi3@cd2-raspberry\e[0m"

sleep 5
sudo reboot 