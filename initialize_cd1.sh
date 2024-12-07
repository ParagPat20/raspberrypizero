#!/bin/bash

# Set hostname for CD1
echo "Setting hostname to cd1-raspberry..."
sudo hostnamectl set-hostname cd1-raspberry

# Update hosts file
echo "Updating /etc/hosts..."
sudo bash -c 'cat > /etc/hosts' << EOF
127.0.0.1       localhost
127.0.1.1       cd1-raspberry
EOF

# Update and upgrade the system
echo "Updating and upgrading the system..."
sudo apt update && sudo apt upgrade -y

# Install tmux
echo "Installing tmux..."
sudo apt install -y tmux

# Install pip3
echo "Installing pip3..."
sudo apt install -y python3-pip

# Install lxml and numpy using pip3
echo "Installing lxml and numpy..."
sudo apt install -y python3-lxml python3-numpy

# Change directory to rpz
cd rpz || { echo "Failed to change directory to rpz"; exit 1; }

# Install dependencies from requirements.txt
echo "Installing dependencies from requirements.txt..."
sudo pip3 install -r req.txt

# Go back to the parent directory
cd ..

# Change directory to /home/oxi
cd /home/oxi || { echo "Failed to change directory to /home/oxi"; exit 1; }

# Create mav.sh and add the socat command
echo "Creating mav.sh and adding the socat command..."
cat <<EOF | sudo tee mav.sh > /dev/null
#!/bin/bash
# Kill all existing tmux sessions
sudo tmux kill-session
sudo tmux new-session -d -s mav 'socat UDP4-DATAGRAM:192.168.22.161:14551 /dev/serial0,b115200,raw,echo=0'
echo "Running mav.sh and adding the socat command..."
EOF

# Make mav.sh executable
echo "Making mav.sh executable..."
sudo chmod +x mav.sh

# Create run.sh to run rpz/run.py in tmux
echo "Creating run.sh to run rpz/run.py in tmux..."
cat <<EOF | sudo tee run.sh > /dev/null
#!/bin/bash
sudo tmux kill-session
sudo tmux new-session -d -s run '/usr/bin/python3 /home/oxi/rpz/run.py'
echo "RPZ is running in tmux session 'run'"
EOF

# Make run.sh executable
echo "Making run.sh executable..."
sudo chmod +x run.sh

# Add run.sh to crontab for automatic execution on reboot
echo "Adding run.sh to crontab for automatic execution on reboot..."
(crontab -l 2>/dev/null; echo "@reboot /home/oxi/run.sh") | crontab -


# Display success message
echo "CD1 Setup completed successfully!"
echo "System will reboot in 5 seconds..."

echo "Do ssh into the CD1 using: ssh oxi@cd1-raspberry"

sleep 5
sudo reboot 