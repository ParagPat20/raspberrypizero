#!/bin/bash

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

# Make initialize.sh executable
echo "Making initialize.sh executable..."
chmod +x initialize.sh

# Create mav.sh and add the socat command
echo "Creating mav.sh and adding the socat command..."
cat <<EOF | sudo tee mav.sh > /dev/null
#!/bin/bash
# Kill all existing tmux sessions
sudo tmux kill-sessions
# Create a new tmux session named 'mav' and run the socat command
sudo tmux new-session -d -s mav 'socat UDP4-DATAGRAM:192.168.22.161:14550 /dev/serial0,b115200,raw,echo=0'
EOF

# Make mav.sh executable
echo "Making mav.sh executable..."
chmod +x mav.sh

# Create run.sh to run rpz/run.py in tmux
echo "Creating run.sh to run rpz/run.py in tmux..."
cat <<EOF | sudo tee run.sh > /dev/null
#!/bin/bash
sudo tmux kill-sessions
sudo tmux new-session -d -s run '/usr/bin/python3 /home/oxi/rpz/run.py'
EOF

# Make run.sh executable
echo "Making run.sh executable..."
chmod +x run.sh

# Add run.sh to crontab for automatic execution on reboot
echo "Adding run.sh to crontab for automatic execution on reboot..."
(crontab -l 2>/dev/null; echo "@reboot /home/oxi/run.sh") | crontab -

# Display success message
echo "Setup completed successfully!"