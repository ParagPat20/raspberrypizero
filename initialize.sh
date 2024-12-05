#!/bin/bash

# Update and upgrade the system
echo "Updating and upgrading the system..."
sudo apt update && sudo apt upgrade -y

# Install tmux
echo "Installing tmux..."
sudo apt install -y tmux

# Install git
echo "Installing git..."
sudo apt install -y git

# Install pip3
echo "Installing pip3..."
sudo apt install -y python3-pip

# Install lxml and numpy using pip3
echo "Installing lxml and numpy..."
sudo apt install python3-lxml python3-numpy

# Clone the repository
echo "Cloning the repository from GitHub..."
git clone https://github.com/ParagPat20/rpz

# Change directory to rpz
cd rpz

# Install dependencies from requirements.txt
echo "Installing dependencies from requirements.txt..."
sudo pip3 install -r requirements.txt

# Go back to the parent directory
cd ..

# Create mav.sh and add the socat command
echo "Creating mav.sh and adding the socat command..."
sudo nano mav.sh <<EOF
#!/bin/bash
socat UDP4-DATAGRAM:192.168.22.161:14550 /dev/serial0,b115200,raw,echo=0
EOF

# Make mav.sh executable
echo "Making mav.sh executable..."
chmod +x mav.sh

# Create run.sh to run rpz/run.py in tmux
echo "Creating run.sh to run rpz/run.py in tmux..."
sudo nano run.sh <<EOF
#!/bin/bash
tmux new-session -d -s run 'python3 /home/oxi/rpz/run.py'
EOF

# Make run.sh executable
echo "Making run.sh executable..."
chmod +x run.sh

# Add run.sh to crontab for automatic execution on reboot
echo "Adding run.sh to crontab for automatic execution on reboot..."
(crontab -l 2>/dev/null; echo "@reboot /home/oxi/run.sh") | crontab -

# Display success message
echo "Setup completed successfully!"

