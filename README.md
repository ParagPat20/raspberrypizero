# RPZ Setup Guide

Welcome to the RPZ project! This guide will walk you through the steps to set up the project on your system, including installing necessary dependencies and configuring scripts for automated execution.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation Steps](#installation-steps)
- [Running the Project](#running-the-project)
- [Scripts Overview](#scripts-overview)


## Prerequisites

Before you begin, ensure you have the following installed on your system:
- **Git**: To clone the repository.
- **Bash**: The script is written in Bash, so a compatible shell is required.

## Installation Steps

Follow these steps to set up the RPZ project:

1. **Clone the Repository**
   ```bash
   git clone https://github.com/ParagPat20/rpz.git
   cd rpz
   ```
   Run the Initialization Script The initialize.sh script will handle the installation of dependencies and setup.
```
chmod +x initialize.sh
./initialize.sh
```
Verify Installation Ensure that all dependencies are installed correctly. The script will output messages indicating the success of each step.
`
Running the Project`
After the setup is complete, you can run the project using the provided scripts:

```
crontab -l
```
Scripts Overview
`
initialize.sh
`
This script performs the following tasks:

Updates and upgrades the system.
Installs necessary packages (tmux, pip3, lxml, numpy).
Clones the RPZ repository.
Installs Python dependencies from requirements.txt.
`
Creates mav.sh and run.sh scripts for managing sessions.`
`
mav.sh
`
Kills all existing tmux sessions.
Creates a new tmux session named mav to run the socat command.
`
run.sh
`
Creates a new tmux session named run to execute the run.py script.


