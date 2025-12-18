---
sidebar_position : 1
---

# ROS 2 Installation Guide

This guide provides step-by-step instructions for installing ROS 2 Humble or Iron on Ubuntu, specifically tailored for a Windows Subsystem for Linux (WSL) environment or a native Ubuntu installation.

## Prerequisites

*   **Operating System** : Ubuntu 22.04 LTS (Jammy Jellyfish) for ROS 2 Humble, or Ubuntu 20.04 LTS (Focal Fossa) for ROS 2 Iron (though Humble is recommended).
*   **Internet Connection** : Required for downloading packages.
*   **Sudo Privileges** : Necessary for system-wide installations.

## 1. Set Up Locale

Ensure your locale is set correctly to `en_US.UTF-8`.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Add ROS 2 Repository

First, ensure you have `curl` and `software-properties-common` installed, then add the ROS 2 GPG key and the repository to your sources list.

```bash
sudo apt update && sudo apt install software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https ://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http ://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. Install ROS 2 Packages

Update your package list and then install either the `ros-humble-desktop` or `ros-iron-desktop` package. Humble is generally recommended for its LTS support.

```bash
sudo apt update
sudo apt upgrade

# Install ROS 2 Humble Desktop (Recommended)
sudo apt install ros-humble-desktop

# OR Install ROS 2 Iron Desktop (if preferred)
# sudo apt install ros-iron-desktop
```

## 4. Environment Setup

Source the ROS 2 setup script in your current shell and optionally add it to your `~/.bashrc` for automatic sourcing.

```bash
# Replace 'humble' with 'iron' if you installed ROS 2 Iron
source /opt/ros/humble/setup.bash

# Optional : Add to your .bashrc for automatic sourcing on new terminals
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 5. Install `rosdep`

`rosdep` is a tool for installing system dependencies for ROS packages.

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Verification

To verify your ROS 2 installation, you can open a new terminal and run :

```bash
ros2 daemon --version
```

You should see output similar to `ROS_VERSION=2 ROS_PYTHON_VERSION=3 ...` indicating ROS 2 is installed and recognized.
We will cover running demo examples in detail in the next sections.
