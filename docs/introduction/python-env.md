---
sidebar_position : 2
---

# Python Environment Setup

This section guides you through setting up a dedicated Python environment (using `venv` or `conda`) for your ROS 2 projects. Maintaining isolated environments is crucial for managing dependencies and avoiding conflicts between different projects. ROS 2 is built to work with Python 3, and for this book, we will primarily use Python 3.10+.

## Prerequisites

*   **Python 3.10+** : Ensure Python 3.10 or a newer version is installed on your system.
*   **Internet Connection** : Required for downloading packages.

## 1. Verify Python Installation

First, check if Python 3 is installed and its version.

```bash
python3 --version
```

If Python 3.10+ is not installed, you might need to install it :

```bash
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip
```

## 2. Create a Virtual Environment

Navigate to your ROS 2 workspace or project directory (e.g., `ros2_ws`) and create a new virtual environment.

```bash
# Navigate to your workspace (if not already there)
# cd ~/ros2_ws

# Create a virtual environment named 'venv'
python3.10 -m venv venv
```

This command creates a directory named `venv` inside your current folder, containing a Python interpreter and its own set of `pip` packages.

## 3. Activate the Virtual Environment

You must activate the virtual environment in each terminal session where you intend to work on your Python-based ROS 2 projects.

```bash
source venv/bin/activate
```

Once activated, your terminal prompt will typically change to indicate that you are operating within the virtual environment (e.g., `(venv) user@hostname :~`).

## 4. Install Project-Specific Python Packages

With the virtual environment active, you can install any Python packages required for your project without affecting your system's global Python installation.

```bash
(venv) pip install --upgrade pip
(venv) pip install some-python-package-for-ros
```

## 5. Deactivate the Virtual Environment

When you are done working on your project, you can deactivate the virtual environment.

```bash
deactivate
```

This will return your shell to its global Python environment.

## Verification

To verify your Python environment setup :

1.  **Activate your virtual environment** :
    ```bash
    source venv/bin/activate
    ```
2.  **Check Python version** :
    ```bash
    python --version
    ```
    Ensure it reports `Python 3.10.x` or newer.

3.  **List installed packages** :
    ```bash
    pip list
    ```
    This should show only `pip`, `setuptools`, and `wheel` initially, confirming a clean environment.

4.  **Deactivate the environment** :
    ```bash
    deactivate
    ```
