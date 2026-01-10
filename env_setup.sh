#!/bin/bash
set -e

# This script sets up the environment for the Truss project.

# Check for python3
which python3 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Python3 is not installed. Please install Python3 to continue."
    exit 1
fi

# Check for pip3
which pip3 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "pip3 is not installed. Please install pip3 to continue."
    exit 1
fi

# Create virtual environment if missing
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Upgrade pip (important for reliability)
pip install --upgrade pip

# Install dependencies if requirements.txt exists
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
else
    echo "No requirements.txt found — skipping dependency install."
fi

echo "Environment setup complete."
echo "Virtual environment is active."
