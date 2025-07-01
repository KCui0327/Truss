#!/bin/bash

# This script sets up the environment for the Truss project.

which python3 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "Python3 is not installed. Please install Python3 to continue."
    exit 1
fi

# Check if pip is installed
which pip3 >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "pip3 is not installed. Please install pip3 to continue."
    exit 1
fi

# Start Virtual Environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    if [ $? -ne 0 ]; then
        echo "Failed to create virtual environment. Please check your Python installation."
        exit 1
    fi
fi 

# # Install required Python packages
# pip3 install -r requirements.txt >/dev/null 2>&1
# if [ $? -ne 0 ]; then
#     echo "Failed to install required Python packages. Please check your internet connection and try again."
#     exit 1
# fi

echo "Environment setup complete. You can now run the Truss project."
echo "To activate the virtual environment, run: source venv/bin/activate"
echo "To deactivate the virtual environment, run: deactivate"
