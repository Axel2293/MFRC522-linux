#!/bin/bash

# Create a venv Virtual environment if it doesn't exist
if [ ! -d "./venv" ]; then
    echo "Creating venv..."
    mkdir ./venv
    python3 -m venv ./venv

    # Install requirements
    echo "Installing requirements..."
    python3 -m pip install --upgrade pip
    python3 -m pip install --upgrade -r ./requirements.txt
else
    echo "Venv already exists, skipping creation."
fi

# Activate venv (will always run on linux)
echo "Activating venv..."
source ./venv/bin/activate

# Start main application
echo "Starting main application..."
python3 ./control_access_system.py

# Deactivate venv
echo "Deactivating venv..."
deactivate
echo "Done."
