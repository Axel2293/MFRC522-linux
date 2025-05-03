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

# Open tmux with one windows with 3 terminals
echo "Starting tmux session..."
tmux new-session -d -s cas
tmux rename-window 'Main'
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux send-keys 'source ./venv/bin/activate' C-m
tmux send-keys 'python3 ./control_access_system.py' C-m

tmux select-pane -t 1
tmux send-keys 'watch gpioinfo gpiochip2' C-m

tmux select-pane -t 2
tmux send-keys 'watch tail ./mfrc522.log' C-m

# Show all windows
tmux attach-session -t cas

# Close tmux session with Ctrl + b -> :kill-session or d

echo "Done."
