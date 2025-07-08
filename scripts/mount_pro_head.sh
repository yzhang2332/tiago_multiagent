#!/bin/bash

# Variables
REMOTE_USER="pal"
REMOTE_HOST="tiago-pro-head-6c.local"
REMOTE_PATH="/home/pal/multiagent_ws_yan"
LOCAL_PATH="$HOME/tiago_ws/src/multiagent/scripts/tiago_pro_head_mount"
PASSWORD="pal"

# Optionally unmount existing mount
if mount | grep "$LOCAL_PATH" > /dev/null; then
    echo "Unmounting previous mount at $LOCAL_PATH"
    fusermount -u "$LOCAL_PATH"
fi

# Run sshfs (will prompt for password)
sshfs ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_PATH} "$LOCAL_PATH" \
  -o reconnect,StrictHostKeyChecking=no,ServerAliveInterval=15,ServerAliveCountMax=3

# Check result
if [ $? -eq 0 ]; then
    echo "Mounted successfully at $LOCAL_PATH"
else
    echo "Failed to mount"
fi