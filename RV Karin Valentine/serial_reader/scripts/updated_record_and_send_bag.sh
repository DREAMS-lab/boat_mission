#!/bin/bash
set -e  # exit on error

# --- CONFIG ---
# Paths will be consistent across systems, derived automatically

REMOTE_USER=${REMOTE_USER:-$USER}
REMOTE_HOSTNAME=$(hostname)
REMOTE_BASE="${HOME}/rosbags"

LOCAL_USER=${LOCAL_USER:-linuxlab}
LOCAL_HOST=${LOCAL_HOST:-192.168.1.10}
LOCAL_BASE="${HOME}/boat/boat_results/mission_bags"

# --- Step 0: Generate UTC timestamp on LOCAL machine (authoritative clock) ---
echo "Getting UTC timestamp from $LOCAL_USER@$LOCAL_HOST..."
BAG_DIR_BASE=$(ssh "$LOCAL_USER@$LOCAL_HOST" 'date -u +rosbag2_%Y_%m_%d_%H_%M_%S') || {
    echo "Failed to get timestamp from $LOCAL_HOST, using local time."
    BAG_DIR_BASE=$(date -u +rosbag2_%Y_%m_%d_%H_%M_%S)
}

REMOTE_BAG_DIR="${REMOTE_BASE}/${BAG_DIR_BASE}"
LOCAL_BAG_DIR="${LOCAL_BASE}/${BAG_DIR_BASE}"

echo "Recording bag will be saved to: $REMOTE_BAG_DIR"

# --- Step 1: Start recording ---
mkdir -p "$REMOTE_BASE"
nohup ros2 bag record -o "$REMOTE_BAG_DIR" -a > "$REMOTE_BASE/ros2_bag.log" 2>&1 &

echo "Recording started (log: $REMOTE_BASE/ros2_bag.log)"
echo "Press [Enter] once recording has completed to begin transfer..."
read -r

# --- Step 2: Transfer to local machine ---
echo "Transferring $REMOTE_BAG_DIR to $LOCAL_USER@$LOCAL_HOST:$LOCAL_BASE ..."
ssh "$LOCAL_USER@$LOCAL_HOST" "mkdir -p '$LOCAL_BASE'"
scp -r "$REMOTE_BAG_DIR" "$LOCAL_USER@$LOCAL_HOST:$LOCAL_BASE/"

echo "Transfer complete!"
echo "File saved to: $LOCAL_BAG_DIR"

