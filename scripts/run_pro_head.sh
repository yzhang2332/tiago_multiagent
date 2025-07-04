#!/bin/bash

ssh pal@tiago-pro-head-6c.local << 'EOF'
  source init-pal-env
  python3 /home/pal/multiagent_ws_yan/scripts/ros2_tcp_bridge.py
EOF
