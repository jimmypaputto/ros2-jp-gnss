#!/bin/bash
# Starts the gnsshat web visualization (port 5000) in ROS 2 mode. The node_id is read from the
# node's config, so the visualization follows /gnss/<node_id>/navigation without extra settings.
set -e
config=${1:-/config/gnss_config.yaml}
node_id=$(python3 -c '
import sys, yaml
try:
    with open(sys.argv[1]) as f:
        print((yaml.safe_load(f) or {}).get("node_id") or "")
except OSError:
    pass
' "${config}")

cd /opt/jp_gnss_hat/visualization
exec venv/bin/python app.py ros2 ${node_id:+"${node_id}"}
