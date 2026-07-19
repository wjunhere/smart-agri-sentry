#!/usr/bin/env bash
# Install systemd autostart for the sentry gateway layer (bridge :8765 +
# web_remote :5000 + weather + LLM). Run once over SSH; afterwards the car
# boots straight into "frontend reachable" state, no SSH needed.
set -euo pipefail

WS_DIR="${SENTRY_WS:-/home/sunrise/dev_ws}"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
SERVICE_NAME="sentry-bridge.service"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}"
RUN_USER="${SENTRY_USER:-sunrise}"

if [ ! -f "${WS_DIR}/install/setup.bash" ]; then
  echo "ERROR: ${WS_DIR}/install/setup.bash missing; build the workspace first." >&2
  exit 1
fi

sudo tee "${SERVICE_FILE}" >/dev/null <<EOF
[Unit]
Description=Sentry gateway layer (miniprogram bridge + web_remote + weather + LLM)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Environment=SENTRY_LLM_API_KEY=${SENTRY_LLM_API_KEY:-}
Environment=DEEPSEEK_API_KEY=${DEEPSEEK_API_KEY:-}
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && source ${WS_DIR}/install/setup.bash && exec ros2 launch sentry_bringup miniprogram_bridge.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"
echo "Installed and started ${SERVICE_NAME}."
echo "Check: systemctl status ${SERVICE_NAME}"
echo "Logs:  journalctl -u ${SERVICE_NAME} -f"
