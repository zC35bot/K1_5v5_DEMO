#!/bin/bash
set -e

cd "$(dirname "$0")"
cd ..
WORKSPACE_ROOT=$(pwd)

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFLICT]"
sudo killall -9 booster-video-stream || true
./scripts/stop.sh || true
sudo jetson_clocks
sudo systemctl mask apt-daily.timer apt-daily-upgrade.timer
sudo systemctl mask unattended-upgrades.service
sudo rm -f /var/lib/systemd/timers/stamp-apt-daily.timer
sudo pkill -9 update_manager || true
sudo pkill -9 python3 || true
systemctl --user disable robocup_game_assist.service || true

echo "[START ROBOCUP NODES]"
source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[START VISION]"
nohup ros2 launch vision launch.py vision_config_path:=/opt/booster save_data:=true > vision.log 2>&1 &

echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &

echo "[START BRAIN FOREGROUND]"
ros2 launch brain launch.py vision_config_path:=/opt/booster "$@"
