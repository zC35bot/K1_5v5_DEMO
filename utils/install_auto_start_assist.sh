#!/bin/sh
set -e

script_path=$(readlink -f "$0")
utils_root=$(dirname "$script_path")
install_root="${1:-$(dirname "$utils_root")}"
service_template="$utils_root/service/robocup_game_assist.service"
service_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
service_file="$service_dir/robocup_game_assist.service"

echo "Script path: $script_path"
echo "Install root: $install_root"

systemctl --user stop robocup_game_assist.service || true
mkdir -p "$service_dir"
sed "s|__ROBOCUP_ROOT__|$install_root|g" "$service_template" > "$service_file"

systemctl --user daemon-reload
systemctl --user start robocup_game_assist.service
systemctl --user enable robocup_game_assist.service
