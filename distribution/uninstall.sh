#!/bin/bash
set -e

script_path=$(readlink -f "$0")
script_dir=$(dirname "$script_path")
state_dir="${XDG_CONFIG_HOME:-$HOME/.config}/robocup"
service_file="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user/robocup_game_assist.service"
workspace_root="${ROBOCUP_WORKSPACE_DIR:-$HOME/Workspace}"
install_root="${ROBOCUP_INSTALL_DIR:-}"

if [ -f "$script_dir/.project_name" ]; then
    project_name=$(cat "$script_dir/.project_name")
elif [ "$(basename "$script_dir")" = "distribution" ]; then
    project_name=$(basename "$(dirname "$script_dir")")
else
    project_name="robocup"
fi

state_file="$state_dir/${project_name}.install_path"

if [ -z "$install_root" ] && [ -f "$state_file" ]; then
    install_root=$(cat "$state_file")
fi

if [ -z "$install_root" ]; then
    install_root="$workspace_root/$project_name"
fi

if [ -x "$install_root/scripts/stop.sh" ]; then
    "$install_root/scripts/stop.sh"
fi

rm -rf "$install_root"
systemctl --user stop robocup_game_assist.service || true
systemctl --user disable robocup_game_assist.service || true
rm -f "$service_file"
systemctl --user daemon-reload
rm -f "$state_file"
