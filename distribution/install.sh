#!/bin/bash
set -e

script_path=$(readlink -f "$0")
script_dir=$(dirname "$script_path")

echo "Script path: $script_path"

if [ -d "$script_dir/utils" ] && [ -d "$script_dir/scripts" ]; then
    source_root="$script_dir"
elif [ "$(basename "$script_dir")" = "distribution" ] && [ -d "$(dirname "$script_dir")/utils" ] && [ -d "$(dirname "$script_dir")/scripts" ]; then
    source_root=$(dirname "$script_dir")
else
    echo "Unable to locate project root from: $script_path" >&2
    exit 1
fi

project_name_file="$source_root/.project_name"
if [ -f "$project_name_file" ]; then
    project_name=$(cat "$project_name_file")
else
    project_name=$(basename "$source_root")
fi

workspace_root="${ROBOCUP_WORKSPACE_DIR:-$HOME/Workspace}"
install_root="${ROBOCUP_INSTALL_DIR:-$workspace_root/$project_name}"
state_dir="${XDG_CONFIG_HOME:-$HOME/.config}/robocup"
state_file="$state_dir/${project_name}.install_path"

echo "Source root: $source_root"
echo "Project name: $project_name"
echo "Install root: $install_root"

rm -rf "$install_root"
mkdir -p "$install_root"
cp -r "$source_root"/* "$install_root"

mkdir -p "$state_dir"
printf '%s\n' "$install_root" > "$state_file"

bash "$install_root/utils/install_auto_start_assist.sh" "$install_root"
echo "Install success"
