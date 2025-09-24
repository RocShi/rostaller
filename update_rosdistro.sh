#!/bin/bash
#
# @File          : update_submodule.sh
# @Description   : Script to update the rosdistro submodule to latest version
# @Author        : ShiPeng
# @Email         : RocShi@outlook.com
# @License       : MIT License
#

set -e

echo "Updating rosdistro submodule to latest version..."

# check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "Error: Not in a git repository. Please run this script from the project root."
    exit 1
fi

# check if submodule exists
if [ ! -f ".gitmodules" ] || ! grep -q "rosdistro" .gitmodules; then
    echo "Error: rosdistro submodule not found. Please add it first:"
    echo "git submodule add --depth 1 https://github.com/ros/rosdistro.git rosdistro"
    exit 1
fi

# update submodule to latest
echo "Fetching latest changes from rosdistro repository..."
git submodule update --remote rosdistro

# show what changed
echo "Updated rosdistro submodule:"
cd rosdistro
echo "Current commit: $(git rev-parse HEAD)"
echo "Latest changes:"
git log --oneline -5
cd ..

# add changes to git
echo "Adding submodule changes to git..."
git add rosdistro

echo "rosdistro submodule updated successfully!"
echo "To commit the changes, run: git commit -m 'Update rosdistro submodule'"
