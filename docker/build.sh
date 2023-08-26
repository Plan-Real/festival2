#!/bin/bash
set -e

# Choose build
# --isolated-install : install each package into its own install space
# --merge-install : merge all packages into one install space

# Set the default build type
BUILD_TYPE=RelWithDebInfo
colcon build \
        --symlink-install \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
        -Wall -Wextra -Wpedantic
