#!/usr/bin/env bash
set -euo pipefail

rostopic list
echo "Checking key topics..."
rostopic hz /localization/odom --window=20