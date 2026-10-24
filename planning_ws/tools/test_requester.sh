#!/usr/bin/env bash
set -eo pipefail
planning_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${AJOUNICE_REQUESTER_TEST_ENV:-}" != 1 ]]; then
  exec env -i PATH=/usr/bin:/bin LANG=C.UTF-8 AJOUNICE_REQUESTER_TEST_ENV=1 bash --noprofile --norc "$0"
fi
source "$planning_root/tools/env.sh"
# Use the supported AW149 overlay for generated requester services.
source "$AJOUNICE_ROOT/scripts/workspace/source_aw149.sh"
export ROS_HOME="$(mktemp -d /tmp/ajounice-requester-ros-XXXXXX)"
export ROS_IP=127.0.0.1
ros_test_port="$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()')"
export ROS_MASTER_URI="http://127.0.0.1:$ros_test_port"
roscore -p "$ros_test_port" > "$ROS_HOME/master.log" 2>&1 &
ros_test_pid=$!
trap 'kill "$ros_test_pid" 2>/dev/null || true; wait "$ros_test_pid" 2>/dev/null || true; echo "Private ROS evidence: $ROS_HOME"' EXIT
python3 - <<'PY'
import os,time,xmlrpc.client
master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
for attempt in range(100):
    try:
        assert master.getPid('/test')[0] == 1
        break
    except Exception:
        time.sleep(.1)
else:
    raise SystemExit('private master startup failed')
PY
python3 "$AJOUNICE_ROOT/base_ws/src/planning_addons/competition_route_requester/test/test_gateway_ros.py"
