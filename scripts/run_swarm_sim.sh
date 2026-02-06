#!/usr/bin/env bash
set -e

# 简易仿真启动脚本：自动启动 roscore 与 swarm_sim.launch
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 加载环境
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_DIR}/devel/setup.bash"
fi

# 启动 roscore（如果未启动）
if ! rostopic list >/dev/null 2>&1; then
  echo "[Swarm] 启动 roscore..."
  roscore >/tmp/roscore_swarm.log 2>&1 &
  sleep 2
fi

# 启动 swarm_sim.launch
echo "[Swarm] 启动 swarm_sim.launch..."
roslaunch sunray_swarm swarm_sim.launch "$@"
