#!/usr/bin/env bash
# Launch friend navigation/UI and integrated AI bridge from a single terminal.
set -e
cd "$HOME/me3243_ee3305_project"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "$HOME/me3243_ai_env/bin/activate"
if [ -z "${GEMINI_API_KEY:-}" ]; then
  echo 'GEMINI_API_KEY is not set. Export it before starting this launcher.' >&2
  exit 1
fi
export PYTHONPATH="$PWD/src/me3243_voice_control:${PYTHONPATH:-}"
ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False &
LAUNCH_PID=$!
cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
python "$PWD/src/me3243_voice_control/me3243_voice_control/ai_web_bridge.py"
