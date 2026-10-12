#!/usr/bin/env bash
set -e
cd "$HOME/me3243_ee3305_project"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "$HOME/me3243_ai_env/bin/activate"
if [ -z "${GEMINI_API_KEY:-}" ]; then
  echo 'Set GEMINI_API_KEY in your shell before starting the AI bridge.' >&2
  exit 1
fi
export PYTHONPATH="$PWD/src/me3243_voice_control:${PYTHONPATH:-}"
python "$PWD/src/me3243_voice_control/me3243_voice_control/ai_web_bridge.py"
