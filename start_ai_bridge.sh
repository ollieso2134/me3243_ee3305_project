#!/usr/bin/env bash
set -e
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

if [ -f "$PROJECT_DIR/.venv_ai/bin/activate" ]; then
  source "$PROJECT_DIR/.venv_ai/bin/activate"
elif [ -f "$HOME/me3243_ai_env/bin/activate" ]; then
  source "$HOME/me3243_ai_env/bin/activate"
fi

if [ -f "$PROJECT_DIR/.env" ]; then
  set -a
  source "$PROJECT_DIR/.env"
  set +a
fi

if [ -z "${GEMINI_API_KEY:-}" ]; then
  echo 'Set GEMINI_API_KEY in your shell or .env before starting the AI bridge.' >&2
  exit 1
fi
export PYTHONPATH="$PWD/src/me3243_voice_control:${PYTHONPATH:-}"
python "$PWD/src/me3243_voice_control/me3243_voice_control/ai_web_bridge.py"
