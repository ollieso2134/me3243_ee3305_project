# ME3243 / EE3305 — Project Setup and Launch Guide

This repository contains a ROS 2 robot simulation and navigation project with a web interface and AI voice-control components. This guide is intended for a **first-time user** downloading the project from GitHub as a ZIP file. No Git commands are required.

## 1. System requirements

- **Operating system:** Ubuntu 24.04 LTS (native), or Windows with **WSL 2 + Ubuntu 24.04**
- **ROS:** ROS 2 **Jazzy** (desktop installation recommended for graphical simulation)
- **Build tools:** `colcon`, `rosdep`, and standard Ubuntu development tools
- **Python:** Python dependencies listed in `requirements_ai.txt`
- **AI features:** A valid **Google Gemini API key** and an internet connection
- **Graphics/audio:** A working Linux graphical environment for simulator windows; voice input/output also depends on microphone and audio access

> **Windows users:** Windows 11 with WSLg is recommended for displaying Linux GUI applications. A native Ubuntu 24.04 installation is also supported by the general ROS workflow, but the project has been developed/tested in the Ubuntu/WSL workflow described here.

## 2. Download and extract the project

1. Open the [project repository](https://github.com/ollieso2134/me3243_ee3305_project) (GitHub access is required because the repository is private).
2. Click **Code → Download ZIP**.
3. Save the ZIP file to your computer.

**Windows / PowerShell:** Open PowerShell and start Ubuntu:

```powershell
wsl -d Ubuntu-24.04
```

If Ubuntu 24.04 is not yet installed in WSL, open **PowerShell as Administrator** and run `wsl --install -d Ubuntu-24.04`, then complete the Ubuntu first-time setup (restart Windows if prompted).

**Ubuntu terminal:** Install the unzip utility if needed, then extract the downloaded ZIP into your Linux home directory:

```bash
sudo apt update
sudo apt install -y unzip
cd "/mnt/c/Users/YOUR_WINDOWS_USERNAME/Downloads"
unzip me3243_ee3305_project-main.zip -d ~
cd ~/me3243_ee3305_project-main
```

Replace `YOUR_WINDOWS_USERNAME` with your Windows user folder name. The ZIP filename or extracted directory name may vary; adjust the commands if yours differs. For **native Ubuntu**, use the actual path to your downloaded ZIP instead of the `/mnt/c/...` Windows path.

Verify that you are in the project root (the directory containing `src/` and `start_all_with_ai.sh`):

```bash
ls
```

## 3. Install ROS 2 and build dependencies (one-time setup)

If ROS 2 Jazzy is **not installed**, follow the official [ROS 2 Jazzy installation instructions for Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), including repository setup, and install the **desktop** variant (`ros-jazzy-desktop`).

Then, in Ubuntu, install the workspace tools:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep \
  python3-pip python3-venv build-essential
```

Initialize the ROS dependency manager **once** (if `rosdep init` says it is already initialized, continue with `rosdep update`):

```bash
sudo rosdep init
rosdep update
```

Confirm that ROS 2 is available:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

## 4. Install project dependencies (one-time setup)

Return to the extracted project folder:

```bash
cd ~/me3243_ee3305_project-main
source /opt/ros/jazzy/setup.bash
```

Install dependencies declared by the ROS packages:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

If Conda is currently active, run `conda deactivate` first. Create a Python virtual environment that can also use the system-installed ROS 2 Python packages:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements_ai.txt
```

> This guide uses a system-Python virtual environment for compatibility with ROS 2 Jazzy installed from Ubuntu packages. A fresh ZIP download does **not** contain a ready-made Python environment.

## 5. Build the project (first run, and after source-code changes)

Run these commands from the project root:

```bash
cd ~/me3243_ee3305_project-main
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

A successful build should report **5 packages finished**. The expected ROS packages are:

```text
ee3305_bringup
ee3305_cpp
ee3305_map_loader
ee3305_py
me3243_voice_control
```

You can verify package discovery using:

```bash
ros2 pkg list | grep -E 'ee3305|me3243'
```

## 6. Configure the Gemini API key

The AI features require your **own** Gemini API key. Create or access one in [Google AI Studio](https://aistudio.google.com/apikey).

In the **same Ubuntu terminal** you will use to launch the project, enter:

```bash
read -rs -p 'Enter Gemini API key: ' GEMINI_API_KEY; echo
export GEMINI_API_KEY
```

Verify it is set (without showing the key):

```bash
[ -n "$GEMINI_API_KEY" ] && echo "Gemini API key is set"
```

The key is only set for the current terminal session and its child processes. **Do not put the real API key inside source files or upload it to GitHub.**

## 7. Launch the complete project

From the project root, after building and setting the API key:

```bash
bash ./start_all_with_ai.sh
```

This is the repository's combined startup script. Check the terminal for the simulation status, the web-interface address (if printed), and any additional instructions. Use **Ctrl+C** to stop the foreground process.

## 8. Quick launch on subsequent runs

**PowerShell (Windows only):**

```powershell
wsl -d Ubuntu-24.04
```

**Ubuntu terminal:**

```bash
cd ~/me3243_ee3305_project-main
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
read -rs -p 'Enter Gemini API key: ' GEMINI_API_KEY; echo
export GEMINI_API_KEY
bash ./start_all_with_ai.sh
```

You do **not** need to rebuild every time, unless you change the code or dependencies.

## 9. Troubleshooting

| Problem | What to check |
| --- | --- |
| `wsl` or Ubuntu is unavailable | Install WSL 2 with Ubuntu 24.04, then restart if requested. |
| `ros2: command not found` | Install ROS 2 Jazzy and run `source /opt/ros/jazzy/setup.bash`. |
| `colcon: command not found` | Install `python3-colcon-common-extensions`. |
| `rosdep init` reports already initialized | This is expected; run `rosdep update` instead. |
| `install/setup.bash: No such file` | Build the workspace first from the project root. |
| Python module import error | Verify `.venv` is activated and `requirements_ai.txt` was installed. |
| ROS Python import problems in a virtual environment | Make sure it was created with `--system-site-packages` and the ROS setup file was sourced. |
| Script not found | Ensure the terminal is in the folder containing `start_all_with_ai.sh`. |
| Gemini authentication error | Re-enter a valid Gemini API key in the same terminal. |
| Simulation GUI or microphone unavailable | Verify your Ubuntu/WSL graphics, audio, and microphone configuration. |

## 10. Project files

```text
me3243_ee3305_project-main/
├── assets/
├── docs/
├── src/
│   ├── ee3305_bringup/
│   ├── ee3305_cpp/
│   ├── ee3305_map_loader/
│   ├── ee3305_py/
│   └── me3243_voice_control/
├── AI_voice.md
├── README.md
├── documentation.md
├── requirements_ai.txt
├── start_ai_bridge.sh
└── start_all_with_ai.sh
```

For further technical information, see [`AI_voice.md`](AI_voice.md), [`documentation.md`](documentation.md), and [`docs/`](docs/).

---

**Launch command:** `bash ./start_all_with_ai.sh`

**Note:** Some simulator, audio, and external service dependencies may require additional machine-specific configuration. Consult `AI_voice.md` and `documentation.md` if the launcher reports missing components.
