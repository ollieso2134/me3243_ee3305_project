# ME3243 — AI Voice Control

**Branch:** `Ai_control`
**Platform:** Ubuntu 24.04 on WSL2, ROS 2 Jazzy, TurtleBot3 Burger in Gazebo
**Main interface:** http://localhost:8080

This document explains how to install, run, test, and maintain the **browser-based AI Voice Control extension** to the existing EE3305/ME3243 navigation project. It supplements the main `README.md` and the documentation in `docs/`; those documents remain the authority for the original Gazebo simulation, mapping, Dijkstra planner, and Pure Pursuit controller.

## 1. What the extension does

Users can select **AI VOICE CONTROL** in the existing web GUI, record speech through the browser microphone, or type a request. Faster-Whisper transcribes audio; the command backend and Gemini interpret requests; the navigation manager dispatches room goals; and the waypoint queue processes multiple rooms sequentially. Edge TTS generates speech feedback, while messages also appear in the web GUI chat.

The AI extension does **not** replace the original path planner or velocity controller. Occasional wall collisions and path-tracking deviations were observed in the shared navigation system; these require separate investigation and are not fixed by AI arrival tolerance.

## 2. System architecture

```text
Browser microphone or typed request
             |
             v
Original web GUI (web_gui.py)
             |
      AI text/audio topic
             |
             v
       ai_web_bridge.py
             |
      +------+------------------+
      |                         |
Audio -> Faster-Whisper      Text input
      |                         |
      +------ Command processing+
             |  command_backend.py / gemini_parser.py
             |  intent_parser.py (multi-room)
             v
     waypoint_queue.py
             |
     navigation_manager.py
             |
         /room_goal
             |
  Original behavior -> Dijkstra -> /path
             |
      Pure Pursuit -> /cmd_vel
             |
       ros_gz_bridge -> Gazebo
```

The bridge also publishes `ai_chat_event` to the GUI and uses Edge TTS for speech responses. Browser audio is uploaded to the web backend, so no WSL microphone passthrough is needed.

### ROS 2 topics

| Topic | Interface | Purpose |
|---|---|---|
| `ai_text_command` | `String`, bridge input | GUI text |
| `ai_audio_path` | `String`, bridge input | Uploaded audio location |
| `ai_stop` | `Bool`, bridge input | Dashboard stop |
| `ai_chat_event` | `String`, bridge output | Chat/status events (JSON text) |
| `ai_ready` | `Bool`, bridge output | Bridge heartbeat |
| `/control_mode` | `String`, navigation input | `voice` or `manual` mode |
| `/odom` | `Odometry`, navigation input | Robot coordinates |
| `/room_goal` | `Int32`, navigation output | Room 1–6 request |
| `/stop` | `Bool`, navigation output | Stop/cancel |

## 3. Prerequisites

Install and verify the **original navigation project** first. It should launch Gazebo and allow MANUAL room navigation before testing the AI extension.

Required for the tested configuration:

- Ubuntu **24.04** (WSL2 was used for development).
- **ROS 2 Jazzy**, Gazebo, the TurtleBot3 simulation, `ros_gz_bridge`, `colcon`, and the original project's ROS dependencies.
- Python 3.12 with `venv` support and internet access for Gemini and Edge TTS.
- A browser with microphone access for voice commands.
- A Google AI Studio API key with permission to use the selected Gemini model.

For ROS 2/Gazebo installation, follow the **main `README.md` and `docs/`**; Python packages alone are not sufficient to install the base project.

### Expected file paths

The current launcher assumes exactly:

```text
~/me3243_ee3305_project/
    README.md
    start_all_with_ai.sh
    src/ee3305_bringup/
    src/ee3305_py/
    src/me3243_voice_control/me3243_voice_control/
~/me3243_ai_env/
```

If your checkout or virtual environment uses different paths, adjust `start_all_with_ai.sh`.

## 4. Obtain the AI branch

Replace `<REPOSITORY_URL>` with the actual team repository URL; this URL was not included in the shared source material.

```bash
git clone --branch Ai_control <REPOSITORY_URL> ~/me3243_ee3305_project
cd ~/me3243_ee3305_project
```

If the repository is already checked out:

```bash
cd ~/me3243_ee3305_project
git fetch origin
git switch Ai_control
```

Check that `src/me3243_voice_control/` and `start_all_with_ai.sh` exist.

## 5. Build the ROS 2 workspace

After installing the original project's prerequisites:

```bash
cd ~/me3243_ee3305_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If dependencies are missing, resolve them using the original project setup guide before continuing.

## 6. Set up AI Python dependencies

Create the virtual environment at the fixed location expected by the launcher:

```bash
python3 -m venv --system-site-packages ~/me3243_ai_env
source ~/me3243_ai_env/bin/activate
python -m pip install --upgrade pip
cd ~/me3243_ee3305_project
python -m pip install -r requirements_ai.txt
```

`--system-site-packages` is useful for importing ROS 2 Python packages installed by Ubuntu. This setup matches the original working ROS 2 Jazzy/Python environment but should still be tested on new machines.

Versions observed in the working virtual environment:

| Package | Version |
|---|---|
| `google-genai` | `2.29.0` |
| `faster-whisper` | `1.2.1` |
| `edge-tts` | `7.2.8` |
| `ctranslate2` | `4.8.2` |
| `av` | `18.1.0` |
| `numpy` | `1.26.4` |
| `requests` | `2.31.0` |
| `websockets` | `16.1.1` |

These are observed core versions, **not** a comprehensive lockfile. Additional transitive dependencies are resolved by pip. The project's ROS/Gazebo dependencies are installed separately.

## 7. Google AI Studio: API key and model

1. Open **https://aistudio.google.com/apikey**.
2. Sign in and create/select an eligible Google Cloud project.
3. Generate a Gemini API key.
4. Verify the available models, pricing, and free-tier eligibility at **https://ai.google.dev/gemini-api/docs/pricing**.
5. Never commit your real key to GitHub or paste it into a shared document.

In the **same Ubuntu terminal** that will run the launcher:

```bash
export GEMINI_API_KEY='PASTE_YOUR_KEY_HERE'
```

The current `gemini_parser.py` reads `GEMINI_MODEL` from the environment, with the **code default** `gemini-3.5-flash-lite`. An identifier appearing in source code does not guarantee that a new user can access that model or use it free. Check Google AI Studio and override it with a supported model if necessary:

```bash
export GEMINI_MODEL='MODEL_ID_AVAILABLE_TO_YOUR_PROJECT'
```

Free-tier access depends on the model, account/project eligibility, quotas, and current Google pricing. Do not assume unlimited free access.

## 8. Start the simulation and AI bridge

```bash
cd ~/me3243_ee3305_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source ~/me3243_ai_env/bin/activate
export GEMINI_API_KEY='PASTE_YOUR_KEY_HERE'
bash ./start_all_with_ai.sh
```

The provided launcher:

1. Changes directory to `~/me3243_ee3305_project`.
2. Sources `/opt/ros/jazzy/setup.bash`, `install/setup.bash`, and `~/me3243_ai_env/bin/activate`.
3. Checks that `GEMINI_API_KEY` is present.
4. Adds `src/me3243_voice_control` to `PYTHONPATH`.
5. Starts `ros2 launch ee3305_bringup run.launch.py cpp:=False headless:=False libgl:=False` in the background.
6. Runs `src/me3243_voice_control/me3243_voice_control/ai_web_bridge.py` in the foreground.
7. Attempts to stop the background launch process on exit.

Leave this terminal open while using the simulation. Initial Whisper model loading/transcription can take longer than subsequent commands. To shut down, stop active navigation in the GUI, press **Ctrl+C** in the launch terminal, and wait for processes to exit before restarting.

## 9. Using the web GUI: typing and speaking to the AI

Open **http://localhost:8080** in the web browser on the computer running the simulation. Allow time for Gazebo, the navigation nodes, and the AI bridge to finish starting.

### 9.1 Select AI Voice Control mode

1. Open the dashboard at **http://localhost:8080**.
2. Select **AI VOICE CONTROL** instead of **MANUAL**. This makes the AI chat controls available and enables the AI navigation manager to accept room commands.
3. Check that the simulation is running and that the robot is not currently executing another mission.
4. You can now interact with the assistant using either the **microphone** or the **text input** in the AI panel.

> **Important:** The current implementation captures voice using the **web browser**. You do **not** need to give Ubuntu/WSL2 direct access to a microphone or launch the old standalone `voice_node.py`.

### 9.2 Speak to the AI using your microphone (step by step)

1. Connect a working microphone to the computer (a laptop's built-in microphone or a headset is suitable).
2. In the web GUI, select **AI VOICE CONTROL** and locate the **microphone button beneath the chat history**.
3. **Click the microphone button** to start recording. If your browser requests microphone permission for `localhost:8080`, choose **Allow**.
4. Speak your command clearly. For a first test, say: **“Go to room two.”**
5. **Stop speaking**. In the current web interface, approximately **1.2 seconds of silence** automatically ends the recording and submits it. If the interface offers a recording stop control, you can use it as well.
6. Wait for the browser to upload the recording and for **Faster-Whisper** to transcribe it. The chat may display a transcribing/processing status while this happens; the first request can be slower because the model is loaded on demand.
7. Check the **chat history** for the recognised text and the AI's response, such as **“Navigating to room 2.”** If Whisper misheard the room number, stop the mission and retry clearly or use text input.
8. Watch the robot in **Gazebo** and the route on the dashboard. The system dispatches the room goal to the existing Dijkstra/Pure Pursuit navigation stack; Gemini does not drive the robot's wheels directly.
9. When the robot reaches the requested room within the configured arrival tolerance, the AI should report arrival in the chat and generate spoken feedback.

**Where the audio goes:** Browser microphone → web GUI audio upload → `ai_web_bridge.py` → Faster-Whisper speech-to-text → command interpretation → ROS2 room goal. **Edge TTS generates the reply on the Ubuntu/WSL side**, so audible playback depends on the audio output configured for that environment; the chat remains a readable record even if speakers are silent.

**Tips for reliable recording:** Use a quiet environment, speak one command clearly, avoid very long pauses within a sentence (the silence detector may end the recording), and wait for a response before issuing another navigation command. Internet access is needed for Gemini interpretation and Edge TTS; Whisper transcription runs locally after its model is available.

### 9.3 Type a command instead

If a microphone is unavailable, permission is denied, or speech is not recognised correctly:

1. Keep **AI VOICE CONTROL** selected.
2. Click the **text input** beneath the chat.
3. Type **`Go to room 2`** and submit it using the interface's send control.
4. Read the AI response in the chat and verify the robot starts toward the correct room.

Typed commands use the same command and navigation backend, but **skip browser audio recording and Whisper transcription**. This makes text input a useful diagnostic if voice input fails.

### 9.4 Speak a multi-room command

You can request several rooms in one instruction. For example, say:

**“Go to room two, then room six, then room four.”**

The AI parses the requested room order and the waypoint queue dispatches **one destination at a time**. It should confirm arrival at each room before starting the next, then report completion after the final destination. The room-order test has been observed working; still supervise Gazebo because physical path tracking is not guaranteed collision-free.

### 9.5 Stop or cancel an active mission

Say **“Stop navigation”** or use the dashboard's **Stop** control. The AI bridge is designed to clear pending destinations when a stop request is handled. Since final emergency-stop regression testing was still pending, **verify that both the robot and queue actually stop** before treating this as a reliable safety mechanism. If the robot is heading toward a wall, use the dashboard stop control rather than waiting for speech transcription.

### 9.6 Example commands

| Intent | Example spoken or typed command | Expected behaviour |
|---|---|---|
| One destination | `Go to room 2` | Dispatch Room 2 |
| Another destination | `Navigate to room 4` | Dispatch Room 4 |
| Two destinations | `Go to room 2, then room 6` | Visit Room 2 before Room 6 |
| Three destinations | `Go to room 2, then room 6, then room 4` | Process rooms sequentially |
| Stop | `Stop navigation` | Request navigation cancellation and clear queued rooms |

Only rooms **1–6** are supported by the current waypoint definitions. Exact wording accepted depends on the command parser.

### 9.7 If the microphone does not work

- **No browser permission prompt / access denied:** Open the site permissions for `localhost:8080`, allow microphone access, and retry. Also check operating-system microphone privacy settings.
- **Recording ends too early:** Speak without long pauses; the silence detector submits the recording after roughly 1.2 seconds of silence.
- **Chat says “Transcribing” but takes a long time:** The first transcription may download/load the Whisper model. Inspect the launch terminal for errors and verify adequate memory and internet access for the initial model download.
- **Wrong room was recognised:** Read the transcribed user message in the chat. Retry clearly or use the text input to isolate microphone/transcription problems.
- **Text works but audio does not:** Focus troubleshooting on browser permission, recording, upload, and Whisper rather than the Dijkstra or Pure Pursuit code.
- **Text response appears but no spoken sound:** Verify the Ubuntu/WSL audio output used by Edge TTS playback. The chat display and speech output are separate feedback channels.
- **AI refuses to navigate:** Confirm **AI VOICE CONTROL** is selected and that the existing navigation manager is not already busy with another room.

## 10. Multi-room queue and arrival detection

`waypoint_queue.py` maintains a list of pending rooms. It starts the first destination, waits while `NavigationManager.active` is true, and advances only once `NavigationManager.status` becomes `succeeded`. On failure/cancellation, pending destinations are cleared.

Room coordinates in `navigation_manager.py`:

| Room | X (m) | Y (m) |
|---|---:|---:|
| 1 | -4.20 | 3.50 |
| 2 | -0.50 | 3.20 |
| 3 | 3.20 | 3.20 |
| 4 | 6.80 | 3.20 |
| 5 | 7.80 | -1.50 |
| 6 | -4.30 | -1.00 |

**Arrival-tolerance fix:** The original behavior node uses `dist < 0.15` metres, while the AI manager initially used `0.10`. This caused missing completion announcements and stalled waypoint queues even when the robot had physically reached the destination. The tested fix is:

```python
self.arrival_tolerance = 0.15
```

Verify that **0.15** is present in the final committed file. This synchronises completion criteria; it does **not** prevent collisions or improve steering accuracy.

## 11. Python module guide

| File | Responsibility |
|---|---|
| `ai_web_bridge.py` | Current browser-to-ROS2 AI entry point; command queue, audio transcription, status events and spoken feedback |
| `command_backend.py` | Forwards a command to Gemini interpretation |
| `gemini_parser.py` | Gemini API/model configuration and structured command parsing |
| `intent_parser.py` | Extracts multi-room requests |
| `navigation_manager.py` | Publishes room goals; tracks mode, odometry and arrival; publishes stop |
| `waypoint_queue.py` | Sequential room dispatch, queue state and cancellation |
| `text_to_speech.py` | Edge TTS responses |
| `speech_to_text.py` | Additional speech-to-text support |
| `whisper_worker.py` | Additional Whisper-processing support |
| `emergency_stop.py` | Stop-related helper module |
| `voice_node.py` | Earlier standalone voice workflow; **not launched** by `start_all_with_ai.sh` |
| `test_navigation.py` | Navigation test utility |
| `navigation_manager_nav2_backup.py` | Backup/legacy module, not the current manager |

`__pycache__`/`.pyc` are generated Python cache files. `ai_web_bridge.py:Zone.Identifier` is Windows metadata and not an executable module.

The bridge shown in the shared source uses worker, navigation-monitor and speech threads. It directly initialises Faster-Whisper's **small, CPU/int8** model for browser audio. The executor configuration should be checked against the final committed file because both multi-threaded and single-threaded variants were trialled during debugging.

## 12. Final validation checklist

- [ ] Base project launches and MANUAL navigation works.
- [ ] AI VOICE CONTROL mode enables browser inputs.
- [ ] Text-based command is interpreted and starts single-room navigation.
- [ ] Browser audio is transcribed correctly.
- [ ] The robot reaches a room and announces arrival.
- [ ] A multi-room request visits destinations in the correct order.
- [ ] A final completion announcement occurs after the last room.
- [ ] A stop command during a multi-room mission stops the robot and clears remaining rooms.
- [ ] A new command can be started safely after a stop.
- [ ] Extended navigation is assessed for collisions, path deviations, and unexpected movement.

**Observed:** Single-room voice requests and ordered multi-room visits have been reported working after the tolerance adjustment. Emergency-stop regression testing and long-run collision-free navigation still require final validation.

## 13. Troubleshooting

### `GEMINI_API_KEY is not set`

Set the variable in the **same terminal** where the launcher runs:

```bash
export GEMINI_API_KEY='YOUR_API_KEY'
bash ./start_all_with_ai.sh
```

### Gemini reports an invalid/unavailable model

The code default is not a guarantee of API availability. Verify model access in Google AI Studio; optionally override `GEMINI_MODEL` with an available model ID. Check API quota and internet access.

### `http://localhost:8080` does not open

Check that the startup terminal is still running and that no previous web GUI process is occupying port 8080. Inspect the ROS launch output. A basic diagnostic is:

```bash
curl -s http://localhost:8080/api/status
```

### Microphone does not work

Grant browser microphone permission. Try a typed command to determine whether the problem is microphone recording/upload or the AI/navigation backend. The WSL shell does not need its own microphone device for the current browser workflow.

### The robot arrives, but no completion announcement

Check the AI `arrival_tolerance` is **0.15 m** and that `/odom` is updating. The queue needs `status == 'succeeded'` before moving to the next room.

### The robot hits a wall or deviates from the GUI path

Compare the same room route using MANUAL mode **while leaving the AI bridge running**. If both modes behave poorly, investigate the shared navigation stack: Dijkstra wall clearance and map constraints, Pure Pursuit speed/lookahead, and control update timing. The AI arrival-tolerance adjustment does not fix path tracking.

### Verify simulator real-time performance

The observed Gazebo world was named `default`:

```bash
timeout 5s gz topic -e -t /world/default/stats
```

A `real_time_factor` near `1.0` suggests the simulation is running at approximately real-time speed; it does not prove every ROS callback is timely.

### Verify reported robot position

```bash
curl -s http://localhost:8080/api/status | python -m json.tool
```

The GUI position matched Gazebo `/odom` in one previous comparison. Recheck if position mismatch is suspected.

## 14. Contributing and documentation

Keep the main `README.md` and existing `docs/` unchanged until the team agrees on integration. Do not commit API keys, audio uploads, build artifacts, Python caches, or local virtual environments. Test single-room, multi-room and emergency-stop flows after any change.

Once reviewed by the team, the original README can link to this guide with:

```markdown
[AI Voice Control](AI_voice.md)
```

### Before committing this guide

Verify the team's Git repository URL; the ROS/Gazebo installation steps in the original documentation; the committed AI arrival tolerance; the final AI bridge executor; emergency-stop behaviour; and current Google model availability. These were not all established from the source snippets supplied for this documentation.
