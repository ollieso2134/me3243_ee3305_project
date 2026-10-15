"""ME3243 browser-to-ROS2 AI bridge (ROS2 Jazzy; run inside me3243_ai_env).

Reuses the existing navigation, queue, Gemini command and Edge TTS modules.
Browser audio is uploaded to web_gui.py on the same ROS host and transcribed
by Faster-Whisper. This module does NOT require a microphone in WSL.
"""
import json
import os
import queue as task_queue
import re
import threading
import time
from urllib.request import urlopen

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String, Bool

from me3243_voice_control.command_backend import CommandBackend
from me3243_voice_control.navigation_manager import NavigationManager
from me3243_voice_control.waypoint_queue import WaypointQueue
from me3243_voice_control.intent_parser import extract_multi_waypoints
from me3243_voice_control.text_to_speech import TextToSpeech

STOP_PHRASES = {
    "stop", "stop robot", "stop moving", "stop navigation", "stop now",
    "emergency stop", "halt", "cancel navigation", "abort mission", "freeze",
}


def is_stop(text):
    return " ".join(re.sub(r"[^\w\s]", " ", text.lower()).split()) in STOP_PHRASES


class AIWebBridge(Node):
    def __init__(self):
        super().__init__("me3243_ai_web_bridge")
        self.pub_event = self.create_publisher(String, "ai_chat_event", 10)
        self.pub_ready = self.create_publisher(Bool, "ai_ready", 10)
        self.create_subscription(String, "ai_text_command", self.on_text, 10)
        self.create_subscription(String, "ai_audio_path", self.on_audio, 10)
        self.create_subscription(Bool, "ai_stop", self.on_stop, 10)
        self.create_timer(1.0, self.heartbeat)
        self.events = task_queue.Queue(maxsize=40)
        self.audio_model = None
        self.model_lock = threading.Lock()
        self.backend = CommandBackend()
        self.tts = TextToSpeech()
        self.nav = NavigationManager()
        self.room_queue = WaypointQueue(self.nav)
        self.state_lock = threading.RLock()
        self.generation = 0
        self.running = True
        self.spoken = task_queue.Queue(maxsize=20)
        self.last_room = None
        self.last_queue_state = "idle"
        self.last_mode = "unknown"
        self.worker = threading.Thread(target=self.process_loop, daemon=True)
        self.monitor = threading.Thread(target=self.monitor_loop, daemon=True)
        self.speaker = threading.Thread(target=self.speak_loop, daemon=True)
        self.worker.start()
        self.monitor.start()
        self.speaker.start()

    def publish_event(self, role, text):
        msg = String()
        msg.data = json.dumps({"role": role, "text": str(text)})
        self.pub_event.publish(msg)

    def heartbeat(self):
        msg = Bool(); msg.data = self.running
        self.pub_ready.publish(msg)

    def on_text(self, msg):
        text = msg.data.strip()
        if not text: return
        self.submit("text", text)

    def on_audio(self, msg):
        path = msg.data
        # Only process files created by our own upload endpoint.
        if not path.startswith("/tmp/me3243_ai_") or not path.endswith((".webm", ".ogg", ".wav", ".m4a")):
            self.publish_event("system", "Rejected unrecognized audio file path")
            return
        self.submit("audio", path)

    def on_stop(self, msg):
        if msg.data:
            self.stop_navigation("dashboard")

    def submit(self, kind, payload):
        with self.state_lock:
            generation = self.generation
        if kind == "text" and is_stop(payload):
            self.publish_event("user", payload)
            self.stop_navigation("chat")
            return
        try:
            self.events.put_nowait((kind, payload, generation))
        except task_queue.Full:
            if kind == "audio":
                try: os.unlink(payload)
                except OSError: pass
            self.publish_event("system", "Assistant busy; please try again")

    def stop_navigation(self, source):
        with self.state_lock:
            self.generation += 1
            self.room_queue.cancel()
            # Cancellation must send /stop even when the queue is idle.
            self.nav.cancel()
        self.publish_event("robot", "Navigation stopped. Remaining destinations cleared.")

    def say(self, message):
        self.publish_event("robot", message)
        try: self.spoken.put_nowait(message)
        except task_queue.Full: pass

    def speak_loop(self):
        while self.running:
            try: message = self.spoken.get(timeout=0.5)
            except task_queue.Empty: continue
            try: self.tts.speak(message)
            except Exception as error:
                self.publish_event("system", "Speech output failed: " + str(error))

    def transcribe(self, path):
        from faster_whisper import WhisperModel
        with self.model_lock:
            if self.audio_model is None:
                self.publish_event("system", "Loading Faster-Whisper speech model...")
                self.audio_model = WhisperModel("small", device="cpu", compute_type="int8")
            segments, _ = self.audio_model.transcribe(path, beam_size=3, language="en", vad_filter=True)
            return " ".join(s.text.strip() for s in segments).strip()

    def mode_sync(self):
        # HTTP status is authoritative, including when the AI node joins late.
        try:
            with urlopen("http://127.0.0.1:8080/api/status", timeout=1.5) as response:
                data = json.load(response)
            mode = data.get("control_mode", "unknown")
            if mode in ("voice", "manual"):
                self.nav.mode_callback(String(data=mode))
            return mode
        except Exception:
            return self.nav.control_mode

    def process_loop(self):
        while self.running:
            try: kind, payload, generation = self.events.get(timeout=0.4)
            except task_queue.Empty: continue
            try:
                if kind == "audio":
                    try:
                        self.publish_event("system", "Transcribing voice command...")
                        text = self.transcribe(payload)
                    finally:
                        try: os.unlink(payload)
                        except OSError: pass
                    if not text:
                        self.publish_event("system", "No speech recognised")
                        continue
                else:
                    text = payload
                with self.state_lock:
                    if generation != self.generation:
                        continue
                self.publish_event("user", text)
                if is_stop(text):
                    self.stop_navigation("voice")
                    continue
                mode = self.mode_sync()
                if mode != "voice":
                    self.say("Select AI VOICE CONTROL on the dashboard first.")
                    continue
                multi = extract_multi_waypoints(text)
                if multi is not None:
                    rooms = multi["waypoints"]
                    with self.state_lock:
                        if generation != self.generation: continue
                        if self.nav.active or self.room_queue.state == "running":
                            response = "Navigation already running. Stop it before a new command."
                        elif self.room_queue.start(rooms):
                            response = "Starting navigation through rooms " + ", ".join(map(str, rooms)) + "."
                        else:
                            response = "Could not start the waypoint queue."
                    self.say(response)
                    continue
                command = self.backend.process(text)
                with self.state_lock:
                    if generation != self.generation: continue
                    action = command.get("action")
                    if action == "goto_room":
                        room = command.get("room")
                        if type(room) is not int or not 1 <= room <= 6:
                            response = "Please specify a room from one to six."
                        elif self.nav.active or self.room_queue.state == "running":
                            response = "Navigation already running. Stop it before a new command."
                        elif self.nav.navigate(room):
                            response = f"Navigating to room {room}."
                        else:
                            response = "Unable to start navigation. Check odometry and AI mode."
                    elif action == "chat":
                        response = command.get("reply", "Could you clarify your request?")
                    elif action == "stop":
                        self.stop_navigation("Gemini")
                        continue
                    elif action in ("describe", "approach"):
                        response = "This function is not connected in the current simulation."
                    else:
                        response = "That command is not supported yet."
                self.say(response)
            except Exception as error:
                self.publish_event("system", "AI processing error: " + str(error))

    def monitor_loop(self):
        last_mode_poll = 0.0
        prev_active_room = None
        prev_state = "idle"
        while self.running:
            time.sleep(0.15)
            if time.monotonic() - last_mode_poll > 1.0:
                mode = self.mode_sync()
                last_mode_poll = time.monotonic()
                if mode == "manual" and self.last_mode == "voice":
                    with self.state_lock:
                        if self.nav.active or self.room_queue.state == "running":
                            self.stop_navigation("manual mode")
                self.last_mode = mode
            with self.state_lock:
                old_room = self.room_queue.current_room
                old_state = self.room_queue.state
                old_nav_active = self.nav.active
                old_nav_status = self.nav.status
                self.room_queue.update()
                new_room = self.room_queue.current_room
                new_state = self.room_queue.state
                if old_nav_status == "succeeded" and old_room is not None and not old_nav_active:
                    # Queue has just consumed a successful arrival.
                    if new_room != old_room or new_state == "completed":
                        if new_state == "completed":
                            self.say(f"Arrived at room {old_room}. All destinations completed successfully.")
                        else:
                            self.say(f"Arrived at room {old_room}. Proceeding to room {new_room}.")
                elif (not old_nav_active and old_nav_status == "succeeded"
                      and old_room is None and new_state != "completed"
                      and old_state != "completed"):
                    # Single-room navigation, avoid repeated arrival announcements.
                    room = self.nav.current_room
                    if room is not None and room != prev_active_room:
                        self.say(f"I have arrived at room {room}.")
                        prev_active_room = room
                if self.nav.active:
                    prev_active_room = None

    def close(self):
        self.running = False
        with self.state_lock:
            self.room_queue.cancel()
            self.nav.cancel()
        for thread in (self.monitor, self.worker, self.speaker):
            thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    bridge = AIWebBridge()
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(bridge.nav)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        executor.shutdown()
        bridge.nav.destroy_node()
        bridge.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
