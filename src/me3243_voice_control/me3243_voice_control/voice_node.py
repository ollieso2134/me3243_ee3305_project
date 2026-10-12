
import re
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from me3243_voice_control.speech_to_text import SpeechToText
from me3243_voice_control.command_backend import CommandBackend
from me3243_voice_control.text_to_speech import TextToSpeech
from me3243_voice_control.navigation_manager import NavigationManager
from me3243_voice_control.waypoint_queue import WaypointQueue
from me3243_voice_control.intent_parser import extract_multi_waypoints


STOP_PHRASES = {
    "stop",
    "stop robot",
    "stop moving",
    "stop navigation",
    "stop now",
    "emergency stop",
    "halt",
    "cancel navigation",
    "abort mission",
    "freeze",
}


def is_stop_command(text):
    normalized = re.sub(r"[^\w\s]", " ", text.lower())
    normalized = " ".join(normalized.split())
    return normalized in STOP_PHRASES


def main():
    rclpy.init()

    stt = SpeechToText()
    backend = CommandBackend()
    tts = TextToSpeech()

    # New ROS2 Jazzy navigation manager.
    nav = NavigationManager()
    queue = WaypointQueue(nav)

    navigation_lock = threading.RLock()
    shutdown_event = threading.Event()
    record_event = threading.Event()
    voice_busy = threading.Event()

    # Incremented whenever earlier voice commands
    # must no longer be executed.
    command_generation = 0

    # -------------------------------------------
    # ROS2 EXECUTOR
    # -------------------------------------------
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(nav)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    spin_thread.start()

    # -------------------------------------------
    # STOP NAVIGATION
    # -------------------------------------------
    def stop_navigation(source="keyboard"):
        nonlocal command_generation

        with navigation_lock:
            command_generation += 1

            was_active = nav.active

            # Clear all remaining room destinations.
            queue.cancel()

            # Queue.cancel() already calls nav.cancel()
            # if a goal is active.
            #
            # Otherwise, publish /stop explicitly to
            # halt any navigation not tracked by this node.
            if not was_active:
                nav.cancel()

        print(f"\nStop requested via {source}.")
        print("ROS2 /stop request published.")

    # -------------------------------------------
    # MONITOR WAYPOINT QUEUE AND CONTROL MODE
    # -------------------------------------------
    def monitor_queue():
        nonlocal command_generation

        while not shutdown_event.wait(0.1):
            with navigation_lock:

                # If the dashboard switches to MANUAL,
                # cancel AI navigation and queued rooms.
                if (
                    nav.control_mode == "manual"
                    and (
                        nav.active
                        or queue.state == "running"
                    )
                ):
                    command_generation += 1
                    queue.cancel()
                    print(
                        "\nAI navigation canceled: "
                        "MANUAL mode selected."
                    )

                queue.update()

    queue_thread = threading.Thread(
        target=monitor_queue,
        daemon=True,
    )
    queue_thread.start()

    # -------------------------------------------
    # PROCESS VOICE COMMANDS
    # -------------------------------------------
    def process_voice_command(text, generation):
        if shutdown_event.is_set():
            return

        print(f"\nVoice command: {text}")

        # STOP commands bypass Gemini.
        if is_stop_command(text):
            stop_navigation("voice")
            tts.speak("Stop requested.")
            return

        # Reject commands recorded before a stop.
        with navigation_lock:
            if generation != command_generation:
                print("Discarding outdated voice command.")
                return

        # ---------------------------------------
        # MULTI-WAYPOINT NAVIGATION
        # ---------------------------------------
        multi_command = extract_multi_waypoints(text)

        if multi_command is not None:
            rooms = multi_command["waypoints"]

            with navigation_lock:
                if (
                    shutdown_event.is_set()
                    or generation != command_generation
                ):
                    return

                if nav.control_mode != "voice":
                    reply = (
                        "Please enable AI voice control "
                        "on the dashboard first."
                    )

                elif nav.active or queue.state in (
                    "running",
                    "canceling",
                ):
                    reply = (
                        "Navigation is already running. "
                        "Please stop it first."
                    )

                elif queue.start(rooms):
                    room_names = ", ".join(
                        str(room) for room in rooms
                    )
                    reply = (
                        "Starting navigation through rooms "
                        + room_names + "."
                    )

                else:
                    reply = (
                        "Unable to start waypoint navigation."
                    )

            if not shutdown_event.is_set():
                tts.speak(reply)

            return

        # ---------------------------------------
        # GEMINI COMMAND INTERPRETATION
        # ---------------------------------------
        command = backend.process(text)

        with navigation_lock:
            if (
                shutdown_event.is_set()
                or generation != command_generation
            ):
                print("Discarding outdated Gemini command.")
                return

            action = command.get("action")

            if action == "goto_room":
                room = command.get("room")

                if type(room) is not int or not 1 <= room <= 6:
                    reply = (
                        "Please specify a valid room "
                        "between one and six."
                    )

                elif nav.control_mode != "voice":
                    reply = (
                        "Please enable AI voice control "
                        "on the dashboard first."
                    )

                elif nav.active or queue.state in (
                    "running",
                    "canceling",
                ):
                    reply = (
                        "Navigation is already running. "
                        "Please stop it first."
                    )

                elif nav.navigate(room):
                    reply = f"Navigating to room {room}."

                else:
                    reply = "Navigation could not be started."

            elif action == "stop":
                reply = "Stop requested."

            elif action == "chat":
                reply = command.get(
                    "reply",
                    "Could you clarify your request?",
                )

            elif action == "describe":
                reply = (
                    "Scene description is not connected yet."
                )

            elif action == "approach":
                reply = (
                    "Object approach is not connected yet."
                )

            else:
                reply = "That command is not supported yet."

        if action == "stop":
            stop_navigation("Gemini")

        if not shutdown_event.is_set():
            tts.speak(reply)

    # -------------------------------------------
    # BACKGROUND VOICE WORKER
    # -------------------------------------------
    def voice_worker():
        while not shutdown_event.is_set():
            record_event.wait()

            if shutdown_event.is_set():
                break

            record_event.clear()
            voice_busy.set()

            try:
                with navigation_lock:
                    generation = command_generation

                text = stt.listen()

                if shutdown_event.is_set():
                    break

                if not text:
                    print("No speech recognised.")
                    continue

                process_voice_command(text, generation)

            except Exception as error:
                print(f"Voice processing error: {error}")

            finally:
                voice_busy.clear()
                print("\nVoice assistant ready.")

    voice_thread = threading.Thread(
        target=voice_worker,
        daemon=True,
    )

    # -------------------------------------------
    # STARTUP
    # -------------------------------------------
    print("\n======================================")
    print("     ME3243 AI VOICE CONTROL")
    print("======================================")
    print("Enter  : Record a voice command")
    print("s      : Stop navigation")
    print("stop   : Stop navigation")
    print("status : Navigation and queue status")
    print("exit   : Close AI voice assistant")
    print("")
    print("Enable AI VOICE CONTROL in the web GUI.")
    print("")

    try:
        # Load Faster-Whisper only once.
        stt.start()

        voice_thread.start()

        tts.speak("AI voice control is ready.")

        # Main thread remains available for
        # keyboard commands during speech processing.
        while rclpy.ok() and not shutdown_event.is_set():

            user_input = input(
                "\nCommand: "
            ).strip().lower()

            # INDEPENDENT KEYBOARD STOP
            if user_input in ("s", "stop"):
                stop_navigation("keyboard")
                continue

            # STATUS
            if user_input == "status":
                with navigation_lock:
                    print(f"Control mode: {nav.control_mode}")
                    print(f"Navigation: {nav.status}")
                    print(f"Room: {nav.current_room}")
                    print(f"Queue: {queue.get_status()}")
                continue

            # EXIT
            if user_input in ("exit", "quit", "q"):
                print("Closing AI voice assistant.")
                break

            # START ONE VOICE RECORDING
            if user_input == "":
                if voice_busy.is_set() or record_event.is_set():
                    print(
                        "Voice processing is already running. "
                        "Please wait."
                    )
                else:
                    record_event.set()

                continue

            print("Unknown command.")

    except (KeyboardInterrupt, EOFError):
        print("\nAI voice assistant interrupted.")

    finally:
        shutdown_event.set()
        record_event.set()

        # Stop the queue monitor.
        queue_thread.join()

        # Cancel active navigation and clear pending rooms.
        stop_navigation("shutdown")

        # Wait for any ongoing speech processing.
        if voice_thread.is_alive():
            voice_thread.join()

        # Shut down the persistent Whisper process.
        stt.close()

        executor.shutdown()
        spin_thread.join()

        nav.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("ME3243 AI voice assistant closed.")


if __name__ == "__main__":
    main()
