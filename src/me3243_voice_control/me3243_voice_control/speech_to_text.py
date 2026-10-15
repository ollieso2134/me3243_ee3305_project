
import json
import os
import subprocess
import tempfile
import threading
import time
import wave
from pathlib import Path

import numpy as np


class SpeechToText:
    def __init__(
        self,
        silence_threshold=250,
        silence_duration=1.2,
        max_duration=10.0,
        start_timeout=5.0,
    ):
        self.silence_threshold = silence_threshold
        self.silence_duration = silence_duration
        self.max_duration = max_duration
        self.start_timeout = start_timeout

        self.sample_rate = 16000
        self.chunk_duration = 0.1

        self.whisper_python = (
            Path.home() / "me3243_ai_env" / "bin" / "python"
        )
        self.worker_path = (
            Path(__file__).resolve().parent / "whisper_worker.py"
        )

        self.worker = None
        self.worker_lock = threading.Lock()

    def start(self):
        """Start Whisper once and wait until the model is ready."""
        with self.worker_lock:
            if self.worker is not None and self.worker.poll() is None:
                return

            print("Starting persistent Faster-Whisper...")

            self.worker = subprocess.Popen(
                [
                    str(self.whisper_python),
                    "-u",
                    str(self.worker_path),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )

            response = self.worker.stdout.readline()

            if not response:
                self.close()
                raise RuntimeError("Whisper worker failed to start.")

            message = json.loads(response)

            if message.get("status") != "ready":
                self.close()
                raise RuntimeError(
                    f"Whisper initialization failed: {message}"
                )

            print("Faster-Whisper ready.")

    def transcribe(self, audio_path):
        """Transcribe a WAV file using the persistent worker."""
        self.start()

        with self.worker_lock:
            request = {"audio_path": str(audio_path)}

            self.worker.stdin.write(json.dumps(request) + "\n")
            self.worker.stdin.flush()

            response = self.worker.stdout.readline()

            if not response:
                raise RuntimeError("Whisper worker stopped unexpectedly.")

            message = json.loads(response)

            if message.get("status") != "success":
                raise RuntimeError(
                    f"Whisper transcription failed: {message}"
                )

            return message.get("text", "").strip()

    def listen(self):
        """Record speech until silence, then transcribe it."""
        chunk_samples = int(
            self.sample_rate * self.chunk_duration
        )
        chunk_bytes = chunk_samples * 2

        command = [
            "parecord",
            "--device=RDPSource",
            "--raw",
            "--format=s16le",
            f"--rate={self.sample_rate}",
            "--channels=1",
        ]

        process = None
        audio_path = None
        frames = []
        speech_started = False
        silence_time = 0.0

        try:
            self.start()
            print("Listening...")

            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )

            start_time = time.monotonic()

            while True:
                elapsed = time.monotonic() - start_time

                if elapsed >= self.max_duration:
                    print("Maximum recording duration reached.")
                    break

                if not speech_started and elapsed >= self.start_timeout:
                    print("No speech detected.")
                    return None

                data = process.stdout.read(chunk_bytes)

                if not data:
                    print("Microphone stream ended.")
                    return None

                frames.append(data)

                samples = np.frombuffer(
                    data, dtype=np.int16
                ).astype(np.float32)

                rms = np.sqrt(np.mean(samples ** 2))

                if rms >= self.silence_threshold:
                    speech_started = True
                    silence_time = 0.0
                elif speech_started:
                    silence_time += self.chunk_duration

                if (
                    speech_started
                    and silence_time >= self.silence_duration
                ):
                    print("Speech ended.")
                    break

            if not speech_started:
                return None

            with tempfile.NamedTemporaryFile(
                suffix=".wav", delete=False
            ) as temp:
                audio_path = temp.name

            with wave.open(audio_path, "wb") as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(self.sample_rate)
                wav.writeframes(b"".join(frames))

            print("Recognising speech with Whisper...")

            text = self.transcribe(audio_path)

            if not text:
                print("No speech recognised.")
                return None

            print(f"You said: {text}")
            return text

        except Exception as error:
            print(f"Speech recognition error: {error}")
            return None

        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()

                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

                if process.stdout is not None:
                    process.stdout.close()

            if audio_path and os.path.exists(audio_path):
                os.remove(audio_path)

    def close(self):
        """Shut down the persistent worker cleanly."""
        worker = self.worker
        self.worker = None

        if worker is None:
            return

        if worker.poll() is None:
            worker.terminate()

        try:
            worker.wait(timeout=3)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.wait()

        if worker.stdin is not None:
            worker.stdin.close()

        if worker.stdout is not None:
            worker.stdout.close()


if __name__ == "__main__":
    stt = SpeechToText()

    try:
        stt.start()

        while True:
            input("\nPress Enter to record (or Ctrl+C to quit)...")
            stt.listen()

    except (KeyboardInterrupt, EOFError):
        print("\nExiting...")

    finally:
        stt.close()
