
import asyncio
import os
import subprocess
import tempfile

import edge_tts


class TextToSpeech:
    def __init__(self, voice="en-US-AriaNeural"):
        self.voice = voice

    def speak(self, text):
        if not text:
            return

        print(f"Robot: {text}")
        audio_path = None

        async def generate(path):
            communicate = edge_tts.Communicate(
                text=str(text),
                voice=self.voice
            )
            await communicate.save(path)

        try:
            with tempfile.NamedTemporaryFile(
                suffix=".mp3", delete=False
            ) as temp:
                audio_path = temp.name

            asyncio.run(generate(audio_path))

            subprocess.run(
                [
                    "ffplay",
                    "-nodisp",
                    "-autoexit",
                    "-loglevel", "error",
                    audio_path
                ],
                check=True
            )

        except Exception as error:
            print(f"TTS error: {error}")

        finally:
            if audio_path and os.path.exists(audio_path):
                os.remove(audio_path)


if __name__ == "__main__":
    tts = TextToSpeech()
    tts.speak("Hello, I am your ME3243 robot assistant.")
    tts.speak("I am ready to receive your commands.")
