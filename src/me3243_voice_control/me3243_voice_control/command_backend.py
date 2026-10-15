
from me3243_voice_control.gemini_parser import parse_command


class CommandBackend:
    def process(self, text):
        command, latency = parse_command(text)

        print(f"Gemini command: {command}")
        print(f"Parser latency: {latency:.2f} s")

        return command


if __name__ == "__main__":
    backend = CommandBackend()

    try:
        while True:
            text = input("You: ").strip()

            if text.lower() in ("exit", "quit"):
                break

            if not text:
                continue

            command = backend.process(text)
            print(f"Robot command: {command}")

    except (KeyboardInterrupt, EOFError):
        print("\nExiting Gemini test.")
