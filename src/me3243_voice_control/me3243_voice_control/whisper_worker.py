
import json
import sys

from faster_whisper import WhisperModel


def send_response(data):
    print(json.dumps(data), flush=True)


def main():
    try:
        print(
            "Loading Faster-Whisper model...",
            file=sys.stderr,
            flush=True,
        )

        model = WhisperModel(
            "small",
            device="cpu",
            compute_type="int8",
        )

        send_response({"status": "ready"})

    except Exception as error:
        send_response({
            "status": "error",
            "message": str(error),
        })
        return

    # Keep running and accept multiple audio recordings.
    for line in sys.stdin:
        try:
            request = json.loads(line)
            audio_path = request["audio_path"]

            segments, info = model.transcribe(
                audio_path,
                language="en",
                beam_size=5,
                condition_on_previous_text=False,
            )

            transcript = " ".join(
                segment.text.strip()
                for segment in segments
            ).strip()

            send_response({
                "status": "success",
                "text": transcript,
            })

        except Exception as error:
            send_response({
                "status": "error",
                "message": str(error),
            })


if __name__ == "__main__":
    main()
