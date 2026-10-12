
import json
import os
import time

from google import genai
from google.genai import types


GEMINI_MODEL = os.getenv(
    "GEMINI_MODEL",
    "gemini-3.5-flash-lite"
)

SYSTEM_PROMPT = """
You are the dialogue manager of an indoor TurtleBot3 robot.

The house contains exactly six valid rooms:
Room 1, Room 2, Room 3, Room 4, Room 5, Room 6.

For EVERY user message, return ONLY one JSON object.

Allowed commands:

1. Navigate to a room:
{"action": "goto_room", "room": 1}

The room must be an integer from 1 to 6.
Understand natural paraphrases such as:
- Go to Room 3
- Could you check room three please?
- Head over to the third room
- Navigate to room number five

2. Describe the current scene:
{"action": "describe"}

3. Approach an object:
{"action": "approach", "object": "cup"}

4. Stop:
{"action": "stop"}

5. General conversation or clarification:
{"action": "chat", "reply": "your response"}

If the user requests an invalid room, explain that
only Rooms 1 through 6 are available.

If the request is unclear, ask for clarification.

Unsupported locomotion:
The robot is a wheeled ground robot.
It cannot fly, jump, climb, swim or teleport.

Do not interpret unsupported locomotion as navigation,
even if a valid room number is mentioned.

Example:
User: Fly to Room 2
Output:
{"action": "chat", "reply": "I cannot fly. I can only navigate on the ground."}

Memory and reminders:
If the user asks you to remember information about
an action, do not execute that action unless explicitly
requested to do it now.

Use conversation history to interpret follow-up requests.
"""

history = []


def parse_command(user_text):
    if not os.getenv("GEMINI_API_KEY"):
        raise RuntimeError(
            "GEMINI_API_KEY environment variable is missing."
        )

    client = genai.Client(
        api_key=os.environ["GEMINI_API_KEY"]
    )

    conversation = ""

    for item in history:
        conversation += (
            f"{item['role'].upper()}: "
            f"{item['content']}\n"
        )

    conversation += f"USER: {user_text}\n"

    start_time = time.monotonic()

    try:
        response = client.models.generate_content(
            model=GEMINI_MODEL,
            contents=conversation,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                response_mime_type="application/json",
                temperature=0.1,
            ),
        )

        command = json.loads(response.text.strip())

        if not isinstance(command, dict):
            raise ValueError("Gemini output is not a JSON object.")

        if command.get("action") not in {
            "goto_room", "describe", "approach", "stop", "chat"
        }:
            raise ValueError("Unsupported Gemini action.")

    except Exception as error:
        print(f"Gemini parser error: {error}")

        command = {
            "action": "chat",
            "reply": (
                "Sorry, I could not understand "
                "that request. Please try again."
            ),
        }

    latency = time.monotonic() - start_time

    history.append({
        "role": "user",
        "content": user_text,
    })

    history.append({
        "role": "assistant",
        "content": json.dumps(command),
    })

    return command, latency
