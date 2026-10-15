
import re


ROOM_PATTERN = re.compile(
    r"\brooms?\s*(?:number\s*)?([1-6]|one|two|three|four|five|six)\b",
    re.IGNORECASE
)

ROOM_NUMBERS = {
    "one": 1,
    "two": 2,
    "three": 3,
    "four": 4,
    "five": 5,
    "six": 6,
}


def extract_multi_waypoints(text):
    if not isinstance(text, str):
        return None

    matches = ROOM_PATTERN.findall(text)

    if len(matches) < 2:
        return None

    rooms = [
        int(m) if m.isdigit() else ROOM_NUMBERS[m.lower()]
        for m in matches
    ]

    return {
        "action": "navigate_multi",
        "waypoints": rooms,
    }


if __name__ == "__main__":
    examples = [
        "Go to room 2 then room 4 then room 6",
        "Visit room one followed by room three",
        "Navigate to room 5",
    ]

    for example in examples:
        print(example, "->", extract_multi_waypoints(example))
