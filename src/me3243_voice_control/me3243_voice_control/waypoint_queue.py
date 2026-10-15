
import threading


class WaypointQueue:
    def __init__(self, navigation_manager):
        self.nav = navigation_manager
        self.rooms = []
        self.current_room = None
        self.state = "idle"

        # Protect the queue against simultaneous voice and
        # navigation-monitor operations.
        self.lock = threading.RLock()

    def start(self, rooms):
        with self.lock:
            if self.nav.active or self.state in ("running", "canceling"):
                return False

            if not isinstance(rooms, list) or not rooms:
                return False

            if any(type(r) is not int or not 1 <= r <= 6 for r in rooms):
                return False

            self.rooms = list(rooms)
            self.current_room = None
            self.state = "running"
            self._next_locked()
            return self.state == "running"

    def _next_locked(self):
        if self.state != "running":
            return

        if not self.rooms:
            self.current_room = None
            self.state = "completed"
            print("All waypoints completed.")
            return

        self.current_room = self.rooms.pop(0)
        print(f"Queue: Navigating to Room {self.current_room}")

        if not self.nav.navigate(self.current_room):
            self.rooms.clear()
            self.state = "failed"
            print("Queue: Failed to start navigation.")

    def update(self):
        with self.lock:
            if self.state == "canceling":
                if not self.nav.active:
                    self.state = "canceled"
                    self.current_room = None
                return

            if self.state != "running":
                return

            if self.nav.active:
                return

            if self.nav.status == "succeeded":
                print(f"Queue: Room {self.current_room} reached.")
                self._next_locked()

            elif self.nav.status in ("failed", "rejected", "canceled"):
                self.rooms.clear()
                self.current_room = None
                self.state = self.nav.status

    def cancel(self):
        with self.lock:
            self.rooms.clear()

            if self.nav.active:
                self.state = "canceling"
                self.nav.cancel()
            else:
                self.state = "canceled"
                self.current_room = None

            print("Queue: Remaining waypoints cleared.")

    def get_status(self):
        with self.lock:
            return {
                "state": self.state,
                "current_room": self.current_room,
                "remaining_rooms": list(self.rooms),
            }
