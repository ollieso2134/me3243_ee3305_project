
import rclpy
from me3243_voice_control.navigation_manager import NavigationManager


def main():
    rclpy.init()
    nav = NavigationManager()

    print("ME3243 Navigation Test")
    print("Commands: go 1-6, stop, status, exit")

    try:
        while rclpy.ok():
            # Process ROS2 callbacks while waiting for user input.
            rclpy.spin_once(nav, timeout_sec=0.1)

            import select
            import sys

            if not select.select([sys.stdin], [], [], 0)[0]:
                continue

            command = input("Command: ").strip().lower()

            if command.startswith("go "):
                try:
                    room = int(command.split()[1])
                    nav.navigate(room)
                except (ValueError, IndexError):
                    print("Use: go 1-6")

            elif command == "stop":
                nav.cancel()

            elif command == "status":
                print(f"Navigation status: {nav.status}")

            elif command == "exit":
                if nav.active:
                    print("Canceling active navigation before exiting...")
                    nav.cancel()
                    while rclpy.ok() and nav.active:
                        rclpy.spin_once(nav, timeout_sec=0.1)
                break

            else:
                print("Unknown command.")

    except KeyboardInterrupt:
        if nav.active:
            nav.cancel()
            while rclpy.ok() and nav.active:
                rclpy.spin_once(nav, timeout_sec=0.1)

    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
