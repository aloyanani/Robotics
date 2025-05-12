from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket

class DroneController:
    def __init__(self):
        print("Connecting to vehicle...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        print("Connected.")

    def interpret_and_move(self, command_line):
        parts = command_line.strip().split()

        # Change mode
        if len(parts) == 2 and parts[0] == "mode":
            new_mode = parts[1].upper()
            print(f"Changing mode to {new_mode}...")
            self.vehicle.mode = VehicleMode(new_mode)
            while not self.vehicle.mode.name == new_mode:
                time.sleep(0.5)
            print(f"Mode changed to {new_mode}")
            return new_mode == "LAND"

        # Arm
        if len(parts) == 1 and parts[0] == "arm":
            print("Arming motors...")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                print("  → Waiting for arming...")
                time.sleep(1)
            print("Motors armed.")
            return False

        # Takeoff part
        if len(parts) == 2 and parts[0] == "takeoff":
            try:
                relative_alt = float(parts[1])
            except ValueError:
                print("Invalid altitude. Use: takeoff 10")
                return False

            if not self.vehicle.armed:
                print("✖ ERROR: Drone is not armed. Say 'arm' first.")
                return False

            if self.vehicle.mode.name != "GUIDED":
                print("✖ ERROR: Mode must be GUIDED. Say 'mode GUIDED' first.")
                return False

            target_alt = self.vehicle.location.global_relative_frame.alt + relative_alt
            print(f"Taking off to {target_alt:.2f} meters...")
            self.vehicle.simple_takeoff(target_alt)
            while self.vehicle.location.global_relative_frame.alt < target_alt - 0.5:
                print(f"  → Altitude: {self.vehicle.location.global_relative_frame.alt:.2f} m")
                time.sleep(1)
            print("Reached target altitude.")
            return False

        # === MOVEMENT ===
        if len(parts) == 2:
            direction, value = parts
            try:
                distance = float(value)
            except ValueError:
                print("Invalid number.")
                return False

            offset = distance * 0.00001  # ~1 meter in degrees
            loc = self.vehicle.location.global_relative_frame
            lat, lon, alt = loc.lat, loc.lon, loc.alt

            if direction == "forward":
                lat += offset
            elif direction == "backward":
                lat -= offset
            elif direction == "right":
                lon += offset
            elif direction == "left":
                lon -= offset
            else:
                print("Unknown direction.")
                return False

            target = LocationGlobalRelative(lat, lon, alt)
            self.vehicle.simple_goto(target)
            print(f"Moving {direction} by {distance} meters.")
            return False

        # If command is unknown
        print("Unknown command or wrong format.")
        return False

    def run(self):
        print("Waiting for voice commands from Windows...")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 5000))
        server.listen(1)

        conn, addr = server.accept()
        print(f"Connection from {addr}")

        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                command_line = data.decode().lower().strip()
                print(f"Received: {command_line}")
                should_stop = self.interpret_and_move(command_line)
                if should_stop:
                    break

        print("Closing vehicle connection...")
        self.vehicle.close()

if __name__ == "__main__":
    controller = DroneController()
    controller.run()
