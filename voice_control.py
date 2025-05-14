from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import math
from pymavlink import mavutil

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

        # Takeoff
        if len(parts) == 3 and parts[0] == "take" and parts[1] == "off":
            try:
                relative_alt = float(parts[2])
            except ValueError:
                print("Invalid altitude.")
                return False

            if not self.vehicle.armed:
                print("✖ ERROR: Drone is not armed.")
                return False
            if self.vehicle.mode.name != "GUIDED":
                print("✖ ERROR: Mode must be GUIDED.")
                return False

            target_alt = self.vehicle.location.global_relative_frame.alt + relative_alt
            print(f"Taking off to {target_alt:.2f} meters...")
            self.vehicle.simple_takeoff(target_alt)
            while self.vehicle.location.global_relative_frame.alt < target_alt - 0.5:
                print(f"  → Altitude: {self.vehicle.location.global_relative_frame.alt:.2f} m")
                time.sleep(1)
            print("Reached target altitude.")
            return False

        # TURN AND MOVE (e.g., "right 5", "left 5")
        if len(parts) == 2 and parts[0] in ["right", "left"]:
            turn_dir = parts[0]
            try:
                distance = float(parts[1])
            except ValueError:
                print("Invalid distance.")
                return False

            original_heading = self.vehicle.heading
            turn_angle = 90  # always turn 90 degrees
            target_heading = (original_heading + (turn_angle if turn_dir == "right" else -turn_angle)) % 360

            print(f"Turning {turn_dir} to heading {target_heading}°...")
            self.vehicle.send_mavlink(self.vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                turn_angle,           # 90° turn
                10,                   # Turn speed (degrees/sec)
                1 if turn_dir == "right" else -1,  # Turn direction
                1,                   # 1 = relative to current heading
                0, 0, 0
            ))
            time.sleep(4)

            heading_rad = math.radians(target_heading)
            loc = self.vehicle.location.global_relative_frame
            d_lat = distance * math.cos(heading_rad) / 111139
            d_lon = distance * math.sin(heading_rad) / (111139 * math.cos(math.radians(loc.lat)))
            target = LocationGlobalRelative(loc.lat + d_lat, loc.lon + d_lon, loc.alt)

            print(f"Moving {distance} meters in {turn_dir} direction.")
            self.vehicle.simple_goto(target)
            return False

        # FORWARD/BACKWARD MOVEMENT
        if len(parts) == 2 and parts[0] in ["forward", "backward"]:
            try:
                distance = float(parts[1])
            except ValueError:
                print("Invalid distance.")
                return False

            heading = math.radians(self.vehicle.heading)
            if parts[0] == "backward":
                heading += math.pi  # reverse

            current = self.vehicle.location.global_relative_frame
            d_lat = distance * math.cos(heading) / 111139
            d_lon = distance * math.sin(heading) / (111139 * math.cos(math.radians(current.lat)))
            target = LocationGlobalRelative(current.lat + d_lat, current.lon + d_lon, current.alt)

            print(f"Moving {parts[0]} {distance} meters based on heading.")
            self.vehicle.simple_goto(target)
            return False

        print("Unknown command.")
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
