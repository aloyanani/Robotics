from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import socket

def get_location_metres(original_location, dNorth, dEast, dAlt=0):
    earth_radius = 6378137.0  # meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.radians(original_location.lat)))
    newlat = original_location.lat + math.degrees(dLat)
    newlon = original_location.lon + math.degrees(dLon)
    newalt = original_location.alt + dAlt
    return LocationGlobalRelative(newlat, newlon, newalt)

class DroneController:
    def __init__(self):
        print("Connecting to vehicle...")
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        print("Connected.")

        print("Setting GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.mode.name == "GUIDED":
            time.sleep(1)

        print("Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        print("Taking off to 10 meters...")
        self.vehicle.simple_takeoff(10)
        while self.vehicle.location.global_relative_frame.alt < 9:
            print(f" Altitude: {self.vehicle.location.global_relative_frame.alt:.2f}")
            time.sleep(1)

        print("Reached target altitude.")

    def move_direction(self, direction, distance):
        direction = direction.lower()
        dNorth = 0
        dEast = 0

        if direction == "forward":
            dNorth = distance
        elif direction == "backward":
            dNorth = -distance
        elif direction == "right":
            dEast = distance
        elif direction == "left":
            dEast = -distance
        else:
            print("Unknown direction. Use: forward, backward, left, right")
            return

        current_location = self.vehicle.location.global_relative_frame
        target_location = get_location_metres(current_location, dNorth, dEast)
        self.vehicle.simple_goto(target_location)
        print(f"Moving {direction} by {distance} meters")

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
                print(f">> {command_line}")
                if command_line.startswith("land"):
                    self.vehicle.mode = VehicleMode("LAND")
                    print("Landing...")
                    break

                parts = command_line.split()
                if len(parts) != 2:
                    print("Invalid command. Format: direction meters")
                    continue

                direction, value = parts
                try:
                    distance = float(value)
                    self.move_direction(direction, distance)
                except ValueError:
                    print("Invalid distance. Please enter a number.")

        print("Closing connection...")
        self.vehicle.close()

if __name__ == "__main__":
    controller = DroneController()
    controller.run()
