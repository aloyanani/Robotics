from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

class DroneController:
    def __init__(self):
        print("Connecting to vehicle...")
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)

        print("Arming and taking off...")
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.mode.name == "GUIDED":
            print(" Waiting for GUIDED mode...")
            time.sleep(1)

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        self.vehicle.simple_takeoff(10)
        while self.vehicle.location.global_relative_frame.alt < 9:
            print(f" Altitude: {self.vehicle.location.global_relative_frame.alt:.2f}")
            time.sleep(1)

        print("Reached target altitude")
        self.current_location = self.vehicle.location.global_relative_frame

    def interpret_and_move(self, command_line):
        parts = command_line.strip().split()
        if len(parts) != 2:
            print("Use format: direction meters (e.g., 'forward 10')")
            return

        direction, distance_str = parts
        try:
            distance = float(distance_str)
        except ValueError:
            print("Invalid distance. Please use a number.")
            return

        # Approximate conversion: 1 meter ≈ 0.00001 degrees
        offset = distance * 0.00001

        # Get current location fresh each time
        current = self.vehicle.location.global_relative_frame
        lat = current.lat
        lon = current.lon
        alt = current.alt

        if direction == "left":
            lon -= offset
        elif direction == "right":
            lon += offset
        elif direction == "forward":
            lat += offset
        elif direction == "backward":
            lat -= offset
        elif direction == "land":
            print("Landing...")
            self.vehicle.mode = VehicleMode("LAND")
            return
        else:
            print("Unknown direction.")
            return

        target = LocationGlobalRelative(lat, lon, alt)
        self.vehicle.simple_goto(target)
        print(f"Moving {direction} by {distance} meters.")

    def run(self):
        print("Type a command like: 'forward 10', 'left 5', or 'land'")
        while True:
            command_line = input(">> ").lower().strip()
            if command_line:
                self.interpret_and_move(command_line)
            if command_line.startswith("land"):
                break

        print("Closing vehicle connection...")
        self.vehicle.close()

# Entry point
if __name__ == "__main__":
    controller = DroneController()
    controller.run()
