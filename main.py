from threading import Timer
import math
import dronekit
import datetime
import time
import threading
import cv2
from dronekit import VehicleMode, LocationGlobalRelative, connect, Command, LocationGlobal
from pymavlink import mavutil

#activates camera for '8' seconds, starts a new thread to not interrupt other functions
def activate_camera():
    def capture_and_display():
        print("Activating camera...")
        camera = cv2.VideoCapture(0)  # Use the first available camera
        if camera.isOpened() == False:
            print("Error: Could not access the camera.")
            return

        # Capture for 8 seconds
        start_time = time.time()
        while time.time() - start_time < 8:
            ret, frame = camera.read()
            if not ret:
                print("Error: Could not read frame.")
                break
            cv2.imshow("Webcam", frame)
        print("Capture stopped")

        camera.release()
        cv2.destroyAllWindows()
        print("Camera deactivated.")
    camera_thread = threading.Thread(target=capture_and_display)
    camera_thread.start()

#function to get distance between two points (used to get distance to next wp)
def get_distance(location1, location2):
    return math.sqrt(((location2.lat - location1.lat) * (location2.lat - location1.lat)) + ((location2.lon - location1.lon) * (location2.lon - location1.lon))) * 1.113195e5

#use the correct port for connection
connection = 'tcp:127.0.0.1:5762'
plane = connect(connection, wait_ready=True, timeout=30)
print("Connected to plane...")

plane.commands.clear()
print("Commands cleared...")

#set mode to TAKEOFF, set ARMING_CHECK to '0' to bypass arming checks
plane.mode = VehicleMode("TAKEOFF")
time.sleep(3)
print("Plane mode switched to 'TAKEOFF'")

#arm the plane, set ARMING_CHECK to bypass all checks
plane.parameters['ARMING_CHECK'] = 0
if plane.armed == False:
    print("Arming the vehicle...")
    plane.armed = True

#timeout until plane is armed
while plane.armed == False:
    print("Waiting for plane to arm...")
    time.sleep(1)

#sets plane.home_location to current plane position if it is 'None' (simülasyon sırasında birden fazla kez kod çalıştırıldığında bozulmasını engelliyor)
if plane.home_location is None:
    print("Warning: Home location is not set yet. Waiting for GPS fix...")
    plane.home_location = plane.location.global_frame
#sets waypoints relative to home location
waypoints = [
    LocationGlobalRelative(plane.home_location.lat, plane.home_location.lon, plane.home_location.alt),
    LocationGlobalRelative(plane.home_location.lat, plane.home_location.lon - 0.005000, plane.home_location.alt),
    LocationGlobalRelative(plane.home_location.lat + 0.005000, plane.home_location.lon - 0.005000, plane.home_location.alt),
    LocationGlobalRelative(plane.home_location.lat + 0.005000, plane.home_location.lon, plane.home_location.alt),
    LocationGlobalRelative(plane.home_location.lat + 0.002500, plane.home_location.lon, plane.home_location.alt),
]

#adds the waypoints to plane commands as 'Command' objects, uploads commands
for waypoint in waypoints:
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0,
                  waypoint.lat, waypoint.lon, waypoint.alt)
    plane.commands.add(cmd)
    print("- waypoint uploaded -")
plane.commands.upload()
print("Mission uploaded...")

#sets mode to 'AUTO', timeout until plane is set to 'AUTO'
plane.mode = VehicleMode("AUTO")
print("Mission started...")
while plane.mode != VehicleMode("AUTO"):
    time.sleep(1)

#periodically (per second) writes plane data onto console
#when plane gets to  waypoint #2 activates camera for 8 seconds
camera_activated = False
while plane.mode.name == "AUTO":
    print(f"[{datetime.datetime.now()}]|LAT:{plane.location.global_frame.lat}/LON:{plane.location.global_frame.lon}/{plane.location.global_frame.alt}|P/Y/R:{plane.attitude.pitch}/{plane.attitude.yaw}/{plane.attitude.roll}|SPEED:{plane.airspeed}|BATTERY:{plane.battery.level}|WP:{get_distance(plane.location.global_frame, waypoints[plane.commands.next])}m")
    if plane.commands.next == 3 and not camera_activated:
        activate_camera()
        camera_activated = True
    time.sleep(1)

#set plane state to 'RTL'
print("Mission complete, switching to RTL mode...")
plane.mode = VehicleMode("RTL")
print("Plane mode switched to: 'RTL'")

while plane.mode.name == "RTL":
    print(f"[{datetime.datetime.now()}]|LAT:{plane.location.global_frame.lat}/LON:{plane.location.global_frame.lon}/{plane.location.global_frame.alt}|P/Y/R:{plane.attitude.pitch}/{plane.attitude.yaw}/{plane.attitude.roll}|SPEED:{plane.airspeed}|BATTERY:{plane.battery.level}|WP:{get_distance(plane.location.global_frame, waypoints[plane.commands.next])}m")
    time.sleep(1)

plane.close()

