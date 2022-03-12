"""Robot_Controller"""

from controller import Robot
import random

# create the Robot instance.

robot = Robot()

# Set Timestep as multiples of WorldInfo/basicTimeStep

TIME_STEP = int(robot.getBasicTimeStep())*1

# Rotational Motors named as the wheel they are attached to

WHEEL_NAMES = ["wheelFrontRight", "wheelBackRight", "wheelBackLeft", "wheelFrontLeft"]

# List of distance sensor names

DISTANCE_SENSOR_NAMES = ["distanceSensorMiddle", "distanceSensorRight", "distanceSensorLeft"]

# gps sensor and camera names

GPS_SENSOR_NAME = "gps"
CAMERA_NAME = "cam"

# Max velocity in rads/sec (due to Rotational Motor)

MAX_VELOCITY = 5.0

# Instantiating the end-effectors and sensors
wheel = {wheel_name:robot.getDevice(wheel_name) for wheel_name in WHEEL_NAMES}
dis_sensor = {sensor_name:robot.getDevice(sensor_name) for sensor_name in DISTANCE_SENSOR_NAMES}
gps = robot.getDevice(GPS_SENSOR_NAME)
cam = robot.getDevice(CAMERA_NAME)
# Set the rotationalMotor target position to infinity for endless rotational motion
# based on velocity. 
for wheel_name in wheel.keys():
    wheel[wheel_name].setPosition(float("inf"))
    wheel[wheel_name].setVelocity(0.0)

# Enable the distance sensor(s) and use TIME_STEP for update interval 
for sensor_name in dis_sensor.keys():
    dis_sensor[sensor_name].enable(TIME_STEP)

# Enable the gps sensor and camera and use TIME_STEP for update interval
gps.enable(TIME_STEP)
cam.enable(TIME_STEP)
cam.recognitionEnable(TIME_STEP)

# Some methods for robot movement

def in_place_left_turn():
    wheel["wheelFrontRight"].setVelocity(MAX_VELOCITY*2)
    wheel["wheelBackRight"].setVelocity(MAX_VELOCITY*2)
    
    wheel["wheelFrontLeft"].setVelocity(-MAX_VELOCITY*2)
    wheel["wheelBackLeft"].setVelocity(-MAX_VELOCITY*2)
    


def in_place_right_turn():
    wheel["wheelFrontRight"].setVelocity(-MAX_VELOCITY*2)
    wheel["wheelBackRight"].setVelocity(-MAX_VELOCITY*2)
    
    wheel["wheelFrontLeft"].setVelocity(MAX_VELOCITY*2)
    wheel["wheelBackLeft"].setVelocity(MAX_VELOCITY*2)
    


def move_forward():
    wheel["wheelFrontRight"].setVelocity(MAX_VELOCITY)
    wheel["wheelBackRight"].setVelocity(MAX_VELOCITY)
    
    wheel["wheelFrontLeft"].setVelocity(MAX_VELOCITY)
    wheel["wheelBackLeft"].setVelocity(MAX_VELOCITY)
    
    
def move_backward():
    wheel["wheelFrontRight"].setVelocity(-MAX_VELOCITY*0.1)
    wheel["wheelBackRight"].setVelocity(-MAX_VELOCITY*0.1)
    
    wheel["wheelFrontLeft"].setVelocity(-MAX_VELOCITY*0.1)
    wheel["wheelBackLeft"].setVelocity(-MAX_VELOCITY*0.1)
    

def stop():
    wheel["wheelFrontRight"].setVelocity(0)
    wheel["wheelBackRight"].setVelocity(0)
    
    wheel["wheelFrontLeft"].setVelocity(0)
    wheel["wheelBackLeft"].setVelocity(0)
    

# rotate and reverse holds the number of Time Steps for an action to continue
rotate = 0
reverse = 0

direction_of_rotation = None

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Reading the sensors:
    cam_objects_num = cam.getRecognitionNumberOfObjects()
    gps_data = gps.getValues()
    dis_sensor_data = {"Middle": dis_sensor["distanceSensorMiddle"].getValue(),
                       "Left": dis_sensor["distanceSensorLeft"].getValue(),
                       "Right": dis_sensor["distanceSensorRight"].getValue()}
    
    print("X = {0:.3f}".format(gps_data[0]), 
          "Y = {0:.3f}".format(gps_data[1]), 
          "Z = {0:.3f}".format(gps_data[2]), sep="\t")
    
    print("Distance Sensor Data")
    print("Left: {0:.3f}".format(dis_sensor_data["Left"]),
          "Middle: {0:.3f}".format(dis_sensor_data["Middle"]),
          "Right: {0:.3f}".format(dis_sensor_data["Right"]), sep="\t")
    
    print("Camera Detected Objects: {}".format(cam_objects_num))
    
    # First Reverse
    if reverse > 0:
        move_backward()
        reverse -= 1
        continue
        
    # After reverse, do rotate    
    if rotate > 0:
        if direction_of_rotation == 0:
            in_place_left_turn()
        else:
            in_place_right_turn()
        
        rotate -= 1
        continue
    
    # check distance sensors and setup rotation and reverse proccedures
    if (min(dis_sensor_data.values()) < 1000):
        stop()
        rotate = random.randint(0, 500)
        direction_of_rotation = random.randint(0, 1)
        reverse = 2;
        continue
    
    # if no obstacle infront, move forward
    else:
        move_forward()
