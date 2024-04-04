# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import time
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab2/Lab2_Task1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()


def PID(x, k):
    if abs(robot.get_lidar_range_image()[400] - x) > 0.01:  # Adjust the tolerance as needed
        cmax = 20
        cmin = -20
        start = robot.get_lidar_range_image()[400]
        
        # Proportional control
        et = x - start
        ut = k * et

        # Saturation limits
        ut = max(min(ut, cmax), cmin)
        print (ut)

        # Set motor velocities  q
        robot.set_right_motors_velocity(-ut)
        robot.set_left_motors_velocity(-ut)

        # Introduce a small delay
        time.sleep(0.1)

    # Stop the motors when the loop exits
    #robot.set_right_motors_velocity(0)
    #robot.set_left_motors_velocity(0)

    return

def test():
    if distance_front_left_wheel_traveled < 2:
    #begin = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    #while(robot.wheel_radius * robot.get_front_left_motor_encoder_reading()!= begin +2):
        robot.set_right_motors_velocity(5)
        robot.set_left_motors_velocity(5)
    
    





# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:

    print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Distance Sensor Values
    print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
    print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
    print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
    print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    print("Lidar Front Reading", robot.get_lidar_range_image()[400])
    print("Lidar Right Reading", robot.get_lidar_range_image()[600])
    print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
    print("Lidar Left Reading", robot.get_lidar_range_image()[200])

    # Sets the robot's motor velocity to 20 rad/sec
    #robot.set_right_motors_velocity(20)
    #robot.set_left_motors_velocity(20)

    # Calculates distance the wheel has turned since beg vinning of simulation
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()

    PID(0.5,10)
    if abs(robot.get_lidar_range_image()[400] - .5) <= 0.01:
        break
    

    #test()

    #Stops the robot after the robot moves a distance of 1.5 meters
 





