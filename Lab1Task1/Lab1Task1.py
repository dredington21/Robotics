"""testing controller."""

# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import time
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1_Task1.xml'
robot.load_environment(maze_file)


# Move robot to a random staring position listed in maze file
robot.move_to_start()
distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()

pizza = int (0)
#startPos = float (0)
x = float(0)
y= float(0)
distWant = int(0)
position = int (0)
begin = time.time()
vOld = float(0)


def goStraight():
    #startPos = distance_front_left_wheel_traveled
    #while robot.wheel_radius * robot.get_front_left_motor_encoder_reading() != x +startPos:
    robot.set_right_motors_velocity(20)
    robot.set_left_motors_velocity(20)
        #break
       # if distance_front_left_wheel_traveled >= x + startPos:
          #  robot.stop
         #   break
    return 0    
    #if distance_front_left_wheel_traveled != s  tartPos + x:
       # robot.stop()
        #break

def turnRight():
    tacos =(robot.get_compass_reading())
    while(robot.get_compass_reading()!= tacos+90):
        print (robot.get_compass_reading())
        robot.set_right_motors_velocity(-20)
        robot.set_left_motors_velocity(+20) 
        return 0
def goCircle(y):
    robot.set_right_motors_velocity(50*(y-.026))
    robot.set_left_motors_velocity(50*(y+.026))
    return 0
 #Main Control Loop for Robot



       
while robot.experiment_supervisor.step(robot.timestep) != -1:

    #print("Max rotational motor velocity: ", robot.max_motor_velocity)

     #Reads and Prints Distance Sensor Values
    #print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
    #print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
    #print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
    #print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

     #Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())

   # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    #print("Lidar Front Reading", robot.get_lidar_range_image()[400])
    #print("Lidar Right Reading", robot.get_lidar_range_image()[600])
   # print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
   # print("Lidar Left Reading", robot.get_lidar_range_image()[200])

    
   
    
    
    # Calculates distance the wheel has turned since beginning of simulation
    
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    robot_VelocityL = robot.wheel_radius * (robot.get_front_left_motor_encoder_reading() / (time.time()-begin))
    robot_VelocityR = robot.wheel_radius * (robot.get_front_right_motor_encoder_reading() / (time.time()-begin))
    #print("Current Velocity is" + str(robot.get_front_left_motor_encoder_reading/pulse))

    # Sets the robot's motor velocity to 20 rad/sec
    if position ==0:
        robot.set_right_motors_velocity(20)
        robot.set_left_motors_velocity(20)
        print("Distance = 2.5m")
        print("Time = 1.94")
        print("VLi " + str(robot_VelocityL))
        print("VRi " + str(robot_VelocityR))
        position +=1

    if position ==1:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled >2.5:
            goCircle(.1)
            print("Distance = pi/4m")
            print("Time = 0.49")
            print("VLi " + str(robot_VelocityL))
            print("VRi " + str(robot_VelocityR))
            position +=1
            
    if position ==2:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled > 3.5:
            goStraight()
            print("Distance = 2m")
            print("Time = 1.55")
            print("VLi " +str(robot_VelocityL))
            print("VRi " +str(robot_VelocityR))
            position +=1
            
    if position ==3:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled >5.5:
            goCircle(.1)
            print("Distance = pi/4m")
            print("Time = 0.49")
            print("VLi " +str(robot_VelocityL))
            print("VRi " +str(robot_VelocityR))
            position +=1
            
    if position ==4:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled >6.5:
            goStraight()
            print("Distance = 1m")
            print("Time = 0.78")
            print("VLi " +str(robot_VelocityL))
            print("VRi " +str(robot_VelocityR))
            position +=1
            
    if position ==5:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled > 7.5:
            goCircle(.35)
            print("Distance = 1.5pi/2")
            print("Time = 1.46")
            print("VLi " +str(robot_VelocityL))
            print("VRi " +str(robot_VelocityR))
            position +=1
    if position ==6:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled > 10:
            goCircle(.1)
            print("Distance = 3pi/4")
            print("Time = 0.73")
            print("VLi " +str(robot_VelocityL))
            print("VRi " +str(robot_VelocityR))
            position +=1
            robot.stop
    if position ==7:
        robot.stop
        if vOld != robot_VelocityL:
            vOld = robot_VelocityL
            print(str(robot_VelocityL))
        if distance_front_left_wheel_traveled >12:
            robot.set_right_motors_velocity(0)
            robot.set_left_motors_velocity(0)
            break
   
  

    #if distance_front_left_wheel_traveled > 1.5:
        #if pizza == 0: 
     #   turnRight()
      #  pizza +=1