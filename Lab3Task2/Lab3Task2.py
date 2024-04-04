# Changes Working Directory to be at the root of FAIRIS-Lite
import os
import time
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot

# Create the robot instance.
robot = MyRobot()

# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab3/Lab3_Task2_1.xml'
robot.load_environment(maze_file)

# Move robot to a random staring position listed in maze file
robot.move_to_start()



def Saturation(v):
   if v>= 26:
      v=26
   if v <=-26:
      v=-26
   return v

    
          
def PIDf(x, k):
    fd = min(robot.get_lidar_range_image()[350:450])
    error = fd - x
    return Saturation(k*error)


def wallFollowing(wD,K,wall):
    vFront = PIDf(.3,20)
    rd = min(robot.get_lidar_range_image()[500:700])
    ld = min(robot.get_lidar_range_image()[100:300])
    if wall == 'R':
        error = wD-rd
        if rd < wD:
            vRight = Saturation(vFront)
            vLeft = Saturation(vFront - abs(K*error))
        elif rd > wD:
            vRight = Saturation(vFront - abs(K*error))
            vLeft = Saturation(vFront)
        elif ld < .2:
            error = .2-ld
            vRight = Saturation(vFront - abs(K*error))
            vLeft = Saturation(0)
        else:
            vRight = vFront
            vLeft =  vFront
    else:
        error = wD-ld
        if ld < wD:
            vRight = Saturation(vFront - abs(K*error))
            vLeft = Saturation(vFront)
        elif ld > wD:
            vRight = Saturation(vFront)
            vLeft = Saturation(vFront - abs(K*error))
        elif rd < .2:
            error = .2-rd
            vRight = Saturation(0)
            vLeft = Saturation(vFront - abs(K*error))
        else:
            vRight = vFront
            vLeft =  vFront
        
    return vLeft, vRight

state = 1
flag = 1
flag2 = 1
end = 0
while robot.experiment_supervisor.step(robot.timestep) != -1:
    position = [0,1,0]
    
    
       

    #print("Max rotational motor velocity: ", robot.max_motor_velocity)

    # Reads and Prints Distance Sensor Values
    #print("Front Left Distance Sensor: ", robot.get_front_left_distance_reading())
    #print("Front Right Distance Sensor: ", robot.get_front_right_distance_reading())
    #print("Rear Left Distance Sensor: ", robot.get_rear_left_distance_reading())
    #print("Rear Right Distance Sensor: ", robot.get_rear_right_distance_reading())

    # Reads and Prints Robot's Encoder Readings
    #print("Motor Encoder Readings: ", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    #print("Lidar Front Reading", robot.get_lidar_range_image()[400])
    #print("Lidar Right Reading", robot.get_lidar_range_image()[600])
    #print("Lidar Rear Reading", robot.get_lidar_range_image()[0])
    #print("Lidar Left Reading", robot.get_lidar_range_image()[200])
    #print(robot.get_compass_reading())

    # Sets the robot's motor velocity to 20 rad/sec
    #robot.set_right_motors_velocity(20)
    #robot.set_left_motors_velocity(20)

    # Calculates distance the wheel has turned since beg vinning of simulation


    wall = 'L'
    objects = robot.rgb_camera.getRecognitionObjects()
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    fDistance = min(robot.get_lidar_range_image()[350:450])

    for obj in objects:
        id = obj.getId()
        #print(id)
        position = obj.getPosition()
        #print(position[0])
        orientation = obj.getOrientation()
        #print(orientation)
        size = obj.getSize()
        #print(size)
        position_on_image = obj.getPositionOnImage()
        #print(position_on_image)
        size_on_image = obj.getSizeOnImage()
        #print(size_on_image[0])
        number_of_colors = obj.getNumberOfColors()
        #print(number_of_colors)
        colors = obj.getColors()
        #print(colors[0])
        model = obj.getModel()
       # print(model)   
        
   

    if abs(position[1])<.4 and end !=1:
        state = 0
        flag2 = 0
        
    if state == 0 and end !=1:
        move = PIDf(.3,10)
        robot.set_right_motors_velocity(move)
        robot.set_left_motors_velocity(move)
    
    if fDistance <=.5:
        state = 1
      #  if flag2 ==0:
            #if wall =='R':
             #   robot.set_right_motors_velocity(-20)
             #   robot.set_left_motors_velocity(20)
           # else:
              #  robot.set_right_motors_velocity(20)
               # robot.set_left_motors_velocity(-20)

    if state ==1 and end !=1:
        vLeft, vRight = wallFollowing(0.3,35, wall)
        robot.set_right_motors_velocity(vRight)
        robot.set_left_motors_velocity(vLeft)

        if fDistance < 0.6 and wall == 'R':
            robot.set_right_motors_velocity(20)
            robot.set_left_motors_velocity(-20)
    
    
        if fDistance <0.6 and wall == 'L':
            robot.set_right_motors_velocity(-20)
            robot.set_left_motors_velocity(20)  

    if size_on_image[0]>=298 and flag ==1:
        flag +=1
        end =1
        robot.set_right_motors_velocity(1)
        robot.set_left_motors_velocity(-1)
        print("entering end")

    if abs(position[1])<.1 and end ==1:
        state = 0
        end = 0
        
        

    if size_on_image[0]>=450 and fDistance <=.5:
        robot.set_right_motors_velocity(0)
        robot.set_left_motors_velocity(0)
        print("all done")
        state = 0
        flag =0
        break


 





