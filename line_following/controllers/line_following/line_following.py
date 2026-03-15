from controller import Robot
import numpy as np 

robot = Robot()
timestep = int(robot.getBasicTimeStep())

max_speed = 6.28

gs = []

for i in range(3):
    gs.append(robot.getDevice('gs'+str(i)))
    gs[-1].enable(timestep)

#for gs: higher value = white, lower value = black

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotorSensor = robot.getDevice('left wheel sensor')
rightMotorSensor = robot.getDevice('right wheel sensor')
leftMotorSensor.enable(timestep)
rightMotorSensor.enable(timestep)


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

phil = max_speed
phir = max_speed

prev_distance_left = 0
prev_distance_right = 0
x = 0
omega = 0 
radius = 0.0201
wheel_distance = 0.052

while robot.step(timestep) != -1:
    g = [] 
    dt = timestep/1000.0
    for gsensor in gs: 
        g.append(gsensor.getValue())
        
    # Angular distance from encoders 
    leftSensor, rightSensor = leftMotorSensor.getValue(), rightMotorSensor.getValue()
    print("wheel sensor (left,right)" , leftSensor, rightSensor)
    
    # Angular speed from differentiation 
    speedLeft = (leftSensor - prev_distance_left)/dt
    speedRight = (rightSensor - prev_distance_right)/dt
    print("wheel speed (left,right)" , speedLeft, speedRight)
    
    # Update distance for next time step
    prev_distance_left = leftSensor 
    prev_distance_right = rightSensor
    
    # get x_distance: 
    v = (radius)*(speedLeft + speedRight)/2 
    x += v*dt
    print("Distance travelled: ", x)
    
    # get omega: 
    w = radius * (speedRight - speedLeft) / wheel_distance
    omega += w * dt
    print("Orientation: ", omega)
    
    
    print(g)
 
    
    if g[0] > 500 and g[1] < 350 and g[2] > 500: 
        phil = max_speed
        phir = max_speed
    
    elif g[2] < 500: 
        phil = 0.25*max_speed
        phir = - 0.1* max_speed
        
        
    elif g[0] < 500: 
        phil = -0.1*max_speed
        phir = 0.25* max_speed
        
    # if g[0] < 300 and g[1] < 300 and g[2] < 300:
        # phil = 0 
        # phir = 0 
        # break 
        
    leftMotor.setVelocity(phil)
    rightMotor.setVelocity(phir)
    pass


