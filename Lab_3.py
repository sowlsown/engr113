

from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 10
th = 150


async def forward(robot):
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, speed)


async def backoff_right(robot):
    await robot.set_lights_on_rgb(255, 80, 0)
    await robot.move(-20)
    await robot.turn_right(90)

async def backoff_left(robot):
    await robot.move(-20)
    await robot.turn_left(90)


def front_obstacle(sensors):
    print(sensors[3])
    return sensors[3] > th

def left_obstacle(sensors):
    return sensors[0] > th

def right_obstacle(sensors):
    return sensors[6] > th

@event(robot.when_play)
async def play(robot):
    
    n_s = 0
    await robot.get_position() # get original pos
    await forward(robot)
    while True:
        
        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0: #detects if theres a wall, backs off, turns, and then stops
        
            if front_obstacle(sensors):
                await backoff_right(robot)
                await robot.set_wheel_speeds(0, 0)
                n_s += 1
        
        elif n_s == 1: #will detect the wall, when theres no wall, it will turn left and stop
            
            if left_obstacle(sensors):
                await robot.set_wheel_speeds(speed, speed)
                if(sensors[0] < th):
                    await robot.move(30)
                    await robot.turn_left(90)
                    await robot.set_wheel_speeds(0, 0)
                    opening1 = await robot.get_position() #get position of opening 1
                    n_s = 2
                    
        elif n_s == 2: #it will move forward unless the robot detects there's something in front of it
            
            if (sensors[3] < th):
                await robot.set_wheel_speeds(speed, speed)
                if front_obstacle(sensors):
                    await backoff_left(robot)
                    await robot.set_wheel_speeds(0, 0)
                    n_s = 3
        
        elif n_s == 3: #This finds the opening and orients itself
            
            if right_obstacle(sensors):
                await robot.set_wheel_speeds(speed, speed)
                if(sensors[6] < th):
                    await robot.move(30)
                    await robot.turn_right(90)
                    await robot.set_wheel_speeds(0, 0)
                    opening2 = await robot.get_position() #get pos of opening 2
                    n_s = 4
        
        elif n_s == 4: #this will drive through the opening with the position
            
            if left_obstacle(sensors) == True or right_obstacle(sensors) == True:
                await robot.set_wheel_speeds(speed, speed)
                if left_obstacle(sensors) == False or right_obstacle(sensors) == False:
                    await robot.set_wheel_speeds(0, 0)
                    Final_Pos = await robot.get_position()
                    n_s = 5
        
        elif n_s == 5: #Want to make it just go to that position
            

robot.play()
