from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 30
th = 150
pressed = 0

async def getpos(robot):
    return await robot.get_position()

def front_obstacle_close(sensors): # will return a value that is greater than the threshold for the front sensor
    return sensors[3] > th #double check this logic bc idk if its close or far which is what I think Huyen was concerned with

def right_obstacle_close(sensors):
    return sensors[6] > th

def left_obstacle_close(sensors):
    return sensors[0] > th

## Rerun Press left button ##
@event(robot.when_touched, [True, False])  # (.) button.
async def touched(robot):
    global pressed
    await robot.set_lights_on_rgb(0, 0, 255)
    pressed = 1


@event(robot.when_play)
async def play(robot):
    n_s = 0
    
    while True:
        sensors = (await robot.get_ir_proximity()).sensors

        print(n_s)

        # State 0: forward
        if n_s == 0:
            
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1

        # State 1: front obstacle?
        elif n_s == 1:
            
            if front_obstacle_close(sensors) == True:
                await robot.move(-5)
                await robot.turn_right(90)
                n_s = 2

        # State 2: look for opening 1
        elif n_s == 2:
            
            await robot.set_wheel_speeds(speed, speed)
            #if sensors[6] < th: Really confused why this is the working logic. Is it the higher the value the closer it is??
            if left_obstacle_close(sensors) == False: # I don't think this is how this works, but we'll try it
                await robot.move(30)
                await robot.turn_left(90)

                openingpos1 = await getpos(robot) # grabs the pos of the opening
                await robot.move(50)
                endingpos1 = await getpos(robot) # grabs the pos of the end of the opening? Or is this the final part
                n_s = 3

        # State 3: look for wall
        elif n_s == 3:
            
            await robot.set_wheel_speeds(speed, speed)
            if front_obstacle_close(sensors) == True: 
                await robot.move(-20)
                await robot.turn_left(90)
                
                n_s = 4
        
        # State 4: look for opening 2
        elif n_s == 4:
            
            await robot.set_wheel_speeds(speed, speed)
            if right_obstacle_close(sensors) == False: # is the wall detected?
                await robot.move(30)
                await robot.turn_right(90)
                
                openingpos2 = await getpos(robot)
                await robot.move(50)
                endingpos2 = await getpos(robot)

                n_s = 5
        
        elif n_s == 5: #kinda temporary/experimental code for the final button press.
            
            await robot.reset_navigation()
            await robot.set_wheel_speeds(0, 0)
            
            if pressed == 1:
                
                await robot.navigate_to(openingpos1)
                await robot.navigate_to(endingpos1)
                
                await robot.navigate_to(openingpos2)
                await robot.navigate_to(endingpos2)


robot.play()