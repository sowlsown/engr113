from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 25
th = 150
doorway_th = 100

LEFT = 0
DIAGONAL_LEFT = 2
FRONT = 3
DIAGONAL_RIGHT = 4
RIGHT = 6

@event(robot.when_play)
async def play(robot):
    
    n_s = 0 #main state
    tmp_s = 0    
    
    await robot.reset_navigation()  # get original pos
    
    while True:
        
        sensors = (await robot.get_ir_proximity()).sensors

        if n_s == 0:
            
            await robot.set_wheel_speeds(speed, speed)
            
            if sensors[FRONT] > th:
                await robot.set_wheel_speeds(0, 0)
                await robot.turn_left(90)
                
                n_s = 1
        
        elif n_s == 1:
            if sensors[RIGHT] > th:
                await robot.set_wheel_speeds(speed, speed)
                if sensors[FRONT] > th and sensors[RIGHT] > th:
                    
                    await robot.set_wheel_speeds(0, 0)
                    await robot.turn_right(180)
                    tmp_s = 1 #if it doesn't see the opening, then I want it to turn the other way and check the sensor.
                    n_s = 100 #just a random number to make sure it doesn't run the first state again                  
                    
            elif sensors[RIGHT] < th: #Goes until it detects the door
                await robot.set_wheel_speeds(0, 0)
                await robot.turn_right(90) #turns right to face the door
                
                n_s = 2                               
     
        elif tmp_s == 1: # will literally do what is written in the main state just opposite so not many comments
            await robot.set_wheel_speeds(speed, speed)
            
            if sensors[LEFT] < th:
                await robot.set_wheel_speeds(0, 0)
                await robot.turn_left(90)
                
                tmp_s = 2
        
        elif tmp_s == 2:
            tmp_s += 1
        
        elif tmp_s == 3:
            if sensors[FRONT] > th:
                await robot.turn_right(90)
                await robot.move(20)
                await robot.turn_left(90)
                
                tmp_s = 2
            
            elif sensors[DIAGONAL_RIGHT] > doorway_th:
                await robot.turn_right(90)
                await robot.move(20)
                await robot.turn_left(90)
                
                tmp_s = 2
            
            elif sensors[FRONT] < th and sensors[DIAGONAL_RIGHT] < doorway_th:
                await robot.set_wheel_speeds(speed, speed)
                
                n_s = 4
        
        elif n_s == 2: #lowkey just needed a state to increment the value so I can rerun this code
            n_s += 1
        
        elif n_s == 3: # check if the front sensor has something in front
            
            if sensors[FRONT] > th:
                await robot.turn_left(90)
                await robot.move(20) #make it just move a certain amount
                await robot.turn_right(90)
                
                n_s = 2
            
            elif sensors[DIAGONAL_RIGHT] > doorway_th: #check if sensor 4 is seeing something slightly further away
                await robot.turn_left(90)
                await robot.move(20)
                await robot.turn_right(90)
                
                n_s = 2
            
            elif sensors[FRONT] < th or sensors[DIAGONAL_RIGHT] < doorway_th: #dont see anything? drive and move on to next state
                await robot.set_wheel_speeds(speed, speed)
                n_s = 4
            
        elif n_s == 4:
            if sensors[LEFT] > th or sensors[RIGHT] > th:
                await robot.move(30)
                await robot.set_wheel_speeds(0, 0)
                break
            
robot.play()