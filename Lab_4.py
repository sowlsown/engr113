from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 25
th = 150

@event(robot.when_play)
async def play(robot):
    n_s = 0
    await robot.reset_navigation()  # get original pos
    
    while True:    

        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await robot.set_wheel_speeds(speed, speed)
            n_s = 1
        
        elif n_s == 1:
            if sensors[5] > th or sensors[6] > th:
                n_s = 2
            elif sensors[1] > th or sensors[0] > th:
                n_s = 3
        
        elif n_s == 2:
            if sensors[6] > th or sensors[5] > th and sensors[3]: # Checks the right wall
                await robot.turn_left(180)
                await robot.set_wheel_speeds(speed, speed)
        
        elif n_s == 3:
            if sensors[1] > th or sensors[0] and sensors[3]:
                await robot.turn_right(180)
                await robot.set_wheel_speeds(speed, speed)
                
robot.play()