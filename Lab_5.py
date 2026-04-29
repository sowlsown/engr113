from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())
speed = 25
th = 150

async def nav_to_waypoint(x, y, h = 90):
    mag = 30.48
    x = x * mag
    y = y * mag
    await robot.navigate_to(x, y, h)

@event(robot.when_play)
async def play(robot):
    n_s = 0
    await robot.reset_navigation()  # get original pos
    # place the robot at 0,0 pls
    
    while True:    

        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await nav_to_waypoint(0, 0, 90)
            waypoint1 = await robot.get_position() # 1, 1
            n_s = 1
        
        elif n_s == 1: #will move to the second waypoint
            
            await nav_to_waypoint(0, 4, 0)
            waypoint2 = await robot.get_position()
        
            n_s = 2
            
        elif n_s == 2: # will move to the first opening to room 101
            
            await nav_to_waypoint(2, 4, 90)
            waypoint3 = await robot.get_position()
            n_s = 3
        
        elif n_s == 3: # right outside room 101
            
            await nav_to_waypoint(2, 6, 180)
            waypoint4 = await robot.get_position()
            
            n_s = 4
        
        elif n_s == 4: # room 101
            
            await nav_to_waypoint(0, 6, 0)
            rm101 = await robot.get_position()
            
            n_s = 5
            
        elif n_s == 5: # main pipeline
            
            await nav_to_waypoint(2, 7, 270)
            await nav_to_waypoint(2, 5, 180) #should be back in the main hallway
            
            n_s = 6
            
        elif n_s == 6: #go home
            
            await nav_to_waypoint(0, 4, 270)
            await nav_to_waypoint(0, 0, 90)
            n_s = 7
        
        elif n_s == 7:
            
            await hand_over() #ends code