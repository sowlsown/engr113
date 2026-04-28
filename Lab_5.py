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
    # place the robot at 0,0 pls
    
    while True:    

        sensors = (await robot.get_ir_proximity()).sensors
        if n_s == 0:
            await robot.navigate_to(30.48, 30.48, 90) # Kind of a base get position. This will be the first 'waypoint'
                                    # 1, 1
            waypoint1 = await robot.get_position()
            n_s = 1
        
        elif n_s == 1: #will move to the second waypoint
            
            await robot.navigate_to(30.48, 152.4, 0) # 1, 5
            waypoint2 = await robot.get_position()
        
            n_s = 2
            
        elif n_s == 2: # will move to the first opening to room 101
            
            await robot.navigate_to(91.44, 152.4, 90) #3, 5
            waypoint3 = await robot.get_position()
            n_s = 3
        
        elif n_s == 3: # right outside room 101
            
            await robot.navigate_to(91.44, 213.36, 180) # 3, 7
            waypoint4 = await robot.get_position()
            
            n_s = 4
        
        elif n_s == 4: # room 101
            
            await robot.navigate_to(30.48, 213.36, 0) #1, 7
            rm101 = await robot.get_position()
            
            n_s = 5
            
        elif n_s == 5: # main pipeline
            
            await robot.navigate_to(91.44, 213.36, 270) #3, 7
            await robot.navigate_to(waypoint3.x, waypoint3.y, 180) #should be back in the main hallway
            
            n_s = 6
            
        elif n_s == 6: #go home
            
            await robot.navigate_to(waypoint2.x, waypoint2.y, 270)
            await robot.navigate_to(waypoint1.x, waypoint1.y, 90)
            n_s = 7
        
        elif n_s == 7:
            
            await hand_over() #ends code