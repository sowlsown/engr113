from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

speed = 30
th = 150
flag = 0

async def stop(robot):
    speed = 0

async def forward(robot):
    speed = 30
    await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, speed)

@event(robot.when_play)
async def play(robot):
  await robot.set_wheel_speeds(speed, speed)
  left_s = 0
  right_s = 0
  
  while True:
    sensors = (await robot.get_ir_proximity()).sensors

    if sensors[3] > th:
      await stop(robot)
    
    if sensors[0] > th:
      await forward(robot)
      left_s = 1
      
    elif sensors[6] > th:
      await forward(robot)
      right_s = 1

    elif left_s == 1:
      await forward(robot)
      if sensors[0] < th:
        await robot.turn_left(90)
        await forward(robot)

    elif right_s == 1:
      await forward(robot)
      if sensors[6] < th:
        await robot.turn_right(90)
        await forward(robot)
        

    
    
    
