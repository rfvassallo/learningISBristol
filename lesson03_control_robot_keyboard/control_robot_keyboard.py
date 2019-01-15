import curses
import os
from is_msgs.robot_pb2 import RobotConfig
from is_wire.core import Channel, Message, Subscription


# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.23:30000")
# Create a subscription 
subscription = Subscription(channel)

def makeMessage (linear,angular):
  # Create message with command for 
  # linear and angular velocities  
  message = Message()
  robotConfig = RobotConfig()
  robotConfig.speed.linear = linear
  robotConfig.speed.angular = angular
  message.pack(robotConfig)
  message.topic = "RobotGateway.0.SetConfig"
  # Public message
  channel.publish(message)

# Using keypad to control the robot with some 
# feedback on the screen

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0, 10, "Hit 'q' to quit")
stdscr.refresh()

key = ''
linear,angular = 0,0

while key != ord('q'):
    key = stdscr.getch()
    stdscr.addch(20, 25, key)
    stdscr.refresh()


    if key == curses.KEY_UP:
       stdscr.addstr(3, 20, "Up   ")
       linear = 10
       angular = 0

    elif key == curses.KEY_DOWN:
       stdscr.addstr(3, 20, "Down ")
       linear = -10
       anglar = 0

    elif key == curses.KEY_LEFT:
       stdscr.addstr(3, 20, "Left ")
       linear = 0
       angular = 0.3

    elif key == curses.KEY_RIGHT:
       stdscr.addstr(3, 20, "Right  ")
       linear = 0
       angular = -0.3
    elif key == ord('s'):
       stdscr.addstr(3, 20, "Stop   ")
       linear = 0
       angular = 0


    makeMessage(linear,angular)

curses.endwin()
makeMessage(0,0)

