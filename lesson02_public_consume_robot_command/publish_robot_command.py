from is_wire.core import Channel, Message, Subscription
from is_msgs.robot_pb2 import RobotConfig


#Create a channel to connet to the broker
channel = Channel("amqp://10.10.2.20:30000")

# Create Message
message = Message()

# Create and set Robot Configuration
robotConfig = RobotConfig()
robotConfig.speed.linear = 40.0
robotConfig.speed.angular = 0.3

# Insert Robot Configuration in the Message
message.pack(robotConfig)

# Define message topic
Topic = "Aula2"
message.topic = Topic  

# Publish the message
channel.publish(message)
