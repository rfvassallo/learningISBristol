from is_wire.core import Channel, Message, Subscription
from is_msgs.robot_pb2 import RobotConfig


# Create a channel to connect to the broker

channel = Channel("amqp://10.10.2.20:30000")

# Create a subscription 
subscription = Subscription(channel)

# Subscribe to a topic
Topic = "Aula2"
subscription.subscribe(Topic)



while True:
    # Keep listening to the channel 
    # to consume messages
    message = channel.consume()
    print (message.unpack(RobotConfig))
    


