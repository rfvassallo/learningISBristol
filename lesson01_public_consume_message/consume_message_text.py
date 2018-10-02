from __future__ import print_function
from is_wire.core import Channel, Subscription, Message


# This program shows how to consume a message published
# in some topic using the Intelligent Space framework

# Connect to the broker
channel = Channel("amqp://10.10.2.20:30000")

# Subscribe to the channel
subscription = Subscription(channel)

# Define topic name
Topic= "Aula01"

# Subscribe to the topic
subscription.subscribe(Topic)


while True:
    # Keep listening to the channel
    message = channel.consume()

    # Print message whenever receive one
    print ("Received message:")
    print (message.body)
    

