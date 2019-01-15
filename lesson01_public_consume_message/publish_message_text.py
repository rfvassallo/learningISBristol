from __future__ import print_function
from is_wire.core import Channel, Subscription, Message


# This program shows how to publish a message using 
# the Intelligent Space framework

# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")

# Create the message
message = Message()
message.body = "Teste"

# Print the message
print ("Sent Message:")
print (message.body)

# Define topic name
Topic= "Aula01"
message.topic = Topic  

# Publish message under the topic
channel.publish(message)


