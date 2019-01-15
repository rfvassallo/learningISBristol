from __future__ import print_function
from is_wire.core import Channel, Subscription, Message
from np_pb import to_tensor, to_np
from is_msgs.common_pb2 import Tensor
import numpy as np 



# This program shows how to publish a message using 
# the Intelligent Space framework

# Connect to the broker
channel = Channel("amqp://10.10.2.23:30000")

# Create the message that is a matrix
message = Message()
test = np.array([[1, 2, 3, 4],[ 5, 6, 7, 8],[ 9, 10, 11, 12],[ 13, 14, 15, 16]])
# Convert to Tensor
test2 = to_tensor(test)
message.pack(test2)


# Define topic name
Topic= "Aula01"
message.topic = Topic  

# Publish message under the topic
channel.publish(message)


