from is_wire.core import Channel, Message, Subscription
from google.protobuf.struct_pb2 import Struct
import time



def awarenessOff(channel):
    '''
    message = Message()
    condition = Struct()
    condition["enabled"] = False
    message.pack(condition)
    message.topic = "RobotGateway.0.SetAwareness"
    channel.publish(message) 
    '''
    message = Message()
    message.topic = "RobotGateway.0.PauseAwareness"
    channel.publish(message) 
    

def awarenessOn(channel):
    '''
    message = Message()
    condition = Struct()
    condition["enabled"] = True
    message.pack(condition)
    message.topic = "RobotGateway.0.SetAwareness"
    channel.publish(message)
    '''
    message = Message()
    message.topic = "RobotGateway.0.ResumeAwareness"
    channel.publish(message) 
    





# Create a channel to connect to the broker
channel = Channel("amqp://10.10.2.20:30000")
# Create a subscription 
subscription = Subscription(channel)



try: 
    
    #result = motion_service.getRobotPosition(True)
    awarenessOff(channel)
    print ("Awareness Disabled")
    print ("#############################")

    time.sleep(10)



    awarenessOn(channel)
    print ("Awareness Enabled")
    print ("#############################")


except KeyboardInterrupt:
    pass


