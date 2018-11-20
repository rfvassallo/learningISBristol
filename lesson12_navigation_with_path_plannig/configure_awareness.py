import qi
import time



session = qi.Session()
try:
    session.connect("tcp://10.10.0.111:9559")
except RuntimeError:
    print ("Can't connect to Naoqi")

awareness_service  = session.service("ALBasicAwareness")
#result = motion_service.getRobotPosition(True)


try: 
    
    #result = motion_service.getRobotPosition(True)
    awareness_service.setEnabled(False)
    print ("Awareness Disabled")
    print ("#############################")

    time.sleep(10)
    awareness_service.setEnabled(True)
    print ("Awareness Enabled")
    print ("#############################")


except KeyboardInterrupt:
    pass


