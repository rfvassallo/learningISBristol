import qi



session = qi.Session()
try:
    session.connect("tcp://10.10.0.111:9559")
except RuntimeError:
    print ("Can't connect to Naoqi")

motion_service  = session.service("ALMotion")
result = motion_service.getRobotPosition(True)


try: 
    while True:
        result = motion_service.getRobotPosition(True)
        print ("Odometry - Robot API")
        print (result)
        print ("#############################")

except KeyboardInterrupt:
    pass


