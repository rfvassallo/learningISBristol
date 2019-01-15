
In this lesson you are going to see how you can publish and consume messages of the RobotConfig type using the Intelligent Space infrastructure.
This type of message will be used in the future to control the robot movement.

###
* consume_robot_command.py

  Connet to the broker, create a channel and subscribe to the message topic, so it can consume text messages whenever they are published under such topic.

###

* publish_robot_command.py

###

  This program illustrates how you can send messages containing velocity commands.
  Connet to the broker, create a channel, create a message that carries angular and linear velocities, and publish the message under a topic using the channel. Anyone subscribed in the topic will receive a message whenever it is published.


