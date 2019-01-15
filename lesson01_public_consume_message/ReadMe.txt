
In this lesson you are going to see how you can publish and consume messages using the Intelligent Space infrastructure.

###
* consume_message_text.py

  Connet to the broker, create a channel and subscribe to the message topic, so it can consume text messages whenever they are published under such topic.

###

* consume_message_tensor.py

  This program illustrates how you can send matricial data, as tensors, using messages.
  Connets to the broker, creates a channel and subscribes to the message topic, so it can consume tensor data embedded in messages whenever they are published under such topic.

###

* publish_message_text.py

  Connet to the broker, create a channel, create a message and publish it under a topic using the channel. Anyone subscribed in the topic will receive a message whenever it is published.

###

* publish_message_tensor.py

  This program illustrates how you can send matricial data, as tensors, using messages.
  Connet to the broker, create a channel, create a message that carries matricial data using tensors and publish the message under a topic using the channel. Anyone subscribed in the topic will receive a message whenever it is published.

###

* np_pb.py 

  Auxiliary program to convert matrix to tensor and vice-versa.

###
