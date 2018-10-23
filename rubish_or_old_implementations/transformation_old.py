import numpy as np
from is_wire.core import Channel, Message, Subscription
from is_msgs.common_pb2 import Tensor
from is_msgs.camera_pb2 import FrameTransformation
from is_msgs.image_pb2 import Vertex, ObjectAnnotations
from utils import to_np

class TransformationFetcher:
  def __init__(self, broker_uri=None):
    self.channel = Channel() if broker_uri is None else Channel(broker_uri)
    self.subscription = Subscription(self.channel)
    self.transformations = {}

  def get_transformation(self, _from, _to):
    if _from in self.transformations and _to in self.transformations[_from]:
        return self.transformations[_from][_to]
    if self._request_transformation(_from, _to):
      return self.transformations[_from][_to]
    return None

  def _request_transformation(self, _from, _to):
    topic = 'FrameTransformation.{}.{}'.format(_from, _to)
    self.subscription.subscribe(topic)
    try:
      msg = self.channel.consume(timeout=5.0)
      self.subscription.unsubscribe(topic)
    except:
      self.subscription.unsubscribe(topic)
      return False
 #   tf = msg.unpack(Tensor)
    frameTrans = msg.unpack(FrameTransformation)
    tf = frameTrans.tf

    if _from not in self.transformations:
      self.transformations[_from] = {}
    self.transformations[_from][_to] = to_np(tf)
    return True


def vertex_to_np(v, homogeneo=False):
    if homogeneo:
        return np.array([v.x, v.y, v.z, 1.0])[:, np.newaxis]
    else:
        return np.array([v.x, v.y, v.z])[:, np.newaxis]


def np_to_vertex(v):
    return Vertex(x=v[0], y=v[1], z=v[2])


def transform_vertex(v, tf):
  new_v = np.matmul(tf, vertex_to_np(v, homogeneo=True))
  return np_to_vertex(new_v)


def transform_object_annotations(objs, tf, ref_id):
  new_objs = ObjectAnnotations(frame_id=ref_id)
  for obj in objs.objects:
    new_obj = new_objs.objects.add()
    for keypoint in obj.keypoints:
      new_keypoint = new_obj.keypoints.add()
      new_keypoint.id = keypoint.id
      new_keypoint.position.CopyFrom(transform_vertex(keypoint.position, tf))
  return new_objs