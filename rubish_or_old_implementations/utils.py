import sys
import cv2
import numpy as np

from is_wire.core import Logger
from is_msgs.image_pb2 import Image
from is_msgs.common_pb2 import Tensor, DataType
from google.protobuf.json_format import Parse
from options_pb2 import SkeletonsHeatmapOptions

def to_pb_image(input_image, encode_format='.jpeg', compression_level=0.8):
    if isinstance(input_image, np.ndarray):
        if encode_format == '.jpeg':
            params = [cv2.IMWRITE_JPEG_QUALITY, int(compression_level * (100 - 0) + 0)]
        elif encode_format == '.png':
            params = [cv2.IMWRITE_PNG_COMPRESSION, int(compression_level * (9 - 0) + 0)]
        else:
            return Image()
        cimage = cv2.imencode(ext=encode_format, img=input_image, params=params)
        return Image(data=cimage[1].tobytes())
    elif isinstance(input_image, Image):
        return input_image
    else:
        return Image()


def to_np(tensor):
    if len(tensor.shape.dims) != 2 or tensor.shape.dims[0].name != 'rows':
        return np.array([])

    shape = (tensor.shape.dims[0].size, tensor.shape.dims[1].size)
    if tensor.type == DataType.Value('INT32_TYPE'):
        return np.array(tensor.ints32, dtype=np.int32, copy=False).reshape(shape)
    if tensor.type == DataType.Value('INT64_TYPE'):
        return np.array(tensor.ints64, dtype=np.int64, copy=False).reshape(shape)
    if tensor.type == DataType.Value('FLOAT_TYPE'):
        return np.array(tensor.floats, dtype=np.float32, copy=False).reshape(shape)
    if tensor.type == DataType.Value('DOUBLE_TYPE'):
        return np.array(tensor.doubles, dtype=np.float64, copy=False).reshape(shape)
    return np.array([])


def load_options():
    log = Logger(name='LoadOptions')
    op_file = sys.argv[1] if len(sys.argv) > 1 else 'options.json'
    try:
        with open(op_file, 'r') as f:
            try:
                op = Parse(f.read(), SkeletonsHeatmapOptions())
                log.info('Options: \n{}', op)
            except Exception as ex:
                log.critical(
                    'Unable to load options from \'{}\'. \n{}', op_file, ex)
                sys.exit(-1)
            except:
                log.critical('Unable to load options from \'{}\'', op_file)
                sys.exit(-1)
    except Exception as ex:
        log.critical('Unable to open file \'{}\'', op_file)
        sys.exit(-1)
    
    message = op.DESCRIPTOR.full_name
    # validation
    if op.period_ms < 200:
        message += " 'period_ms' field must be equal or greater than 200. "
        message += "Given {}".format(op.period_ms)
        raise Exception(message)
    if op.period_ms > 1000:
        message += " 'period_ms' field must be equal or less than 1000. "
        message += "Given {}".format(op.period_ms)
        raise Exception(message)
    
    return op