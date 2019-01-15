import sys
import numpy as np
from is_wire.core import Logger
from is_msgs.common_pb2 import Tensor, DataType
from google.protobuf.json_format import Parse



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
                op = Parse(f.read(), ObjectAnnotationsTransformerOptions())
                log.info('Options: \n{}', op)
            except Exception as ex:
                log.critical('Unable to load options from \'{}\'. \n{}', op_file, ex)
            except:
                log.critical('Unable to load options from \'{}\'', op_file)
    except Exception as ex:
        log.critical('Unable to open file \'{}\'', op_file)

    return op
