#!/usr/bin/env python3

"""
Utility functions for working with PointCloud2 messages.
"""

import sys
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a PointCloud2 message.
    
    Args:
        cloud (PointCloud2): The point cloud to read from
        field_names (list): The names of fields to read. If None, read all fields
        skip_nans (bool): If True, skip NaN values
        uvs (list): If specified, only read points at these pixel coordinates
        
    Returns:
        generator: Generator yielding the point data
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a PointCloud2'
    
    if field_names is None:
        field_names = [f.name for f in cloud.fields]
    
    # Convert the fields to a list of tuples (field_name, datatype, offset)
    fields = []
    for field_name in field_names:
        fields.append((field_name, _get_struct_fmt(cloud, field_name), _get_field_offset(cloud, field_name)))
    
    # Compute the point step and point format
    point_step, point_fmt = _get_point_fmt(cloud)
    
    # Read the points
    if uvs:
        for u, v in uvs:
            yield _read_point(cloud, u, v, point_step, point_fmt, fields, skip_nans)
    else:
        for i in range(cloud.width * cloud.height):
            yield _read_point_from_buffer(cloud.data, i, point_step, point_fmt, fields, skip_nans)

def _get_struct_fmt(cloud, field_name):
    """Get the struct format for a field."""
    for f in cloud.fields:
        if f.name == field_name:
            datatype, datatype_size = _DATATYPES[f.datatype]
            return datatype
    raise ValueError('Field %s not found in PointCloud2' % field_name)

def _get_field_offset(cloud, field_name):
    """Get the offset of a field."""
    for i, f in enumerate(cloud.fields):
        if f.name == field_name:
            return f.offset
    raise ValueError('Field %s not found in PointCloud2' % field_name)

def _get_point_fmt(cloud):
    """Get the format and step of a point."""
    fmt = '>'
    step = 0
    for f in cloud.fields:
        datatype, datatype_size = _DATATYPES[f.datatype]
        fmt += datatype
        step += datatype_size
    return step, fmt

def _read_point(cloud, u, v, point_step, point_fmt, fields, skip_nans=False):
    """Read a point from a PointCloud2 message at pixel coordinates (u, v)."""
    width = cloud.width
    height = cloud.height
    point_step = cloud.point_step
    row_step = cloud.row_step
    
    if v >= height or u >= width:
        raise IndexError('Point (%d, %d) is out of bounds (%d, %d)' % (u, v, width, height))
    
    data_idx = row_step * v + point_step * u
    return _read_point_from_buffer(cloud.data, data_idx, point_step, point_fmt, fields, skip_nans)

def _read_point_from_buffer(data_buffer, data_idx, point_step, point_fmt, fields, skip_nans=False):
    """Read a point from a buffer."""
    point_data = data_buffer[data_idx:data_idx+point_step]
    
    # Unpack the point data
    point = list(struct.unpack(point_fmt, point_data))
    
    # Extract the fields
    result = []
    for field_name, datatype, offset in fields:
        result.append(point[offset])
    
    # Skip NaNs if requested
    if skip_nans and any(np.isnan(result)):
        return None
    
    return result
