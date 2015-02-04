"""autogenerated by genpy from canusb/CAN.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class CAN(genpy.Message):
  _md5sum = "c2f02b75374baa8642f5921662ba71d1"
  _type = "canusb/CAN"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """time timestamp
uint16 stdId
int16 extId
uint8[] data

"""
  __slots__ = ['timestamp','stdId','extId','data']
  _slot_types = ['time','uint16','int16','uint8[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,stdId,extId,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CAN, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.stdId is None:
        self.stdId = 0
      if self.extId is None:
        self.extId = 0
      if self.data is None:
        self.data = ''
    else:
      self.timestamp = genpy.Time()
      self.stdId = 0
      self.extId = 0
      self.data = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_2IHh.pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.stdId, _x.extId))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.stdId, _x.extId,) = _struct_2IHh.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_2IHh.pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.stdId, _x.extId))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.stdId, _x.extId,) = _struct_2IHh.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2IHh = struct.Struct("<2IHh")
