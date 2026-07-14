# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hamr_interfaces:msg/ReferenceTraj.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReferenceTraj(type):
    """Metaclass of message 'ReferenceTraj'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hamr_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hamr_interfaces.msg.ReferenceTraj')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__reference_traj
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__reference_traj
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__reference_traj
            cls._TYPE_SUPPORT = module.type_support_msg__msg__reference_traj
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__reference_traj

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReferenceTraj(metaclass=Metaclass_ReferenceTraj):
    """Message class 'ReferenceTraj'."""

    __slots__ = [
        '_x',
        '_y',
        '_roll',
        '_pitch',
        '_yaw',
        '_x_dot',
        '_y_dot',
        '_roll_dot',
        '_pitch_dot',
        '_yaw_dot',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'x': 'double',
        'y': 'double',
        'roll': 'double',
        'pitch': 'double',
        'yaw': 'double',
        'x_dot': 'double',
        'y_dot': 'double',
        'roll_dot': 'double',
        'pitch_dot': 'double',
        'yaw_dot': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.roll = kwargs.get('roll', float())
        self.pitch = kwargs.get('pitch', float())
        self.yaw = kwargs.get('yaw', float())
        self.x_dot = kwargs.get('x_dot', float())
        self.y_dot = kwargs.get('y_dot', float())
        self.roll_dot = kwargs.get('roll_dot', float())
        self.pitch_dot = kwargs.get('pitch_dot', float())
        self.yaw_dot = kwargs.get('yaw_dot', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.roll != other.roll:
            return False
        if self.pitch != other.pitch:
            return False
        if self.yaw != other.yaw:
            return False
        if self.x_dot != other.x_dot:
            return False
        if self.y_dot != other.y_dot:
            return False
        if self.roll_dot != other.roll_dot:
            return False
        if self.pitch_dot != other.pitch_dot:
            return False
        if self.yaw_dot != other.yaw_dot:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def roll(self):
        """Message field 'roll'."""
        return self._roll

    @roll.setter
    def roll(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._roll = value

    @builtins.property
    def pitch(self):
        """Message field 'pitch'."""
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw = value

    @builtins.property
    def x_dot(self):
        """Message field 'x_dot'."""
        return self._x_dot

    @x_dot.setter
    def x_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_dot = value

    @builtins.property
    def y_dot(self):
        """Message field 'y_dot'."""
        return self._y_dot

    @y_dot.setter
    def y_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_dot = value

    @builtins.property
    def roll_dot(self):
        """Message field 'roll_dot'."""
        return self._roll_dot

    @roll_dot.setter
    def roll_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'roll_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._roll_dot = value

    @builtins.property
    def pitch_dot(self):
        """Message field 'pitch_dot'."""
        return self._pitch_dot

    @pitch_dot.setter
    def pitch_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'pitch_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._pitch_dot = value

    @builtins.property
    def yaw_dot(self):
        """Message field 'yaw_dot'."""
        return self._yaw_dot

    @yaw_dot.setter
    def yaw_dot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw_dot' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'yaw_dot' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._yaw_dot = value
