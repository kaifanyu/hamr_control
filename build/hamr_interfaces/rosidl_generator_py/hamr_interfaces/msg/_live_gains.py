# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hamr_interfaces:msg/LiveGains.idl
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


class Metaclass_LiveGains(type):
    """Metaclass of message 'LiveGains'."""

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
                'hamr_interfaces.msg.LiveGains')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__live_gains
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__live_gains
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__live_gains
            cls._TYPE_SUPPORT = module.type_support_msg__msg__live_gains
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__live_gains

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LiveGains(metaclass=Metaclass_LiveGains):
    """Message class 'LiveGains'."""

    __slots__ = [
        '_p_x',
        '_d_x',
        '_i_x',
        '_p_y',
        '_d_y',
        '_i_y',
        '_p_yaw',
        '_d_yaw',
        '_i_yaw',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'p_x': 'double',
        'd_x': 'double',
        'i_x': 'double',
        'p_y': 'double',
        'd_y': 'double',
        'i_y': 'double',
        'p_yaw': 'double',
        'd_yaw': 'double',
        'i_yaw': 'double',
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
        self.p_x = kwargs.get('p_x', float())
        self.d_x = kwargs.get('d_x', float())
        self.i_x = kwargs.get('i_x', float())
        self.p_y = kwargs.get('p_y', float())
        self.d_y = kwargs.get('d_y', float())
        self.i_y = kwargs.get('i_y', float())
        self.p_yaw = kwargs.get('p_yaw', float())
        self.d_yaw = kwargs.get('d_yaw', float())
        self.i_yaw = kwargs.get('i_yaw', float())

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
        if self.p_x != other.p_x:
            return False
        if self.d_x != other.d_x:
            return False
        if self.i_x != other.i_x:
            return False
        if self.p_y != other.p_y:
            return False
        if self.d_y != other.d_y:
            return False
        if self.i_y != other.i_y:
            return False
        if self.p_yaw != other.p_yaw:
            return False
        if self.d_yaw != other.d_yaw:
            return False
        if self.i_yaw != other.i_yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def p_x(self):
        """Message field 'p_x'."""
        return self._p_x

    @p_x.setter
    def p_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'p_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'p_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._p_x = value

    @builtins.property
    def d_x(self):
        """Message field 'd_x'."""
        return self._d_x

    @d_x.setter
    def d_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'd_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_x = value

    @builtins.property
    def i_x(self):
        """Message field 'i_x'."""
        return self._i_x

    @i_x.setter
    def i_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'i_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i_x = value

    @builtins.property
    def p_y(self):
        """Message field 'p_y'."""
        return self._p_y

    @p_y.setter
    def p_y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'p_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'p_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._p_y = value

    @builtins.property
    def d_y(self):
        """Message field 'd_y'."""
        return self._d_y

    @d_y.setter
    def d_y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'd_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_y = value

    @builtins.property
    def i_y(self):
        """Message field 'i_y'."""
        return self._i_y

    @i_y.setter
    def i_y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'i_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i_y = value

    @builtins.property
    def p_yaw(self):
        """Message field 'p_yaw'."""
        return self._p_yaw

    @p_yaw.setter
    def p_yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'p_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'p_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._p_yaw = value

    @builtins.property
    def d_yaw(self):
        """Message field 'd_yaw'."""
        return self._d_yaw

    @d_yaw.setter
    def d_yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'd_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_yaw = value

    @builtins.property
    def i_yaw(self):
        """Message field 'i_yaw'."""
        return self._i_yaw

    @i_yaw.setter
    def i_yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'i_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i_yaw = value
