// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "hamr_interfaces/msg/detail/live_gains__struct.h"
#include "hamr_interfaces/msg/detail/live_gains__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool hamr_interfaces__msg__live_gains__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("hamr_interfaces.msg._live_gains.LiveGains", full_classname_dest, 41) == 0);
  }
  hamr_interfaces__msg__LiveGains * ros_message = _ros_message;
  {  // p_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // i_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "i_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->i_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // i_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "i_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->i_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // i_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "i_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->i_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hamr_interfaces__msg__live_gains__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LiveGains */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hamr_interfaces.msg._live_gains");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LiveGains");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hamr_interfaces__msg__LiveGains * ros_message = (hamr_interfaces__msg__LiveGains *)raw_ros_message;
  {  // p_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // i_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->i_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "i_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // i_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->i_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "i_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // i_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->i_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "i_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
