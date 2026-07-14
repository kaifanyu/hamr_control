// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from hamr_interfaces:msg/StateError.idl
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
#include "hamr_interfaces/msg/detail/state_error__struct.h"
#include "hamr_interfaces/msg/detail/state_error__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool hamr_interfaces__msg__state_error__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("hamr_interfaces.msg._state_error.StateError", full_classname_dest, 43) == 0);
  }
  hamr_interfaces__msg__StateError * ros_message = _ros_message;
  {  // err_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // err_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "err_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->err_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * hamr_interfaces__msg__state_error__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StateError */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("hamr_interfaces.msg._state_error");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StateError");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  hamr_interfaces__msg__StateError * ros_message = (hamr_interfaces__msg__StateError *)raw_ros_message;
  {  // err_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // err_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->err_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "err_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
