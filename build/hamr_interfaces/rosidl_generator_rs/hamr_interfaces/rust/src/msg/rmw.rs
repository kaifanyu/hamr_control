#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "hamr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__StateError() -> *const std::ffi::c_void;
}

#[link(name = "hamr_interfaces__rosidl_generator_c")]
extern "C" {
    fn hamr_interfaces__msg__StateError__init(msg: *mut StateError) -> bool;
    fn hamr_interfaces__msg__StateError__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StateError>, size: usize) -> bool;
    fn hamr_interfaces__msg__StateError__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StateError>);
    fn hamr_interfaces__msg__StateError__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StateError>, out_seq: *mut rosidl_runtime_rs::Sequence<StateError>) -> bool;
}

// Corresponds to hamr_interfaces__msg__StateError
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StateError {

    // This member is not documented.
    #[allow(missing_docs)]
    pub err_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub err_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub err_yaw: f64,

}



impl Default for StateError {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hamr_interfaces__msg__StateError__init(&mut msg as *mut _) {
        panic!("Call to hamr_interfaces__msg__StateError__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StateError {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__StateError__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__StateError__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__StateError__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StateError {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StateError where Self: Sized {
  const TYPE_NAME: &'static str = "hamr_interfaces/msg/StateError";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__StateError() }
  }
}


#[link(name = "hamr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__LiveGains() -> *const std::ffi::c_void;
}

#[link(name = "hamr_interfaces__rosidl_generator_c")]
extern "C" {
    fn hamr_interfaces__msg__LiveGains__init(msg: *mut LiveGains) -> bool;
    fn hamr_interfaces__msg__LiveGains__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LiveGains>, size: usize) -> bool;
    fn hamr_interfaces__msg__LiveGains__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LiveGains>);
    fn hamr_interfaces__msg__LiveGains__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LiveGains>, out_seq: *mut rosidl_runtime_rs::Sequence<LiveGains>) -> bool;
}

// Corresponds to hamr_interfaces__msg__LiveGains
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LiveGains {

    // This member is not documented.
    #[allow(missing_docs)]
    pub p_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub i_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub p_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub i_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub p_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub d_yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub i_yaw: f64,

}



impl Default for LiveGains {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hamr_interfaces__msg__LiveGains__init(&mut msg as *mut _) {
        panic!("Call to hamr_interfaces__msg__LiveGains__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LiveGains {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__LiveGains__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__LiveGains__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__LiveGains__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LiveGains {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LiveGains where Self: Sized {
  const TYPE_NAME: &'static str = "hamr_interfaces/msg/LiveGains";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__LiveGains() }
  }
}


#[link(name = "hamr_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__ReferenceTraj() -> *const std::ffi::c_void;
}

#[link(name = "hamr_interfaces__rosidl_generator_c")]
extern "C" {
    fn hamr_interfaces__msg__ReferenceTraj__init(msg: *mut ReferenceTraj) -> bool;
    fn hamr_interfaces__msg__ReferenceTraj__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReferenceTraj>, size: usize) -> bool;
    fn hamr_interfaces__msg__ReferenceTraj__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReferenceTraj>);
    fn hamr_interfaces__msg__ReferenceTraj__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReferenceTraj>, out_seq: *mut rosidl_runtime_rs::Sequence<ReferenceTraj>) -> bool;
}

// Corresponds to hamr_interfaces__msg__ReferenceTraj
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReferenceTraj {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x_dot: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y_dot: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub roll_dot: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_dot: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_dot: f64,

}



impl Default for ReferenceTraj {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hamr_interfaces__msg__ReferenceTraj__init(&mut msg as *mut _) {
        panic!("Call to hamr_interfaces__msg__ReferenceTraj__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReferenceTraj {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__ReferenceTraj__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__ReferenceTraj__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hamr_interfaces__msg__ReferenceTraj__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReferenceTraj {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReferenceTraj where Self: Sized {
  const TYPE_NAME: &'static str = "hamr_interfaces/msg/ReferenceTraj";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hamr_interfaces__msg__ReferenceTraj() }
  }
}


