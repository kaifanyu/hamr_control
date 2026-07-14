#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to hamr_interfaces__msg__StateError

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StateError::default())
  }
}

impl rosidl_runtime_rs::Message for StateError {
  type RmwMsg = super::msg::rmw::StateError;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        err_x: msg.err_x,
        err_y: msg.err_y,
        err_yaw: msg.err_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      err_x: msg.err_x,
      err_y: msg.err_y,
      err_yaw: msg.err_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      err_x: msg.err_x,
      err_y: msg.err_y,
      err_yaw: msg.err_yaw,
    }
  }
}


// Corresponds to hamr_interfaces__msg__LiveGains

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LiveGains::default())
  }
}

impl rosidl_runtime_rs::Message for LiveGains {
  type RmwMsg = super::msg::rmw::LiveGains;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        p_x: msg.p_x,
        d_x: msg.d_x,
        i_x: msg.i_x,
        p_y: msg.p_y,
        d_y: msg.d_y,
        i_y: msg.i_y,
        p_yaw: msg.p_yaw,
        d_yaw: msg.d_yaw,
        i_yaw: msg.i_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      p_x: msg.p_x,
      d_x: msg.d_x,
      i_x: msg.i_x,
      p_y: msg.p_y,
      d_y: msg.d_y,
      i_y: msg.i_y,
      p_yaw: msg.p_yaw,
      d_yaw: msg.d_yaw,
      i_yaw: msg.i_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      p_x: msg.p_x,
      d_x: msg.d_x,
      i_x: msg.i_x,
      p_y: msg.p_y,
      d_y: msg.d_y,
      i_y: msg.i_y,
      p_yaw: msg.p_yaw,
      d_yaw: msg.d_yaw,
      i_yaw: msg.i_yaw,
    }
  }
}


// Corresponds to hamr_interfaces__msg__ReferenceTraj

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ReferenceTraj::default())
  }
}

impl rosidl_runtime_rs::Message for ReferenceTraj {
  type RmwMsg = super::msg::rmw::ReferenceTraj;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
        roll: msg.roll,
        pitch: msg.pitch,
        yaw: msg.yaw,
        x_dot: msg.x_dot,
        y_dot: msg.y_dot,
        roll_dot: msg.roll_dot,
        pitch_dot: msg.pitch_dot,
        yaw_dot: msg.yaw_dot,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      x_dot: msg.x_dot,
      y_dot: msg.y_dot,
      roll_dot: msg.roll_dot,
      pitch_dot: msg.pitch_dot,
      yaw_dot: msg.yaw_dot,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
      roll: msg.roll,
      pitch: msg.pitch,
      yaw: msg.yaw,
      x_dot: msg.x_dot,
      y_dot: msg.y_dot,
      roll_dot: msg.roll_dot,
      pitch_dot: msg.pitch_dot,
      yaw_dot: msg.yaw_dot,
    }
  }
}


