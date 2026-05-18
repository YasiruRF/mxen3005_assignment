#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to dobot_interface__srv__PickAndPlace_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickAndPlace_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pick_pose: [f32; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub place_pose: [f32; 4],

}



impl Default for PickAndPlace_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PickAndPlace_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PickAndPlace_Request {
  type RmwMsg = super::srv::rmw::PickAndPlace_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pick_pose: msg.pick_pose,
        place_pose: msg.place_pose,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pick_pose: msg.pick_pose,
        place_pose: msg.place_pose,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pick_pose: msg.pick_pose,
      place_pose: msg.place_pose,
    }
  }
}


// Corresponds to dobot_interface__srv__PickAndPlace_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickAndPlace_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for PickAndPlace_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PickAndPlace_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PickAndPlace_Response {
  type RmwMsg = super::srv::rmw::PickAndPlace_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}






#[link(name = "dobot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__dobot_interface__srv__PickAndPlace() -> *const std::ffi::c_void;
}

// Corresponds to dobot_interface__srv__PickAndPlace
#[allow(missing_docs, non_camel_case_types)]
pub struct PickAndPlace;

impl rosidl_runtime_rs::Service for PickAndPlace {
    type Request = PickAndPlace_Request;
    type Response = PickAndPlace_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__dobot_interface__srv__PickAndPlace() }
    }
}


