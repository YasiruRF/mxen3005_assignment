#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "dobot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dobot_interface__srv__PickAndPlace_Request() -> *const std::ffi::c_void;
}

#[link(name = "dobot_interface__rosidl_generator_c")]
extern "C" {
    fn dobot_interface__srv__PickAndPlace_Request__init(msg: *mut PickAndPlace_Request) -> bool;
    fn dobot_interface__srv__PickAndPlace_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Request>, size: usize) -> bool;
    fn dobot_interface__srv__PickAndPlace_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Request>);
    fn dobot_interface__srv__PickAndPlace_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickAndPlace_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Request>) -> bool;
}

// Corresponds to dobot_interface__srv__PickAndPlace_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dobot_interface__srv__PickAndPlace_Request__init(&mut msg as *mut _) {
        panic!("Call to dobot_interface__srv__PickAndPlace_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickAndPlace_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickAndPlace_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickAndPlace_Request where Self: Sized {
  const TYPE_NAME: &'static str = "dobot_interface/srv/PickAndPlace_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dobot_interface__srv__PickAndPlace_Request() }
  }
}


#[link(name = "dobot_interface__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__dobot_interface__srv__PickAndPlace_Response() -> *const std::ffi::c_void;
}

#[link(name = "dobot_interface__rosidl_generator_c")]
extern "C" {
    fn dobot_interface__srv__PickAndPlace_Response__init(msg: *mut PickAndPlace_Response) -> bool;
    fn dobot_interface__srv__PickAndPlace_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Response>, size: usize) -> bool;
    fn dobot_interface__srv__PickAndPlace_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Response>);
    fn dobot_interface__srv__PickAndPlace_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickAndPlace_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PickAndPlace_Response>) -> bool;
}

// Corresponds to dobot_interface__srv__PickAndPlace_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickAndPlace_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for PickAndPlace_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !dobot_interface__srv__PickAndPlace_Response__init(&mut msg as *mut _) {
        panic!("Call to dobot_interface__srv__PickAndPlace_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickAndPlace_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { dobot_interface__srv__PickAndPlace_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickAndPlace_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickAndPlace_Response where Self: Sized {
  const TYPE_NAME: &'static str = "dobot_interface/srv/PickAndPlace_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__dobot_interface__srv__PickAndPlace_Response() }
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


