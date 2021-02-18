//! Enumeration describing the different domains for timestamps acquired by a frame.
//!
//! **NOTE**: Arrival timestamps from the frame are always in system time, whereas other timestamps
//! (presentation and middle-exposure) will be in the frame's specified timestamp domain.

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;
use std::ffi::CStr;

/// Enumeration of possible timestamp domains that frame timestamps are delivered in.
#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2TimestampDomain {
    /// Timestamp is measured in relation to the device's internal clock
    HardwareClock = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK,
    /// Timestamp was measured in relation to the OS (host) system clock
    SystemTime = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME,
    /// Timestamp was measured in relation to the device's clock converted to the system clock.
    ///
    /// The timestamp is measured directly relative to the device's internal clock, and then
    /// converted to the OS (host) system clock by measuring the difference.
    GlobalTime = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME,
    // Not included since this just tells us the total number of domains
    //
    // Count = sys::rs2_timestamp_domain_RS2_TIMESTAMP_DOMAIN_COUNT,
}

impl Rs2TimestampDomain {
    pub fn as_cstr(&self) -> &'static CStr {
        unsafe {
            let ptr = sys::rs2_timestamp_domain_to_string(*self as sys::rs2_timestamp_domain);
            CStr::from_ptr(ptr)
        }
    }

    pub fn as_str(&self) -> &'static str {
        self.as_cstr().to_str().unwrap()
    }
}

impl ToString for Rs2TimestampDomain {
    fn to_string(&self) -> String {
        self.as_str().to_owned()
    }
}
