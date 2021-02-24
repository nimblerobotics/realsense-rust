//! A type for abstracting over the concept of a RealSense "device"
//!
//! A device in librealsense2 refers to a complete set of sensors that comprise e.g. a D400 / L500
//! / T200 unit. A D435 or D435i, for example, is a device, whereas the individual parts that
//! comprise that device (IR cameras, depth camera, color camera, IMU) are referred to as sensors.
//! See [`sensors`](crate::sensor) for more info.

use crate::{
    check_rs2_error,
    kind::{Rs2CameraInfo, Rs2Exception},
    sensor::Sensor,
};
use anyhow::Result;
use num_traits::ToPrimitive;
use realsense_sys as sys;
use std::{convert::From, ffi::CStr, ptr::NonNull};
use thiserror::Error;

/// Enumeration of possible errors that can occur during device construction
#[derive(Error, Debug)]
pub enum DeviceConstructionError {
    /// System was unable to get the device pointer that corresponds to a given [`Sensor`]
    #[error("Could not create device from sensor. Type: {0}; Reason: {1}")]
    CouldNotCreateDeviceFromSensor(Rs2Exception, String),
    /// Could not get device from device list
    #[error("Could not get device from device list. Type: {0}; Reason: {1}")]
    CouldNotGetDeviceFromDeviceList(Rs2Exception, String),
}

/// A type representing a RealSense device.
///
/// A device in librealsense2 corresponds to a physical unit that connects to your computer
/// (usually via USB). Devices hold a list of sensors, which in turn are represented by a list of
/// streams producing frames.
///
/// Devices are usually acquired by the driver context.
///
#[derive(Debug)]
pub struct Device {
    device_ptr: NonNull<sys::rs2_device>,
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_device(self.device_ptr.as_ptr());
        }
    }
}

unsafe impl Send for Device {}

impl From<NonNull<sys::rs2_device>> for Device {
    /// Attempt to construct a Device from a non-null pointer to `rs2_device`.
    ///
    /// Constructs a device from a pointer to an `rs2_device` type from the C-FFI.
    ///
    fn from(device_ptr: NonNull<sys::rs2_device>) -> Self {
        Device { device_ptr }
    }
}

impl Device {
    /// Attempt to construct a Device given a device list and index into the device list.
    ///
    /// # Errors
    ///
    /// Returns [`DeviceConstructionError::CouldNotGetDeviceFromDeviceList`] if the device cannot
    /// be retrieved from the device list (e.g. if the index is invalid).
    ///
    pub(crate) fn try_create(
        device_list: &NonNull<sys::rs2_device_list>,
        index: i32,
    ) -> Result<Self, DeviceConstructionError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let device_ptr = sys::rs2_create_device(device_list.as_ptr(), index, &mut err);
            check_rs2_error!(
                err,
                DeviceConstructionError::CouldNotGetDeviceFromDeviceList
            )?;

            let nonnull_device_ptr = NonNull::new(device_ptr).unwrap();
            Ok(Device::from(nonnull_device_ptr))
        }
    }

    /// Gets a list of sensors associated with the device.
    ///
    /// Returns a vector of zero size if any error occurs while trying to read the sensor list.
    /// This can occur if the physical device is disconnected before this call is made.
    ///
    pub fn sensors(&self) -> Vec<Sensor> {
        unsafe {
            let mut sensors = Vec::new();

            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let sensor_list_ptr = sys::rs2_query_sensors(self.device_ptr.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                return sensors;
            }

            let nonnull_sensor_list = NonNull::new(sensor_list_ptr).unwrap();

            let len = sys::rs2_get_sensors_count(nonnull_sensor_list.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                sys::rs2_delete_sensor_list(nonnull_sensor_list.as_ptr());
                return sensors;
            }

            sensors.reserve(len as usize);
            for i in 0..len {
                match Sensor::try_create(&nonnull_sensor_list, i) {
                    Ok(s) => {
                        sensors.push(s);
                    }
                    Err(_) => {
                        continue;
                    }
                }
            }
            sys::rs2_delete_sensor_list(nonnull_sensor_list.as_ptr());
            sensors
        }
    }

    /// Takes ownership of the device and forces a hardware reset on the device.
    ///
    /// Ownership of the device is taken as the underlying state can no longer be safely retained
    /// after resetting the device.
    ///
    pub fn hardware_reset(self) {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            // The only failure this can have is if device_ptr is null. This should not be the case
            // since we're storing a `NonNull` type.
            //
            // It's a bit weird, but we don't need to actually check the error. Because if the
            // device is null and this fails: you have an invalid device (so panic?) but if it
            // succeeds, the device is no longer valid and we need to drop it. This is why this
            // interface takes ownership of `self`.
            sys::rs2_hardware_reset(self.device_ptr.as_ptr(), &mut err);
        }
    }

    /// Gets the value associated with the provided camera info key from the device.
    ///
    /// Returns some information value associated with the camera info key if the `camera_info` is
    /// supported by the device, else it returns `None`.
    ///
    pub fn info(&self, camera_info: Rs2CameraInfo) -> Option<&CStr> {
        if !self.supports_info(camera_info) {
            return None;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let val = sys::rs2_get_device_info(
                self.device_ptr.as_ptr(),
                camera_info.to_u32().unwrap(),
                &mut err,
            );

            if err.as_ref().is_some() {
                None
            } else {
                Some(CStr::from_ptr(val))
            }
        }
    }

    /// Predicate for checking if `camera_info` is supported for this device.
    ///
    /// Returns true iff the device has a value associated with the `camera_info` key.
    ///
    pub fn supports_info(&self, camera_info: Rs2CameraInfo) -> bool {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let supports_info = sys::rs2_supports_device_info(
                self.device_ptr.as_ptr(),
                camera_info.to_u32().unwrap(),
                &mut err,
            );

            err.as_ref().is_none() && supports_info != 0
        }
    }

    /// Get the underlying low-level pointer to the context object
    ///
    /// # Safety
    ///
    /// This method is not intended to be called or used outside of the crate itself. Be warned, it
    /// is _undefined behaviour_ to delete or try to drop this pointer in any context. If you do,
    /// you risk a double-free or use-after-free error.
    pub(crate) unsafe fn get_raw(&self) -> NonNull<sys::rs2_device> {
        self.device_ptr
    }
}
