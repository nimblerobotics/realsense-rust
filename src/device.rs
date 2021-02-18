//! A type for abstracting over the concept of a RealSense "device"
//!
//! A device in librealsense2 refers to a complete set of sensors that comprise e.g. a D400 / L500
//! / T200 unit. A D435 or D435i, for example, is a device, whereas the individual parts that
//! comprise that device (IR cameras, depth camera, color camera, IMU) are referred to as sensors.
//! See [`sensors`](crate::sensor) for more info.

use crate::{check_rs2_error, kind::Rs2CameraInfo, sensor::Sensor};
use anyhow::Result;
use num_traits::ToPrimitive;
use realsense_sys as sys;
use std::{convert::TryFrom, ffi::CStr, ptr::NonNull};
use thiserror::Error;

/// Enumeration of possible errors that can occur during device construction
#[derive(Error, Debug)]
pub enum DeviceConstructionError {
    /// System was unable to get the device pointer that corresponds to a given [`Sensor`]
    #[error("Could not create device from sensor. Reason: {0}")]
    CouldNotCreateDeviceFromSensor(String),
    /// Could not generate the sensor list corresponding to the device during construction.
    #[error("Could not generate sensor list for device. Reason: {0}")]
    CouldNotGenerateSensorList(String),
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
    sensor_list_ptr: NonNull<sys::rs2_sensor_list>,
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_sensor_list(self.sensor_list_ptr.as_ptr());
            sys::rs2_delete_device(self.device_ptr.as_ptr());
        }
    }
}

unsafe impl Send for Device {}

impl TryFrom<NonNull<sys::rs2_device>> for Device {
    type Error = DeviceConstructionError;

    /// Attempt to construct a Device from a non-null pointer to `rs2_device`.
    ///
    /// Constructs a device from a pointer to an `rs2_device` type from the C-FFI, or returns an
    /// error if the device and its corresponding sensor list cannot be obtained.
    ///
    /// # Errors
    ///
    /// Returns [`DeviceConstructionError::CouldNotGenerateSensorList`] if the sensor list cannot
    /// be captured during construction.
    ///
    fn try_from(device_ptr: NonNull<sys::rs2_device>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let sensor_list = sys::rs2_query_sensors(device_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, DeviceConstructionError::CouldNotGenerateSensorList)?;

            Ok(Device {
                device_ptr,
                sensor_list_ptr: NonNull::new(sensor_list).unwrap(),
            })
        }
    }
}

impl Device {
    /// Gets a list of sensors associated with the device.
    ///
    /// Returns a vector of zero size if any error occurs while trying to read the sensor list.
    /// This can occur if the physical device is disconnected before this call is made.
    ///
    pub fn sensors(&self) -> Vec<Sensor> {
        let mut sensors = Vec::new();
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let len = sys::rs2_get_sensors_count(self.sensor_list_ptr.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                return sensors;
            }

            for i in 0..len {
                match Sensor::try_create(&self.sensor_list_ptr, i) {
                    Ok(s) => {
                        sensors.push(s);
                    }
                    Err(_) => {
                        continue;
                    }
                }
            }
        }
        sensors
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
}
