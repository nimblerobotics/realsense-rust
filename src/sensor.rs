//! Type for abstracting over the concept of a Sensor
//!
//! The words device and sensor are overloaded in this industry, and librealsense2 makes no
//! attempts to fix that. While the words device and sensor are often used colloquially, here
//! `Device` refers to a whole Realsense package, such as a D435i, L515, etc. `Sensor` refers to
//! "sub-devices"; rather, the individual sensing components that one can stream from. A sensor may
//! be an IMU, which has accelerometer and gyroscope streams, or a camera of some kind, which has
//! streams related to different types of images.
//!
//! The hierarchy is effectively:
//!
//! [`Device`] |-> [`Sensor`] |-> [`StreamProfile`]
//!

use crate::{
    check_rs2_error,
    device::{Device, DeviceConstructionError},
    kind::{
        OptionSetError, Rs2CameraInfo, Rs2Extension, Rs2Option, Rs2OptionRange, SENSOR_EXTENSIONS,
    },
    stream::StreamProfile,
};
use anyhow::Result;
use num_traits::ToPrimitive;
use realsense_sys as sys;
use std::{convert::TryFrom, ffi::CStr, mem::MaybeUninit, ptr::NonNull};
use thiserror::Error;

/// Type describing errors that can occur when trying to construct a sensor.
///
/// Follows the standard pattern of errors where the enum variant describes what the low-level code
/// was attempting to do while the string carried alongside describes the underlying error message
/// from any C++ exceptions that occur.
#[derive(Error, Debug)]
pub enum SensorConstructionError {
    /// Could not generate the stream profile list for the sensor at construction time.
    #[error("Could not generate stream profile list for sensor. Reason: {0}")]
    CouldNotGenerateStreamProfileList(String),
    /// Could not get the correct sensor from the sensor list.
    #[error("Could not get correct sensor from sensor list. Reason: {0}")]
    CouldNotGetSensorFromList(String),
}

/// Type for holding sensor-related data.
///
/// A sensor in librealsense2 corresponds to a physical component on the unit in some way, shape,
/// or form. These may or may not correspond to multiple streams. e.g. an IMU on the device may
/// correspond to accelerometer and gyroscope streams, or an IR camera sensor on the device may
/// correspond to depth & video streams.
///
/// Sensors are constructed one of two ways:
///
/// 1. From the device's [sensor list](crate::device::Device::sensors)
/// 2. By getting the sensor that [corresponds to a given frame](crate::frame::FrameEx::sensor)
///
pub struct Sensor {
    // The underlying non-null sensor pointer.
    //
    // This should not be deleted unless the sensor was constructed via `rs2_create_sensor`
    sensor_ptr: NonNull<sys::rs2_sensor>,
    // Pointer to this sensor's underlying stream profiles list.
    //
    // This is always deleted when the sensor is dropped.
    stream_profiles_ptr: NonNull<sys::rs2_stream_profile_list>,
    // Boolean used for telling us if we should drop the sensor pointer or not.
    should_drop: bool,
}

impl Drop for Sensor {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_stream_profiles_list(self.stream_profiles_ptr.as_ptr());

            if self.should_drop {
                sys::rs2_delete_sensor(self.sensor_ptr.as_ptr());
            }
        }
    }
}

unsafe impl Send for Sensor {}

impl std::convert::TryFrom<NonNull<sys::rs2_sensor>> for Sensor {
    type Error = SensorConstructionError;

    /// Attempt to construct a Sensor from a non-null pointer to `rs2_sensor`.
    ///
    /// # Errors
    ///
    /// This method will only fail if the stream profile list cannot be obtained from the
    /// underlying `rs2_sensor`. The only error then is if this returns
    /// [`SensorConstructionError::CouldNotGenerateStreamProfileList`].
    ///
    fn try_from(sensor_ptr: NonNull<sys::rs2_sensor>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let stream_profiles_ptr = sys::rs2_get_stream_profiles(sensor_ptr.as_ptr(), &mut err);
            check_rs2_error!(
                err,
                SensorConstructionError::CouldNotGenerateStreamProfileList
            )?;

            Ok(Sensor {
                sensor_ptr,
                stream_profiles_ptr: NonNull::new(stream_profiles_ptr).unwrap(),
                should_drop: false,
            })
        }
    }
}

impl Sensor {
    /// Create a sensor from a sensor list and an index
    ///
    /// Unlike when you directly acquire a `*mut rs2_sensor` from an API in librealsense2, such as
    /// when calling `rs2_get_frame_sensor`, you have to drop this pointer at the end (because you
    /// now own it). When calling `try_from` we don't want to drop in the default case, since our
    /// `*mut rs2_sensor` may not be owned by us, but by the device / frame / etc.
    ///
    /// The main difference then is that this API defaults to using `rs2_create_sensor` vs. a call
    /// to get a sensor from somewhere else.
    ///
    /// This can fail for similar reasons to `try_from`, and is likewise only valid if `index` is
    /// less than the length of `sensor_list` (see `rs2_get_sensors_count` for how to get that
    /// length).
    ///
    /// Guaranteeing the lifetime / semantics of the sensor is difficult, so this should probably
    /// not be used outside of this crate. See `crate::device::Device` for where this is used.
    ///
    /// # Errors
    ///
    /// Returns [`SensorConstructionError::CouldNotGetSensorFromList`] if the index is invalid or
    /// if the sensor list is invalid in some way.
    ///
    /// Returns [`SensorConstructionError::CouldNotGenerateStreamProfileList`] if the stream
    /// profile list cannot be obtained during construction.
    ///
    pub(crate) fn try_create(
        sensor_list: &NonNull<sys::rs2_sensor_list>,
        index: i32,
    ) -> Result<Self, SensorConstructionError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let sensor_ptr = sys::rs2_create_sensor(sensor_list.as_ptr(), index, &mut err);
            check_rs2_error!(err, SensorConstructionError::CouldNotGetSensorFromList)?;

            let nonnull_ptr = NonNull::new(sensor_ptr).unwrap();
            let mut sensor = Sensor::try_from(nonnull_ptr)?;
            sensor.should_drop = true;
            Ok(sensor)
        }
    }

    /// Get the parent device that this sensor corresponds to.
    ///
    /// Returns the device that this sensor corresponds to iff that device is still connected and
    /// the sensor is still valid. Otherwise returns an error.
    ///
    /// # Errors
    ///
    /// Returns
    /// [`DeviceConstructionError::CouldNotCreateDeviceFromSensor`](crate::device::DeviceConstructionError::CouldNotCreateDeviceFromSensor)
    /// if the device cannot be obtained due to the physical device being disconnected or the
    /// internal sensor pointer becoming invalid.
    ///
    /// Returns
    /// [`DeviceConstructionError::CouldNotGenerateSensorList`](crate::device::DeviceConstructionError::CouldNotGenerateSensorList)
    /// if the sensor list cannot be captured during construction.
    ///
    pub fn device(&self) -> Result<Device> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let device_ptr = sys::rs2_create_device_from_sensor(self.sensor_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, DeviceConstructionError::CouldNotCreateDeviceFromSensor)?;

            Ok(Device::try_from(NonNull::new(device_ptr).unwrap())?)
        }
    }

    /// List of all valid sensor extensions that apply to this sensor.
    ///
    /// See the [list of possible sensor extensions](crate::kind::Rs2Extension) for
    /// more info.
    pub fn extensions(&self) -> Vec<Rs2Extension> {
        SENSOR_EXTENSIONS
            .iter()
            .filter_map(|ext| unsafe {
                let mut err = std::ptr::null_mut::<sys::rs2_error>();
                let is_extendable = sys::rs2_is_sensor_extendable_to(
                    self.sensor_ptr.as_ptr(),
                    ext.to_u32().unwrap(),
                    &mut err,
                );

                if err.as_ref().is_some() {
                    None
                } else if is_extendable != 0 {
                    Some(*ext)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get the value associated with the provided Rs2Option for the sensor.
    ///
    /// Returns An `f32` value corresponding to that option within the librealsense2 library, or None
    /// if the option is not supported.
    ///
    pub fn get_option(&self, option: Rs2Option) -> Option<f32> {
        if !self.supports_option(option) {
            return None;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let val = sys::rs2_get_option(
                self.sensor_ptr.as_ptr().cast::<sys::rs2_options>(),
                option.to_u32().unwrap(),
                &mut err,
            );

            if err.as_ref().is_none() {
                Some(val)
            } else {
                None
            }
        }
    }

    /// Sets the `value` associated with the provided `option` for the sensor.
    ///
    /// Returns null tuple if the option can be successfully set on the sensor, otherwise an error.
    ///
    /// # Errors
    ///
    /// Returns [`OptionSetError::OptionNotSupported`] if the option is not supported on this
    /// sensor.
    ///
    /// Returns [`OptionSetError::OptionIsReadOnly`] if the option is supported but cannot be set
    /// on this sensor.
    ///
    /// Returns [`OptionSetError::CouldNotSetOption`] if the option is supported and not read-only,
    /// but could not be set for another reason (invalid value, internal exception, etc.).
    ///
    pub fn set_option(&mut self, option: Rs2Option, value: f32) -> Result<(), OptionSetError> {
        if !self.supports_option(option) {
            return Err(OptionSetError::OptionNotSupported);
        }

        if self.is_option_read_only(option) {
            return Err(OptionSetError::OptionIsReadOnly);
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            sys::rs2_set_option(
                self.sensor_ptr.as_ptr().cast::<sys::rs2_options>(),
                option.to_u32().unwrap(),
                value,
                &mut err,
            );
            check_rs2_error!(err, OptionSetError::CouldNotSetOption)?;

            Ok(())
        }
    }

    /// Gets the range for a given option.
    ///
    /// Returns some option range if the sensor supports the option, else `None`.
    ///
    pub fn get_option_range(&self, option: Rs2Option) -> Option<Rs2OptionRange> {
        if !self.supports_option(option) {
            return None;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let mut min = MaybeUninit::uninit();
            let mut max = MaybeUninit::uninit();
            let mut step = MaybeUninit::uninit();
            let mut default = MaybeUninit::uninit();

            sys::rs2_get_option_range(
                self.sensor_ptr.as_ptr().cast::<sys::rs2_options>(),
                option.to_u32().unwrap(),
                min.as_mut_ptr(),
                max.as_mut_ptr(),
                step.as_mut_ptr(),
                default.as_mut_ptr(),
                &mut err,
            );

            if err.as_ref().is_none() {
                Some(Rs2OptionRange {
                    min: min.assume_init(),
                    max: max.assume_init(),
                    step: step.assume_init(),
                    default: default.assume_init(),
                })
            } else {
                None
            }
        }
    }

    /// Predicate for determining if this sensor supports a given option
    ///
    /// Returns true iff the option is supported by this sensor.
    ///
    pub fn supports_option(&self, option: Rs2Option) -> bool {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let val = sys::rs2_supports_option(
                self.sensor_ptr.as_ptr().cast::<sys::rs2_options>(),
                option.to_u32().unwrap(),
                &mut err,
            );

            if err.as_ref().is_none() {
                val != 0
            } else {
                false
            }
        }
    }

    /// Predicate for determining if the provided option is immutable or not.
    ///
    /// Returns true if the option is supported and can be mutated, otherwise false.
    ///
    pub fn is_option_read_only(&self, option: Rs2Option) -> bool {
        if !self.supports_option(option) {
            return false;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let val = sys::rs2_is_option_read_only(
                self.sensor_ptr.as_ptr().cast::<sys::rs2_options>(),
                option.to_u32().unwrap(),
                &mut err,
            );

            if err.as_ref().is_none() {
                val != 0
            } else {
                false
            }
        }
    }

    /// Get a list of stream profiles associated with this sensor
    ///
    /// Returns a vector containing all the stream profiles associated with the sensor. The vector
    /// will have a length of zero if an error occurs while getting the stream profiles.
    ///
    pub fn stream_profiles(&self) -> Vec<StreamProfile> {
        let mut profiles = Vec::new();
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let len =
                sys::rs2_get_stream_profiles_count(self.stream_profiles_ptr.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                return profiles;
            }

            for i in 0..len {
                let profile_ptr =
                    sys::rs2_get_stream_profile(self.stream_profiles_ptr.as_ptr(), i, &mut err);

                if err.as_ref().is_some() {
                    err = std::ptr::null_mut();
                    continue;
                }

                let nonnull_ptr =
                    NonNull::new(profile_ptr as *mut sys::rs2_stream_profile).unwrap();

                match StreamProfile::try_from(nonnull_ptr) {
                    Ok(s) => {
                        profiles.push(s);
                    }
                    Err(_) => {
                        continue;
                    }
                }
            }
        }
        profiles
    }

    // fn recommended_processing_blocks(&self) -> Vec<ProcessingBlock>{}

    /// Gets the value associated with the provided camera info key from the sensor.
    ///
    /// Returns some value corresponding to the camera info requested if this sensor supports that
    /// camera info, else `None`.
    ///
    pub fn info(&self, camera_info: Rs2CameraInfo) -> Option<&CStr> {
        if !self.supports_info(camera_info) {
            return None;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let val = sys::rs2_get_sensor_info(
                self.sensor_ptr.as_ptr(),
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

    /// Predicate method for determining if the sensor supports a certain kind of camera info.
    ///
    /// Returns true iff the sensor has a value associated with the `camera_info` key.
    ///
    pub fn supports_info(&self, camera_info: Rs2CameraInfo) -> bool {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let supports_info = sys::rs2_supports_sensor_info(
                self.sensor_ptr.as_ptr(),
                camera_info.to_u32().unwrap(),
                &mut err,
            );

            err.as_ref().is_none() && supports_info != 0
        }
    }
}
