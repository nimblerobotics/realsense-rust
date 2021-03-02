//! Type for representing a RealSense Motion frame.
//!
//! Motion data for any Motion frame is held as a 3-vector. This data (retrieved
//! through `motion()`) represents different things depending on the device recorded.
//!
//! See the docs for [MotionFrame::motion] for more.

use super::prelude::{CouldNotGetFrameSensorError, FrameConstructionError, FrameEx};
use crate::{
    check_rs2_error,
    common::*,
    kind::{Extension, Rs2Extension, Rs2FrameMetadata, Rs2TimestampDomain},
    sensor::Sensor,
    stream_profile::StreamProfile,
};
use anyhow::Result;
use num_traits::ToPrimitive;
use std::convert::TryFrom;

/// Holds the raw data pointer and derived data from an RS2 Motion Frame.
///
/// All fields in this struct are initialized during struct creation (via `try_from`).
/// Everything called from here during runtime should be valid as long as the
/// Frame is in scope... like normal Rust.
#[derive(Debug)]
pub struct MotionFrame<'a> {
    /// The raw data pointer from the original rs2 frame.
    frame_ptr: NonNull<sys::rs2_frame>,
    /// The timestamp of the frame.
    timestamp: f64,
    /// The RealSense time domain from which the timestamp is derived.
    timestamp_domain: Rs2TimestampDomain,
    /// The Stream Profile that created the frame.
    frame_stream_profile: StreamProfile<'a>,
    /// The motion data held in this Motion Frame. Motion data is represented as a
    /// 3-vector, with different conventions depending on the device recorded.
    /// See the [motion](MotionFrame::motion) function for more documentation.
    motion: [f32; 3],
    /// A boolean used during `Drop` calls. This allows for proper handling of the pointer
    /// during ownership transfer.
    should_drop: bool,
}

impl<'a> Extension for MotionFrame<'a> {
    fn extension() -> Rs2Extension {
        Rs2Extension::MotionFrame
    }
}

impl<'a> Drop for MotionFrame<'a> {
    /// Drop the raw pointer stored with this struct whenever it goes out of scope.
    fn drop(&mut self) {
        unsafe {
            if self.should_drop {
                sys::rs2_release_frame(self.frame_ptr.as_ptr());
            }
        }
    }
}

unsafe impl<'a> Send for MotionFrame<'a> {}

impl<'a> TryFrom<NonNull<sys::rs2_frame>> for MotionFrame<'a> {
    type Error = anyhow::Error;

    /// Attempt to create an Image frame of extension K from the raw `rs2_frame`. All
    /// members of the ImageFrame struct are validated and populated during this call.
    ///
    /// # Errors
    ///
    /// There are a number of errors that may occur if the data in the `rs2_frame` is not
    /// valid, all of type [FrameConstructionError].
    ///
    /// - [CouldNotGetTimestamp](FrameConstructionError::CouldNotGetTimestamp)
    /// - [CouldNotGetTimestampDomain](FrameConstructionError::CouldNotGetTimestampDomain)
    /// - [CouldNotGetFrameStreamProfile](FrameConstructionError::CouldNotGetFrameStreamProfile)
    /// - [CouldNotGetDataSize](FrameConstructionError::CouldNotGetDataSize)
    /// - [CouldNotGetData](FrameConstructionError::CouldNotGetData)
    ///
    /// See [FrameConstructionError] documentation for more details.
    ///
    fn try_from(frame_ptr: NonNull<sys::rs2_frame>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();

            let timestamp = sys::rs2_get_frame_timestamp(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetTimestamp)?;

            let timestamp_domain =
                sys::rs2_get_frame_timestamp_domain(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetTimestampDomain)?;

            let profile_ptr = sys::rs2_get_frame_stream_profile(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetFrameStreamProfile)?;

            let nonnull_profile_ptr =
                NonNull::new(profile_ptr as *mut sys::rs2_stream_profile).unwrap();
            let profile = StreamProfile::try_from(nonnull_profile_ptr)?;

            let size = sys::rs2_get_frame_data_size(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetDataSize)?;

            let ptr = sys::rs2_get_frame_data(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetData)?;

            let data_as_ptr = ptr.as_ref().unwrap() as *const std::os::raw::c_void;
            let motion_raw = std::slice::from_raw_parts(
                data_as_ptr.cast::<f32>(),
                size as usize / std::mem::size_of::<f32>(),
            );

            Ok(MotionFrame {
                frame_ptr,
                timestamp,
                timestamp_domain: Rs2TimestampDomain::from_u32(timestamp_domain).unwrap(),
                frame_stream_profile: profile,
                motion: [motion_raw[0], motion_raw[1], motion_raw[2]],
                should_drop: true,
            })
        }
    }
}

impl<'a> FrameEx<'a> for MotionFrame<'a> {
    fn profile(&'a self) -> &'a StreamProfile<'a> {
        &self.frame_stream_profile
    }

    fn sensor(&self) -> Result<Sensor> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let sensor_ptr = sys::rs2_get_frame_sensor(self.frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, CouldNotGetFrameSensorError)?;

            Ok(Sensor::try_from(NonNull::new(sensor_ptr).unwrap())?)
        }
    }
    fn timestamp(&self) -> f64 {
        self.timestamp
    }

    fn timestamp_domain(&self) -> Rs2TimestampDomain {
        self.timestamp_domain
    }

    fn metadata(&self, metadata_kind: Rs2FrameMetadata) -> Option<std::os::raw::c_longlong> {
        if !self.supports_metadata(metadata_kind) {
            return None;
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let val = sys::rs2_get_frame_metadata(
                self.frame_ptr.as_ptr(),
                metadata_kind.to_u32().unwrap(),
                &mut err,
            );
            err.as_ref()?;

            Some(val)
        }
    }

    fn supports_metadata(&self, metadata_kind: Rs2FrameMetadata) -> bool {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let supports_metadata = sys::rs2_supports_frame_metadata(
                self.frame_ptr.as_ptr(),
                metadata_kind.to_u32().unwrap(),
                &mut err,
            );

            err.as_ref().is_none() && supports_metadata != 0
        }
    }

    unsafe fn get_owned_raw(mut self) -> NonNull<sys::rs2_frame> {
        self.should_drop = false;

        self.frame_ptr
    }
}

impl<'a> MotionFrame<'a> {
    /// Returns a 3-item array representing the sensor motion recorded in the Motion frame.
    ///
    /// This function will return different data conventions entirely depending on the device
    /// used to create the measurement.
    ///
    /// Gyroscope measurements are reported in radians.
    ///
    /// Accelerometer readings are reported in m/s^2.
    ///
    /// # Intel RealSense D435i
    ///
    /// - `motion[0]`: Positive x-axis points to the right.
    /// - `motion[1]`: Positive y-axis points down.
    /// - `motion[2]`: Positive z-axis points forward.
    ///
    /// # Intel RealSense T265
    ///
    /// - `motion[0]`: Positive X direction is towards right imager.
    /// - `motion[1]`: Positive Y direction is upwards toward the top of the device.
    /// - `motion[2]`: Positive Z direction is inwards toward the back of the device.
    ///
    /// Read more about the coordinate frames of RealSense motion in
    /// [the RealSense docs](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/)
    ///
    pub fn motion(&self) -> &[f32; 3] {
        &self.motion
    }
}
