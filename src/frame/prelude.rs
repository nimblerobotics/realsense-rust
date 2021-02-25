//! Traits for describing frame operations.
//!
//! Many rs2 frame types hold certain data structures in common, but have
//! different functionalities. These are encapsulated by the `*FrameEx` traits,
//! with the wildcard describing the specialization that goes with that type.

use crate::{
    common::*,
    kind::{Rs2Exception, Rs2FrameMetadata, Rs2TimestampDomain},
    sensor::Sensor,
    stream_profile::StreamProfile,
};
use anyhow::Result;
use realsense_sys as sys;
use thiserror::Error;

/// How many bits are in a byte? Who can truly say.
pub const BITS_PER_BYTE: i32 = 8;

/// Occurs when a frame type cannot be constructed from the given data.
#[derive(Error, Debug)]
pub enum FrameConstructionError {
    /// Could not get frame width.
    #[error("Could not get frame width. Type: {0}; Reason: {1}")]
    CouldNotGetWidth(Rs2Exception, String),
    /// Could not get frame height.
    #[error("Could not get frame height. Type: {0}; Reason: {1}")]
    CouldNotGetHeight(Rs2Exception, String),
    /// Could not get the pixel stride.
    #[error("Could not get stride. Type: {0}; Reason: {1}")]
    CouldNotGetStride(Rs2Exception, String),
    /// Could not get the bit count per pixel.
    #[error("Could not get bits-per-pixel. Type: {0}; Reason: {1}")]
    CouldNotGetBitsPerPixel(Rs2Exception, String),
    /// Could not get the frame timestamp.
    #[error("Could not get timestamp. Type: {0}; Reason: {1}")]
    CouldNotGetTimestamp(Rs2Exception, String),
    /// Could not get the frame timestamp's time domain, e.g. to which
    /// clock its time is relative.
    #[error("Could not get timestamp domain. Type: {0}; Reason: {1}")]
    CouldNotGetTimestampDomain(Rs2Exception, String),
    /// Could not get the stream profile that describes the frame.
    #[error("Could not get frame stream profile. Type: {0}; Reason: {1}")]
    CouldNotGetFrameStreamProfile(Rs2Exception, String),
    /// Could not get the total data size of the frame in bytes.
    #[error("Could not get data size (in bytes). Type: {0}; Reason: {1}")]
    CouldNotGetDataSize(Rs2Exception, String),
    /// Could not get the data of the frame.
    #[error("Could not get pointer to frame data. Type: {0}; Reason: {1}")]
    CouldNotGetData(Rs2Exception, String),
    /// Could not get the number of points in a Points frame.
    #[error("Could not get number of points: Type: {0}; Reason: {1}")]
    CouldNotGetPointCount(Rs2Exception, String),
}

/// Occurs when certain data cannot be derived from a Depth frame.
#[derive(Error, Debug)]
pub enum DepthError {
    /// Cannot derive distance.
    #[error("Could not get distance. Type: {0}; Reason: {1}")]
    CouldNotGetDistance(Rs2Exception, String),
    /// Cannot derive the depth units used.
    #[error("Could not get depth units. Type: {0}; Reason: {1}")]
    CouldNotGetDepthUnits(Rs2Exception, String),
}

/// Occurs when a baseline cannot be derived from a Disparity frame.
#[derive(Error, Debug)]
#[error("Could not get baseline. Type: {0}; Reason: {1}")]
pub struct DisparityError(pub Rs2Exception, pub String);

/// Cannot get the frame sensor.
#[derive(Error, Debug)]
#[error("Could not get frame sensor. Type: {0}; Reason: {1}")]
pub struct CouldNotGetFrameSensorError(pub Rs2Exception, pub String);

/// Describes common functionality across frame types.
pub trait FrameEx<'a> {
    /// Get the stream profile of the object.
    fn profile(&'a self) -> &'a StreamProfile<'a>;

    /// Get the frame sensor.
    fn sensor(&self) -> Result<Sensor>;

    /// Get the frame timestamp.
    fn timestamp(&self) -> f64;

    /// Get the RealSense timestamp domain for the current timestamp.
    fn timestamp_domain(&self) -> Rs2TimestampDomain;

    /// Get the frame metadata.
    fn metadata(&self, metadata_kind: Rs2FrameMetadata) -> Option<std::os::raw::c_longlong>;

    /// Test whether the metadata arguemnt is supported by the frame.
    fn supports_metadata(&self, metadata_kind: Rs2FrameMetadata) -> bool;

    /// Get (and own) the underlying frame pointer for this frame.
    ///
    /// This is primarily useful for passing this frame forward to a processing block or blocks
    /// (either via frame queue, directly, callback, etc).
    ///
    /// # Safety
    ///
    /// This does not destroy the underlying frame pointer once self
    /// goes out of scope. Instead, the program expects that whatever
    /// object was assigned to by this function now manages the lifetime.
    unsafe fn get_owned_raw(self) -> NonNull<sys::rs2_frame>;
}
