//! Traits for describing frame operations.
//!
//! Many rs2 frame types hold certain data structures in common, but have
//! different functionalities. These are encapsulated by the `*FrameEx` traits,
//! with the wildcard describing the specialization that goes with that type.

use super::pixel::PixelKind;
use crate::{
    common::*,
    kind::{Rs2FrameMetadata, Rs2TimestampDomain},
    sensor::Sensor,
    stream::StreamProfile,
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
    #[error("Could not get frame width. Reason: {0}")]
    CouldNotGetWidth(String),
    /// Could not get frame height.
    #[error("Could not get frame height. Reason: {0}")]
    CouldNotGetHeight(String),
    /// Could not get the pixel stride.
    #[error("Could not get stride. Reason: {0}")]
    CouldNotGetStride(String),
    /// Could not get the bit count per pixel.
    #[error("Could not get bits-per-pixel. Reason: {0}")]
    CouldNotGetBitsPerPixel(String),
    /// Could not get the frame timestamp.
    #[error("Could not get timestamp. Reason: {0}")]
    CouldNotGetTimestamp(String),
    /// Could not get the frame timestamp's time domain, e.g. to which
    /// clock its time is relative.
    #[error("Could not get timestamp domain. Reason: {0}")]
    CouldNotGetTimestampDomain(String),
    /// Could not get the stream profile that describes the frame.
    #[error("Could not get frame stream profile. Reason: {0}")]
    CouldNotGetFrameStreamProfile(String),
    /// Could not get the total data size of the frame in bytes.
    #[error("Could not get data size (in bytes). Reason: {0}")]
    CouldNotGetDataSize(String),
    /// Could not get the data of the frame.
    #[error("Could not get pointer to frame data. Reason: {0}")]
    CouldNotGetData(String),
    /// Could not get the number of points in a Points frame.
    #[error("Could not get number of points: {0}")]
    CouldNotGetPointCount(String),
}

/// Occurs when certain data cannot be derived from a Depth frame.
#[derive(Error, Debug)]
pub enum DepthError {
    /// Cannot derive distance.
    #[error("Could not get distance. Reason: {0}")]
    CouldNotGetDistance(String),
    /// Cannot derive the depth units used.
    #[error("Could not get depth units. Reason: {0}")]
    CouldNotGetDepthUnits(String),
}

/// Occurs when a baseline cannot be derived from a Disparity frame.
#[derive(Error, Debug)]
#[error("Could not get baseline. Reason: {0}")]
pub struct DisparityError(pub String);

/// Cannot get the frame sensor.
#[derive(Error, Debug)]
#[error("Could not get frame sensor. Reason: {0}")]
pub struct CouldNotGetFrameSensorError(pub String);

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

    /// Transfers ownership of the underlying frame data pointer
    ///
    /// # Safety
    ///
    /// This does not destroy the underlying frame pointer once self
    /// goes out of scope. Instead, the program expects that whatever
    /// object was assigned to by this function now manages the lifetime.
    unsafe fn get_owned_frame_ptr(self) -> NonNull<sys::rs2_frame>;
}

/// Describes functionality specific to Depth frames.
pub trait DepthFrameEx {
    /// Given the 2D depth coordinate (x,y) provide the corresponding depth in metric units.
    fn distance(&self, col: usize, row: usize) -> Result<f32, DepthError>;

    /// Get the metric units currently used for reporting depth information.
    fn depth_units(&self) -> Result<f32>;
}

/// Describes functionality specific to Disparity frames.
pub trait DisparityFrameEx {
    /// Get the baseline used during construction of the Disparity frame
    fn baseline(&self) -> Result<f32, DisparityError>;
}

/// Describes unsafe functionality specific to Video frames.
pub trait VideoFrameUnsafeEx<'a> {
    /// Get a pixel value from the Video Frame.
    ///
    /// # Safety
    ///
    /// This makes a call directly to the underlying data pointer inherited from
    /// the `rs2_frame`.
    fn get_unchecked(&'a self, col: usize, row: usize) -> PixelKind<'a>;

    /// Get the stride of this Video frame's pixel in bytes.
    fn stride(&self) -> usize;

    /// Get the bits per pixel.
    fn bits_per_pixel(&self) -> usize;

    /// Get the size of the data in this Video frame in bytes.
    fn get_raw_size(&self) -> usize;

    /// Get a reference to the raw data held by this Video frame.
    fn get_raw(&'a self) -> &'a std::os::raw::c_void;
}

/// Describes functionality specific to Video frames.
pub trait VideoFrameEx<'a> {
    /// Get the width of this Video frame in pixels
    fn width(&self) -> usize;

    /// Get the height of this Video frame in pixels
    fn height(&self) -> usize;

    /// Given a row and column index, Get a pixel value from this frame.
    fn get(&'a self, col: usize, row: usize) -> Option<PixelKind<'a>>;
}
