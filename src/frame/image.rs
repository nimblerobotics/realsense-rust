//! Type for representing an Image frame taken from a RealSense camera.
//!
//! An "Image" Frame can be one of several things:
//!
//! - Depth Frame: A depth frame taken from a synthetic depth camera.
//! - Disparity Frame: A disparity frame taken from a synthetic depth camera.
//! - Video Frame: A frame holding color or monochrome data.
//!
//! Each frame type can hold data in multiple formats. The data type presented
//! depends on the settings and flags used at runtime on the RealSense device.

use super::pixel::{get_pixel, PixelKind};
use super::prelude::{
    CouldNotGetFrameSensorError, DepthError, DisparityError, FrameConstructionError, FrameEx,
    BITS_PER_BYTE,
};
use crate::{
    check_rs2_error,
    common::*,
    kind::{Extension, Rs2Extension, Rs2FrameMetadata, Rs2Option, Rs2TimestampDomain},
    sensor::Sensor,
    stream_profile::StreamProfile,
};
use anyhow::Result;
use num_traits::ToPrimitive;
use std::convert::TryFrom;

/// A unit struct defining a Depth frame.
#[derive(Debug)]
pub struct Depth;
/// A unit struct defining a Disparity frame.
#[derive(Debug)]
pub struct Disparity;
/// A unit struct defining a Video frame.
#[derive(Debug)]
pub struct Video;

/// Holds the raw data pointer and derived data for an RS2 Image frame.
///
/// This generic type isn't particularly useful on it's own. In all cases, you want a specialized
/// version of this class ([`DepthFrame`], [`VideoFrame`], [`DisparityFrame`]).
#[derive(Debug)]
pub struct ImageFrame<'a, Kind> {
    /// The raw data pointer from the original rs2 frame.
    frame_ptr: NonNull<sys::rs2_frame>,
    /// The width of the frame in pixels.
    width: usize,
    /// The height of the frame in pixels.
    height: usize,
    /// The pixel stride of the frame in bytes.
    stride: usize,
    /// The number of bits per pixel.
    bits_per_pixel: usize,
    /// The timestamp of the frame.
    timestamp: f64,
    /// The RealSense time domain from which the timestamp is derived.
    timestamp_domain: Rs2TimestampDomain,
    /// The Stream Profile that created the frame.
    frame_stream_profile: StreamProfile<'a>,
    /// The size in bytes of the data contained in the frame.
    data_size_in_bytes: usize,
    /// The frame data contained in the frame.
    data: &'a std::os::raw::c_void,
    /// A boolean used during `Drop` calls. This allows for proper handling of the pointer
    /// during ownership transfer.
    should_drop: bool,
    /// Holds the type metadata of this frame.
    _phantom: PhantomData<Kind>,
}

pub struct Iter<'a, K> {
    pub(crate) frame: &'a ImageFrame<'a, K>,
    pub(crate) column: usize,
    pub(crate) row: usize,
}

impl<'a, K> Iterator for Iter<'a, K> {
    type Item = PixelKind<'a>;

    /// Provides a row-major iterator over an entire Image.
    fn next(&mut self) -> Option<Self::Item> {
        if self.column >= self.frame.width() || self.row >= self.frame.height() {
            return None;
        }

        let next = self.frame.get_unchecked(self.column, self.row);

        self.column += 1;

        if self.column >= self.frame.width() {
            self.column = 0;
            self.row += 1;
        }
        Some(next)
    }
}

/// An ImageFrame type holding the raw pointer and derived metadata for an RS2 Depth frame.
///
/// All fields in this struct are initialized during struct creation (via `try_from`).
/// Everything called from here during runtime should be valid as long as the
/// Frame is in scope... like normal Rust.
pub type DepthFrame<'a> = ImageFrame<'a, Depth>;
/// An ImageFrame type holding the raw pointer and derived metadata for an RS2 Disparity frame.
///
/// All fields in this struct are initialized during struct creation (via `try_from`).
/// Everything called from here during runtime should be valid as long as the
/// Frame is in scope... like normal Rust.
pub type DisparityFrame<'a> = ImageFrame<'a, Disparity>;
/// An ImageFrame type holding the raw pointer and derived metadata for an RS2 Video frame.
///
/// All fields in this struct are initialized during struct creation (via `try_from`).
/// Everything called from here during runtime should be valid as long as the
/// Frame is in scope... like normal Rust.
pub type VideoFrame<'a> = ImageFrame<'a, Video>;

impl<'a, K> Drop for ImageFrame<'a, K> {
    fn drop(&mut self) {
        unsafe {
            if self.should_drop {
                sys::rs2_release_frame(self.frame_ptr.as_ptr());
            }
        }
    }
}

impl<'a, K> IntoIterator for &'a ImageFrame<'a, K> {
    type Item = <Iter<'a, K> as Iterator>::Item;
    type IntoIter = Iter<'a, K>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, K> TryFrom<NonNull<sys::rs2_frame>> for ImageFrame<'a, K> {
    type Error = anyhow::Error;

    /// Attempt to construct an Image frame of extension K from the raw `rs2_frame`.
    ///
    /// All members of the `ImageFrame` struct are validated and populated during this call.
    ///
    /// # Errors
    ///
    /// There are a number of errors that may occur if the data in the `rs2_frame` is not valid,
    /// all of type [`FrameConstructionError`].
    ///
    /// - [`CouldNotGetWidth`](FrameConstructionError::CouldNotGetWidth)
    /// - [`CouldNotGetHeight`](FrameConstructionError::CouldNotGetHeight)
    /// - [`CouldNotGetBitsPerPixel`](FrameConstructionError::CouldNotGetBitsPerPixel)
    /// - [`CouldNotGetStride`](FrameConstructionError::CouldNotGetStride)
    /// - [`CouldNotGetTimestamp`](FrameConstructionError::CouldNotGetTimestamp)
    /// - [`CouldNotGetTimestampDomain`](FrameConstructionError::CouldNotGetTimestampDomain)
    /// - [`CouldNotGetFrameStreamProfile`](FrameConstructionError::CouldNotGetFrameStreamProfile)
    /// - [`CouldNotGetDataSize`](FrameConstructionError::CouldNotGetDataSize)
    /// - [`CouldNotGetData`](FrameConstructionError::CouldNotGetData)
    ///
    /// See [`FrameConstructionError`] documentation for more details.
    ///
    fn try_from(frame_ptr: NonNull<sys::rs2_frame>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();
            let width = sys::rs2_get_frame_width(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetWidth)?;

            let height = sys::rs2_get_frame_height(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetHeight)?;

            let bits_per_pixel = sys::rs2_get_frame_bits_per_pixel(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetBitsPerPixel)?;

            let stride = sys::rs2_get_frame_stride_in_bytes(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetStride)?;

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

            debug_assert_eq!(size, width * height * bits_per_pixel / BITS_PER_BYTE);

            let ptr = sys::rs2_get_frame_data(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetData)?;

            Ok(ImageFrame {
                frame_ptr,
                width: width as usize,
                height: height as usize,
                stride: stride as usize,
                bits_per_pixel: bits_per_pixel as usize,
                timestamp,
                timestamp_domain: Rs2TimestampDomain::from_u32(timestamp_domain).unwrap(),
                frame_stream_profile: profile,
                data_size_in_bytes: size as usize,
                data: ptr.as_ref().unwrap(),
                should_drop: true,
                _phantom: PhantomData::<K> {},
            })
        }
    }
}

impl<'a> Extension for DepthFrame<'a> {
    fn extension() -> Rs2Extension {
        Rs2Extension::DepthFrame
    }
}

impl<'a> Extension for DisparityFrame<'a> {
    fn extension() -> Rs2Extension {
        Rs2Extension::DisparityFrame
    }
}

impl<'a> Extension for VideoFrame<'a> {
    fn extension() -> Rs2Extension {
        Rs2Extension::VideoFrame
    }
}

impl<'a, T> FrameEx<'a> for ImageFrame<'a, T> {
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

impl<'a> DepthFrame<'a> {
    /// Given the 2D depth coordinate (x,y) provide the corresponding depth in metric units.
    ///
    /// # Warning
    ///
    /// It is fairly expensive to use this in practice as it will copy the underlying pixel into a
    /// f32 value that gives you the direct distance. In practice getting
    /// `DepthFrame::depth_units` and then applying that to the raw data with [`ImageFrame::get`]
    /// is a much more efficient way to handle this.
    pub fn distance(&self, col: usize, row: usize) -> Result<f32, DepthError> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();
            let distance = sys::rs2_depth_frame_get_distance(
                self.frame_ptr.as_ptr(),
                col as c_int,
                row as c_int,
                &mut err,
            );
            check_rs2_error!(err, DepthError::CouldNotGetDistance)?;
            Ok(distance)
        }
    }

    /// Get the metric units currently used for reporting depth information.
    pub fn depth_units(&self) -> Result<f32> {
        let sensor = self.sensor()?;
        let depth_units = sensor.get_option(Rs2Option::DepthUnits).ok_or_else(|| {
            anyhow::anyhow!("Option is not supported on the sensor for this frame type.")
        })?;
        Ok(depth_units)
    }
}

impl<'a> DisparityFrame<'a> {
    /// Given the 2D depth coordinate (x,y) provide the corresponding depth in metric units.
    ///
    /// # Warning
    ///
    /// Like with depth frames, this method is fairly expensive to use in practice. The disparity
    /// can be converted to depth fairly easily, but this will effectively copy every pixel if you
    /// loop through the data with this method for every index.
    ///
    /// It is often much more efficient to directly stream the
    /// [`Rs2Format::Distance`](crate::kind::Rs2Format::Distance) format if you want the distance
    /// directly, and access the frame data with [`ImageFrame::get`].
    pub fn distance(&self, col: usize, row: usize) -> Result<f32, DepthError> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();
            let distance = sys::rs2_depth_frame_get_distance(
                self.frame_ptr.as_ptr(),
                col as c_int,
                row as c_int,
                &mut err,
            );
            check_rs2_error!(err, DepthError::CouldNotGetDistance)?;
            Ok(distance)
        }
    }

    /// Get the metric units currently used for reporting depth information.
    pub fn depth_units(&self) -> Result<f32> {
        let sensor = self.sensor()?;
        let depth_units = sensor.get_option(Rs2Option::DepthUnits).ok_or_else(|| {
            anyhow::anyhow!("Option is not supported on the sensor for this frame type.")
        })?;
        Ok(depth_units)
    }

    /// Get the baseline used during construction of the Disparity frame
    pub fn baseline(&self) -> Result<f32, DisparityError> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();
            let baseline =
                sys::rs2_depth_stereo_frame_get_baseline(self.frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, DisparityError)?;
            Ok(baseline)
        }
    }
}

impl<'a, K> ImageFrame<'a, K> {
    /// Iterator through every [pixel](crate::frame::PixelKind) of an image frame.
    pub fn iter(&'a self) -> Iter<'a, K> {
        Iter {
            frame: self,
            column: 0,
            row: 0,
        }
    }

    /// Get a pixel value from the Video Frame.
    ///
    /// # Safety
    ///
    /// This makes a call directly to the underlying data pointer inherited from
    /// the `rs2_frame`.
    pub fn get_unchecked(&'a self, col: usize, row: usize) -> PixelKind<'a> {
        unsafe {
            get_pixel(
                self.frame_stream_profile.format(),
                self.data_size_in_bytes,
                self.data,
                self.stride,
                col,
                row,
            )
        }
    }

    /// Get the stride of this Video frame's pixel in bytes.
    pub fn stride(&self) -> usize {
        self.stride
    }

    /// Get the bits per pixel.
    pub fn bits_per_pixel(&self) -> usize {
        self.bits_per_pixel
    }

    /// Get the size of the data in this Video frame in bytes.
    pub fn get_data_size(&self) -> usize {
        self.data_size_in_bytes
    }

    /// Get a reference to the raw data held by this Video frame.
    pub fn get_data(&'a self) -> &'a std::os::raw::c_void {
        self.data
    }

    /// Get the width of this Video frame in pixels
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the height of this Video frame in pixels
    pub fn height(&self) -> usize {
        self.height
    }

    /// Given a row and column index, Get a pixel value from this frame.
    pub fn get(&'a self, col: usize, row: usize) -> Option<PixelKind<'a>> {
        if col >= self.width || row >= self.height {
            None
        } else {
            Some(self.get_unchecked(col, row))
        }
    }
}
