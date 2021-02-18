//! Type describing the stream profile information (format, stream kind, framerate, etc.)

use crate::{
    check_rs2_error,
    kind::{Rs2Format, Rs2StreamKind},
};
use anyhow::Result;
use num_traits::FromPrimitive;
use realsense_sys as sys;
use std::{marker::PhantomData, mem::MaybeUninit, ptr::NonNull};
use thiserror::Error;

/// Type describing errors that can occur when trying to construct a stream profile.
///
/// Follows the standard pattern of errors where the enum variant describes what the low-level code
/// was attempting to do while the string carried alongside describes the underlying error message
/// from any C++ exceptions that occur.
#[derive(Error, Debug)]
pub enum StreamConstructionError {
    /// Could not get stream data during construction.
    #[error("Could not retrieve stream data. Reason: {0}")]
    CouldNotRetrieveStreamData(String),
    /// Could not determine if this stream is the default stream during construction.
    #[error("Could not determine if this is the default stream. Reason: {0}")]
    CouldNotDetermineIsDefault(String),
}

/// Type describing errors in getting or setting stream-related data.
///
/// Follows the standard pattern of errors where the enum variant describes what the low-level code
/// was attempting to do while the string carried alongside describes the underlying error message
/// from any C++ exceptions that occur.
#[derive(Error, Debug)]
pub enum DataError {
    /// Could not get extrinsics between the requested streams.
    #[error("Could not get extrinsics. Reason: {0}")]
    CouldNotGetExtrinsics(String),
    /// Could not set extrinsics between the requested streams.
    #[error("Could not set extrinsics. Reason: {0}")]
    CouldNotSetExtrinsics(String),
    /// This stream does not have video intrinsics.
    #[error("Stream does not have video intrinsics")]
    StreamDoesNotHaveVideoIntrinsics,
    /// This stream does not have motion intrinsics.
    #[error("Stream does not have motion intrinsics")]
    StreamDoesNotHaveMotionIntrinsics,
    /// Could not get video intrinsics from the requested stream.
    #[error("Could not get video intrinsics. Reason: {0}")]
    CouldNotGetIntrinsics(String),
    /// Could not get motion intrinsics from the requested stream.
    #[error("Could not get motion intrinsics. Reason: {0}")]
    CouldNotGetMotionIntrinsics(String),
}

/// Type for holding the stream profile information.
///
/// This type exists as a high-level wrapper around an underlying `rs2_stream_profile` pointer. On
/// construction, we cache a copy of the stream data and also cache whether or not this stream
/// profile is the default stream.
///
/// # Lifetimes
///
/// Stream profiles are acquired one of two ways:
///
/// 1. The stream profile list via the [`stream_profiles`](crate::sensor::Sensor::stream_profiles))
///    method on the [`Sensor`](crate::sensor::Sensor) type.
/// 2. The frame-specific stream profile via the [`profile`](crate::frame::FrameEx::profile).
///
/// Stream profiles do not outlive the parent object that you obtained them from. This is somewhat
/// artificial because this lifetime is not enforced or even documented this way in the C bindings
/// for librealsense2, however this is a useful feature so as to encourage always grabbing the
/// latest stream profile from the correct source.
///
pub struct StreamProfile<'a> {
    // Underlying non-null pointer from realsense-sys.
    //
    // Unlike many other pointer types that we get from the ffi boundary, this pointer should not
    // be manually deleted using `rs2_delete_stream_profile`. Streams are owned and managed by
    // their corresponding sensor, which are owned and managed by their corresponding devices.
    // Stream profile pointers should only be manually deleted if they are created by
    // `rs2_clone_stream_profile`, which we do not use in the high-level API.
    ptr: NonNull<sys::rs2_stream_profile>,
    // The kind of stream (e.g. depth, video, accelerometer, gyroscope, etc.)
    stream: Rs2StreamKind,
    // The bit format of the underlying data.
    //
    // For video streams this will describe how the pixels are packed and padded, for motion,
    // pose, and point frame streams this will describe how to deconstruct individual points or
    // observations.
    format: Rs2Format,
    // The stream index. Useful if you wish to enable / disable certain streams by index.
    index: usize,
    // The unique identifier for the stream.
    unique_id: i32,
    // The framerate of the stream (how fast it outputs data)
    framerate: i32,
    // Whether or not the stream is a default stream.
    is_default: bool,
    // This phantom reference to a null tuple is required to enforce the lifetime of the entire
    // struct. The purpose of this is to ensure that stream profiles do not outlive their parent
    // components.
    //
    // This is necessary because whether or not you get your profile from the sensor or an
    // individual frame, you cannot guarantee that this data will necessarily be valid beyond the
    // lifespan of that frame / sensor (because the frame / sensor own the stream profile in the
    // underlying C++ API).
    //
    // This is more or less just a means to prevent users from violating ownership / lifetime
    // semantics across the FFI boundary.
    _phantom: PhantomData<&'a ()>,
}

impl<'a> std::convert::TryFrom<NonNull<sys::rs2_stream_profile>> for StreamProfile<'a> {
    type Error = StreamConstructionError;

    /// Attempt to create a stream profile from a pointer to an `rs2_stream_profile` type.
    ///
    /// # Errors
    ///
    /// Returns [`StreamConstructionError::CouldNotRetrieveStreamData`] if the stream data
    /// associated with this stream profile cannot be retrieved.
    ///
    /// Returns [`StreamConstructionError::CouldNotDetermineIsDefault`] if it cannot be determined
    /// whether or not this stream is a default stream. This usually will only happen if the stream
    /// is invalidated (e.g. due to a device disconnect) when you try to construct it.
    ///
    fn try_from(stream_profile_ptr: NonNull<sys::rs2_stream_profile>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let mut stream = MaybeUninit::uninit();
            let mut format = MaybeUninit::uninit();
            let mut index = MaybeUninit::uninit();
            let mut unique_id = MaybeUninit::uninit();
            let mut framerate = MaybeUninit::uninit();

            sys::rs2_get_stream_profile_data(
                stream_profile_ptr.as_ptr(),
                stream.as_mut_ptr(),
                format.as_mut_ptr(),
                index.as_mut_ptr(),
                unique_id.as_mut_ptr(),
                framerate.as_mut_ptr(),
                &mut err,
            );
            check_rs2_error!(err, StreamConstructionError::CouldNotRetrieveStreamData)?;

            let is_default =
                sys::rs2_is_stream_profile_default(stream_profile_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, StreamConstructionError::CouldNotDetermineIsDefault)?;

            Ok(StreamProfile {
                ptr: stream_profile_ptr,
                stream: Rs2StreamKind::from_u32(stream.assume_init()).unwrap(),
                format: Rs2Format::from_u32(format.assume_init()).unwrap(),
                index: index.assume_init() as usize,
                unique_id: unique_id.assume_init(),
                framerate: framerate.assume_init(),
                is_default: is_default != 0,
                _phantom: PhantomData {},
            })
        }
    }
}

impl<'a> StreamProfile<'a> {
    /// Predicate for whether or not the stream is a default stream.
    pub fn is_default(&self) -> bool {
        self.is_default
    }

    /// Gets the stream kind from the stream data.
    ///
    /// This can be e.g. Depth, Video, Accel, Gyro, etc.
    pub fn stream(&self) -> Rs2StreamKind {
        self.stream
    }

    /// Gets the format for the underlying data.
    ///
    /// For video streams this will describe how the pixels are packed and padded, for motion,
    /// pose, and point frame streams this will describe how to deconstruct individual points or
    /// observations.
    pub fn format(&self) -> Rs2Format {
        self.format
    }

    /// Gets the stream's index.
    ///
    /// This is useful if you want to enable / disable a particular stream according to its index.
    pub fn stream_index(&self) -> usize {
        self.index
    }

    /// Gets the stream's unique identifier.
    pub fn unique_id(&self) -> i32 {
        self.unique_id
    }

    /// Gets the framerate / data rate of frames generated by the stream.
    pub fn framerate(&self) -> i32 {
        self.framerate
    }

    /// Get extrinsics between the origin stream (`self`) and target stream (`to_profile`).
    ///
    /// Returns the extrinsics between the origin and target streams from the underlying realsense
    /// driver iff both underlying stream pointers are valid and extrinsics exist. Otherwise
    /// returns an error.
    ///
    /// # Errors
    ///
    /// Returns [`DataError::CouldNotGetExtrinsics`] if this call fails for whatever reason.
    ///
    pub fn get_extrinsics(
        &self,
        to_profile: &StreamProfile,
    ) -> Result<sys::rs2_extrinsics, DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let mut extrinsics = MaybeUninit::<sys::rs2_extrinsics>::uninit();

            sys::rs2_get_extrinsics(
                self.ptr.as_ptr(),
                to_profile.ptr.as_ptr(),
                extrinsics.as_mut_ptr(),
                &mut err,
            );
            check_rs2_error!(err, DataError::CouldNotGetExtrinsics)?;

            Ok(extrinsics.assume_init())
        }
    }

    /// Set `extrinsics` between the origin stream (`self`) and target stream (`to_profile`).
    ///
    /// Returns null tuple `()` iff the streams are valid and the extrinsics are successfully set.
    /// Otherwise returns an error.
    ///
    /// # Errors
    ///
    /// Returns [`DataError::CouldNotSetExtrinsics`] if this call fails for whatever reason.
    ///
    pub fn set_extrinsics(
        &self,
        to_profile: &StreamProfile,
        extrinsics: sys::rs2_extrinsics,
    ) -> Result<(), DataError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            sys::rs2_register_extrinsics(
                self.ptr.as_ptr(),
                to_profile.ptr.as_ptr(),
                extrinsics,
                &mut err,
            );
            check_rs2_error!(err, DataError::CouldNotSetExtrinsics)?;

            Ok(())
        }
    }

    /// Get video intrinsics from the stream.
    ///
    /// Returns a set of video intrinsics for the stream iff the stream has video intrinsics and the stream
    /// pointer is valid. Otherwise returns an error.
    ///
    /// # Errors
    ///
    /// Returns [`DataError::StreamDoesNotHaveVideoIntrinsics`] if the stream does not have video
    /// intrinsics.
    ///
    /// Returns [`DataError::CouldNotGetIntrinsics`] if this call fails for any other reason.
    ///
    pub fn intrinsics(&self) -> Result<sys::rs2_intrinsics, DataError> {
        match self.stream {
            Rs2StreamKind::Depth => (),
            Rs2StreamKind::Color => (),
            Rs2StreamKind::Infrared => (),
            Rs2StreamKind::Fisheye => (),
            _ => {
                return Err(DataError::StreamDoesNotHaveVideoIntrinsics);
            }
        }
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let mut intrinsics = MaybeUninit::<sys::rs2_intrinsics>::uninit();

            sys::rs2_get_video_stream_intrinsics(
                self.ptr.as_ptr(),
                intrinsics.as_mut_ptr(),
                &mut err,
            );
            check_rs2_error!(err, DataError::CouldNotGetIntrinsics)?;

            Ok(intrinsics.assume_init())
        }
    }

    /// Get motion intrinsics from the stream.
    ///
    /// Returns a set of motion device intrinsics for the stream iff the stream has motion device
    /// intrinsics and the stream pointer is valid. Otherwise returns an error.
    ///
    /// # Errors
    ///
    /// Returns
    /// [`DataError::StreamDoesNotHaveMotionIntrinsics`](DataError::StreamDoesNotHaveMotionIntrinsics)
    /// if the stream does not have motion intrinsics.
    ///
    /// Returns [`DataError::CouldNotGetMotionIntrinsics`](DataError::CouldNotGetMotionIntrinsics)
    /// if this call fails for any other reason.
    ///
    pub fn motion_intrinsics(&self) -> Result<sys::rs2_motion_device_intrinsic, DataError> {
        match self.stream {
            Rs2StreamKind::Gyro => (),
            Rs2StreamKind::Accel => (),
            _ => {
                return Err(DataError::StreamDoesNotHaveMotionIntrinsics);
            }
        }
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let mut intrinsics = MaybeUninit::<sys::rs2_motion_device_intrinsic>::uninit();

            sys::rs2_get_motion_intrinsics(self.ptr.as_ptr(), intrinsics.as_mut_ptr(), &mut err);
            check_rs2_error!(err, DataError::CouldNotGetMotionIntrinsics)?;

            Ok(intrinsics.assume_init())
        }
    }
}
