//! Composite frame type containing all other potential frame types.
//!
//! Each Pipeline produces a synchronized collection of frames for all streams
//! configured for its allocated device. These frames will be presented to the user as
//! a collection via the Composite Frame type.
//!
//! This is typically what is delivered from the pipeline.

use crate::{common::*, kind::Extension};
use num_traits::ToPrimitive;
use std::convert::{From, TryFrom};

/// Holds the raw data pointer from an RS2 Composite frame type.
pub struct CompositeFrame {
    /// The raw data pointer from the original rs2 frame
    pub ptr: NonNull<sys::rs2_frame>,
}

impl Drop for CompositeFrame {
    /// Drop the raw pointer stored with this struct whenever it goes out of scope.
    fn drop(&mut self) {
        unsafe {
            sys::rs2_release_frame(self.ptr.as_ptr());
        }
    }
}

impl From<NonNull<sys::rs2_frame>> for CompositeFrame {
    fn from(frame_ptr: NonNull<sys::rs2_frame>) -> Self {
        Self { ptr: frame_ptr }
    }
}

impl CompositeFrame {
    /// Gets the number of individual frames included in the composite frame.
    pub fn count(&self) -> usize {
        unsafe {
            let mut err: *mut sys::rs2_error = ptr::null_mut();
            let count = sys::rs2_embedded_frames_count(self.ptr.as_ptr(), &mut err);
            if NonNull::new(err).is_some() {
                0
            } else {
                count as usize
            }
        }
    }

    /// Checks if the Composite frame collection is empty.
    pub fn is_empty(&self) -> bool {
        self.count() == 0
    }

    /// Retrieves all frames in the Composite frame collection of a given type.
    ///
    /// # Generic Arguments
    ///
    /// `E` must implement [`Extension`](crate::kind::Extension). Some examples of good types to
    /// use for this are:
    ///
    /// * [`VideoFrame`](crate::frame::VideoFrame)
    /// * [`DepthFrame`](crate::frame::DepthFrame)
    /// * [`DisparityFrame`](crate::frame::DisparityFrame)
    /// * [`PoseFrame`](crate::frame::PoseFrame)
    /// * [`PointsFrame`](crate::frame::PointsFrame)
    ///
    pub fn frames_of_extension<E>(&self) -> Vec<E>
    where
        E: TryFrom<NonNull<sys::rs2_frame>> + Extension,
    {
        let mut frames = Vec::new();
        for i in 0..self.count() {
            let ptr = unsafe {
                let mut err: *mut sys::rs2_error = ptr::null_mut();
                let ptr = sys::rs2_extract_frame(self.ptr.as_ptr(), i as c_int, &mut err);

                if NonNull::new(err).is_some() {
                    None
                } else {
                    NonNull::new(ptr)
                }
            };

            if let Some(ptr) = ptr {
                unsafe {
                    let mut err: *mut sys::rs2_error = ptr::null_mut();
                    let is_kind = sys::rs2_is_frame_extendable_to(
                        ptr.as_ptr(),
                        E::extension().to_u32().unwrap(),
                        &mut err,
                    );
                    if NonNull::new(err).is_none() && is_kind != 0 {
                        if let Ok(f) = E::try_from(ptr) {
                            frames.push(f);
                        }
                    }
                }
            }
        }
        frames
    }
}
