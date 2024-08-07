//! Processing block filling holes in the depth image
//!
//! Based on an example here:
//! https://github.com/IntelRealSense/librealsense/blob/4673a37d981164af8eeb8e296e430fc1427e008d/doc/post-processing-filters.md?plain=1#L111

use crate::{
    check_rs2_error,
    frame::{DepthFrame, FrameEx},
    kind::{OptionSetError, Rs2Option},
    processing_blocks::errors::{ProcessFrameError, ProcessingBlockConstructionError},
};
use anyhow::Result;
use realsense_sys as sys;
use std::{
    convert::{TryFrom, TryInto},
    ptr::NonNull,
    task::Poll,
    time::Duration,
};

/// Processing Block and Frame Queue for spatial filtering a stream to a certain [StreamKind]
#[derive(Debug, Clone)]
pub struct Spatial {
    /// The processing block for the "Spatial" method
    processing_block: NonNull<sys::rs2_processing_block>,
    /// The frame queue upon which the processing block will deposit filled frames. We check this
    /// for completed block operations.
    processing_queue: NonNull<sys::rs2_frame_queue>,
}

impl Drop for Spatial {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_frame_queue(self.processing_queue.as_ptr());
            sys::rs2_delete_processing_block(self.processing_block.as_ptr());
        }
    }
}

impl Spatial {
    /// Create a new Spatial object
    pub fn new(processing_queue_size: i32) -> Result<Self, ProcessingBlockConstructionError> {
        let (processing_block, processing_queue) = unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let ptr = sys::rs2_create_spatial_filter_block(&mut err);
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotCreateProcessingBlock
            )?;

            let queue_ptr = sys::rs2_create_frame_queue(processing_queue_size, &mut err);
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotCreateProcessingQueue
            )?;

            sys::rs2_start_processing_queue(ptr, queue_ptr, &mut err);
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotStartProcessingQueue
            )?;
            (NonNull::new(ptr).unwrap(), NonNull::new(queue_ptr).unwrap())
        };

        Ok(Self {
            processing_block,
            processing_queue,
        })
    }

    /// Own and process the depth frame and return the filled frames.
    pub fn queue(&mut self, frame: DepthFrame) -> Result<(), ProcessFrameError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            sys::rs2_process_frame(
                self.processing_block.as_ptr(), // -> *mut
                frame.get_owned_raw().as_ptr(),
                &mut err,
            );
            check_rs2_error!(err, |kind, context| { ProcessFrameError { kind, context } })?;
            Ok(())
        }
    }

    /// Wait to receive the results of the processing block
    pub fn wait(&mut self, timeout: Duration) -> Result<DepthFrame, ProcessFrameError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let timeout_millis = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);

            let processed_frame =
                sys::rs2_wait_for_frame(self.processing_queue.as_ptr(), timeout_millis, &mut err);
            check_rs2_error!(err, |kind, context| { ProcessFrameError { kind, context } })?;
            Ok(DepthFrame::try_from(NonNull::new(processed_frame).unwrap()).unwrap())
        }
    }

    /// Poll to receive the results of the processing block
    pub fn poll(&mut self) -> Result<Poll<Result<DepthFrame>>> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let mut frame = std::ptr::null_mut::<sys::rs2_frame>();
            let is_ready =
                sys::rs2_poll_for_frame(self.processing_queue.as_ptr(), &mut frame, &mut err);

            // Check for errors
            check_rs2_error!(err, |kind, context| { ProcessFrameError { kind, context } })?;

            // Check for queue readiness
            if is_ready == 0 {
                Ok(Poll::Pending)
            } else {
                Ok(Poll::Ready(DepthFrame::try_from(
                    NonNull::new(frame).unwrap(),
                )))
            }
        }
    }

    /// Predicate for determining if this processing block supports a given option
    ///
    /// Returns true iff the option is supported by this sensor.
    pub fn supports_option(&self, option: Rs2Option) -> bool {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        let val = unsafe {
            sys::rs2_supports_option(
                self.processing_block.as_ptr().cast::<sys::rs2_options>(),
                #[allow(clippy::useless_conversion)]
                (option as i32).try_into().unwrap(),
                &mut err,
            )
        };

        if err.is_null() {
            val != 0
        } else {
            unsafe {
                sys::rs2_free_error(err);
            }
            false
        }
    }

    /// Predicate for determining if the provided option is immutable or not.
    ///
    /// Returns true if the option is supported and can be mutated, otherwise false.
    pub fn is_option_read_only(&self, option: Rs2Option) -> bool {
        if !self.supports_option(option) {
            return false;
        }

        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        let val = unsafe {
            sys::rs2_is_option_read_only(
                self.processing_block.as_ptr().cast::<sys::rs2_options>(),
                #[allow(clippy::useless_conversion)]
                (option as i32).try_into().unwrap(),
                &mut err,
            )
        };

        if err.is_null() {
            val != 0
        } else {
            unsafe {
                sys::rs2_free_error(err);
            }
            false
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
    pub fn set_option(&mut self, option: Rs2Option, value: f32) -> Result<(), OptionSetError> {
        if !self.supports_option(option) {
            return Err(OptionSetError::OptionNotSupported);
        }

        if self.is_option_read_only(option) {
            return Err(OptionSetError::OptionIsReadOnly);
        }

        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        unsafe {
            sys::rs2_set_option(
                self.processing_block.as_ptr().cast::<sys::rs2_options>(),
                #[allow(clippy::useless_conversion)]
                (option as i32).try_into().unwrap(),
                value,
                &mut err,
            );
            check_rs2_error!(err, OptionSetError::CouldNotSetOption)?;

            Ok(())
        }
    }
}
