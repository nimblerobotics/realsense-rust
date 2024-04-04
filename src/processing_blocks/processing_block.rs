//! Generic processing block structure for holding all of the possible processing types.
//!
//! Based on an example here:
//! https://github.com/IntelRealSense/librealsense/blob/4673a37d981164af8eeb8e296e430fc1427e008d/doc/post-processing-filters.md?plain=1#L111

use crate::{
    check_rs2_error,
    frame::CompositeFrame,
    kind::Rs2StreamKind,
    processing_blocks::errors::{ProcessFrameError, ProcessingBlockConstructionError},
};
use anyhow::Result;
use realsense_sys as sys;
use std::{convert::TryFrom, ptr::NonNull, task::Poll, time::Duration};

pub trait ProcessingBlockExt {
    type Configuration;
    type Options;

    fn create_block(
        config: Self::Configuration,
        err: *mut *mut sys::rs2_error,
    ) -> Result<NonNull<sys::rs2_processing_block>, ProcessingBlockConstructionError>;

    fn set_options(options: Self::Options) -> Result<(), ProcessingBlockOptionError>;
}

pub struct Align;

impl ProcessingBlockExt for Align {
    type Configuration = Rs2StreamKind;
    type Options = ();

    fn create_block(
        align_to: Self::Configuration,
        err: *mut *mut sys::rs2_error,
    ) -> Result<NonNull<sys::rs2_processing_block>, ProcessingBlockConstructionError> {
        let ptr = sys::rs2_create_align(align_to as sys::rs2_stream, &mut err);
        check_rs2_error!(
            err,
            ProcessingBlockConstructionError::CouldNotCreateProcessingBlock
        )?;

        Ok(NonNull::new(ptr).unwrap())
    }

    fn set_options(options: Self::Options) -> Result<(), ProcessingBlockOptionError> {
        Ok(())
    }
}

pub struct Decimate;

pub struct DecimateOptions {
    magnitude: f32,
}

impl ProcessingBlockExt for Decimate {
    type Configuration = ();
    type Options = DecimateOptions;
    fn create_block(
        config: Self::Configuration,
        err: *mut *mut sys::rs2_error,
    ) -> Result<NonNull<sys::rs2_processing_block>, ProcessingBlockConstructionError> {
        let ptr = sys::rs2_create_decimation_filter_block(&mut err);
        check_rs2_error!(
            err,
            ProcessingBlockConstructionError::CouldNotCreateProcessingBlock
        )?;

        Ok(NonNull::new(ptr).unwrap())
    }

    fn set_options(options: Self::Options) -> Result<(), ProcessingBlockOptionError> {
        todo!();
    }
}

/// Processing Block and Frame Queue for operating on a data stream
#[derive(Debug, Clone)]
pub struct ProcessingBlock<T, Input, Output>
where
    T: ProcessingBlockExt,
    Input: GetOwnedRaw + Sized,
    Output: TryFrom<*const sys::rs2_frame>,
{
    /// The processing block for the "Align" method
    processing_block: NonNull<sys::rs2_processing_block>,
    /// The frame queue upon which the processing block will deposit aligned frames. We check this
    /// for completed block operations.
    processing_queue: NonNull<sys::rs2_frame_queue>,

    /// Phantom data to hold the processing block kind
    _phantom: PhantomData<fn() -> (T, Input, Output)>,
}

impl<T> Drop for ProcessingBlock<T>
where
    T: ProcessingBlockExt,
{
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_frame_queue(self.processing_queue.as_ptr());
            sys::rs2_delete_processing_block(self.processing_block.as_ptr());
        }
    }
}

impl<T> ProcessingBlock<T>
where
    T: ProcessingBlockExt,
{
    /// Create a new Align object
    pub fn new(
        config: T::Configuration,
        options: Option<T::Options>,
        processing_queue_size: i32,
    ) -> Result<Self, ProcessingBlockConstructionError> {
        let (processing_block, processing_queue) = unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();

            let ptr = T::create_block(config, err)?;
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotCreateProcessingBlock
            )?;

            let queue_ptr = sys::rs2_create_frame_queue(processing_queue_size, &mut err);
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotCreateProcessingQueue
            )?;

            sys::rs2_start_processing_queue(ptr.as_ptr(), queue_ptr, &mut err);
            check_rs2_error!(
                err,
                ProcessingBlockConstructionError::CouldNotStartProcessingQueue
            )?;
            (ptr, NonNull::new(queue_ptr).unwrap())
        };

        Ok(Self {
            processing_block,
            processing_queue,
            _phantom: PhantomData,
        })
    }
}

pub trait ProcessingBlockExtExt<T, U>
where
    T: GetOwnedRaw + Sized,
    U: TryFrom<*const sys::rs2_frame, Error = Rs2Exception> + Sized,
{
    pub fn queue(&mut self, frame: T) -> Result<(), ProcessFrameError> {
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

    pub fn wait(&mut self, timeout: Duration) -> Result<U, ProcessFrameError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let timeout_millis = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);

            let frame =
                sys::rs2_wait_for_frame(self.processing_queue.as_ptr(), timeout_millis, &mut err);
            check_rs2_error!(err, |kind, context| { ProcessFrameError { kind, context } })?;
            let frame =
                U::try_from(NonNull::new(frame).unwrap()).map_err(|kind| ProcessFrameError {
                    kind,
                    // TODO: give context
                    context: String::from("TODO"),
                })?;
            Ok(frame)
        }
    }

    pub fn poll(&mut self) -> Result<U, ProcessFrameError> {
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
                let frame = U::try_from(NonNull::new(frame).unwrap()).map_err(|kind| {
                    ProcessFrameError {
                        kind,
                        // TODO: Add context
                        context: String::from("TODO"),
                    }
                })?;
                Ok(Poll::Ready(frame))
            }
        }
    }
}

///////////////////
// Implementations of the processing blocks for the different frame types

impl ProcessingBlockExtExt<CompositeFrame, CompositeFrame> for ProcessingBlock<Align> {}

// let decimate = ...;
//
// <Decimate as ProcessingBlockExtExt<ColorFrame, DepthFrame>>::queue(decimate, composite_frame)?;
//
// let depth_frame: DepthFrame = decimate.wait(timeout)?;
// <Decimate as ProcessingBlockExtExt<_, CompositeFrame>::poll(decimate, timeout)?;

pub type DecimateBlock<Frame> = ProcessingBlock<Decimate, Frame, Frame>;
pub type PointCloudBlock = ProcessingBlock<PC, Depth, PointCloud>;

// let decimate: Decimate<ColorFrame, ColorFrame>
