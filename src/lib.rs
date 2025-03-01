#![doc = include_str!("../README.md")]
#![allow(unknown_lints)]
#![allow(clippy::all)]

pub mod base;
pub mod config;
pub mod context;
pub mod device;
pub mod device_hub;
pub mod docs;
mod error;
pub mod frame;
pub mod kind;
pub mod pipeline;
pub mod processing_blocks;
pub mod sensor;
pub mod stream_profile;

pub mod logging;

/// The module collects common used traits from this crate.
pub mod prelude {
    pub use crate::frame::{FrameCategory, FrameEx};
}
