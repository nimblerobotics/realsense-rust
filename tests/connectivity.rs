//! Tests for evaluating connectivity / configuration of sensors

#![cfg(feature = "test-single-device")]

use realsense_rust::{
    config::Config,
    context::Context,
    kind::{Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom};

/// Ensure at least one intel device is "connected" as far as the driver is concerned.
///
/// Seems dumb but this is a necessary check for every other test.
#[test]
fn ensure_at_least_one_intel_device_connected() {
    let context = Context::new().unwrap();
    let mut mask = HashSet::new();
    mask.insert(Rs2ProductLine::AnyIntel);

    let devices = context.query_devices(mask);

    assert!(!devices.is_empty());
}

#[test]
fn can_resolve_all_streams_always() {
    let context = Context::new().unwrap();
    let mut config = Config::new();
    config.enable_all_streams().unwrap();

    let pipeline = InactivePipeline::try_from(&context).unwrap();

    assert!(pipeline.can_resolve(&config));
}

#[test]
fn cannot_resolve_bad_config() {
    let context = Context::new().unwrap();
    let mut config = Config::new();

    config
        .disable_all_streams()
        .unwrap()
        .enable_stream(
            Rs2StreamKind::Depth,
            Some(0),
            0,
            0,
            // Depth should not be able to provide motion data!
            Rs2Format::MotionXyz32F,
            100,
        )
        .unwrap();

    let pipeline = InactivePipeline::try_from(&context).unwrap();

    assert!(!pipeline.can_resolve(&config));
    assert!(pipeline.resolve(&config).is_none());
}
