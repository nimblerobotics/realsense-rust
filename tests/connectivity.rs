//! Tests for evaluating connectivity / configuration of sensors

#![cfg(feature = "test-single-device")]

use realsense_rust::{
    config::Config,
    context::Context,
    frame::{ColorFrame, DepthFrame, InfraredFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

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
fn can_resolve_color_and_depth_and_infrared_on_d400_series() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::D400);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Color, Some(0), 0, 0, Rs2Format::Rgba8, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, Some(0), 0, 0, Rs2Format::Z16, 30)
            .unwrap()
            // RealSense doesn't seem to like index zero for the IR cameras
            //
            // Really not sure why? This seems like an implementation issue, but in practice most
            // won't be after the IR image directly (I think?).
            .enable_stream(Rs2StreamKind::Infrared, Some(1), 0, 0, Rs2Format::Y8, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Infrared, Some(2), 0, 0, Rs2Format::Any, 30)
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();

        assert!(pipeline.can_resolve(&config));
        assert!(pipeline.resolve(&config).is_some());
    }
}

#[test]
fn can_resolve_color_and_depth_and_infrared_on_l500_series() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::L500);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Rgba8, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, Some(0), 0, 0, Rs2Format::Z16, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Infrared, Some(0), 0, 0, Rs2Format::Y8, 30)
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();

        assert!(pipeline.can_resolve(&config));
        assert!(pipeline.resolve(&config).is_some());
    }
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

#[test]
fn d400_streams_at_expected_framerate() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::D400);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        let framerate = 30;

        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Rgb8, framerate)
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, framerate)
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();

        assert!(pipeline.can_resolve(&config));

        let mut pipeline = pipeline.start(Some(&config)).unwrap();

        let mut nframes = 0usize;
        let number_of_seconds = 5;
        let iters = number_of_seconds * framerate;

        let begin = std::time::SystemTime::now();
        let mut first_iter_time = 0;

        for i in 0..iters {
            let frames = if i == 0 {
                // The first frames captured always seems to have a delay.
                //
                // For the D400, this is 1.5s but can probably get worse than this. Instead, we
                // choose the default timeout for the first frame.
                let frames = pipeline.wait(None).unwrap();
                first_iter_time = begin.elapsed().unwrap().as_millis();
                frames
            } else {
                pipeline.wait(Some(Duration::from_millis(34))).unwrap()
            };
            nframes += frames.count();
        }

        let elapsed_time_ms = begin.elapsed().unwrap().as_millis();
        let expected_time_ms = 1000 * (number_of_seconds as u128);

        let absdiff_from_expected = if elapsed_time_ms > expected_time_ms {
            elapsed_time_ms - expected_time_ms
        } else {
            expected_time_ms - elapsed_time_ms
        };

        assert!(
            absdiff_from_expected <= first_iter_time + 200,
            "Difference in time from expected time: {}",
            absdiff_from_expected
        );

        assert_eq!(nframes, framerate * number_of_seconds * 2);
    }
}

#[test]
fn l500_streams_at_expected_framerate() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::L500);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        let framerate = 30;

        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, framerate)
            .unwrap()
            .enable_stream(
                Rs2StreamKind::Infrared,
                None,
                0,
                0,
                Rs2Format::Y8,
                framerate,
            )
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();

        assert!(pipeline.can_resolve(&config));

        let mut pipeline = pipeline.start(Some(&config)).unwrap();

        let mut nframes = 0usize;
        let number_of_seconds = 5;
        let iters = number_of_seconds * framerate;

        let begin = std::time::SystemTime::now();
        let mut first_iter_time = 0;

        for i in 0..iters {
            let frames = if i == 0 {
                // The first frame captured always seems to have a delay.
                //
                // For the L515, this is observably around 1.5s, but can probably be worse than
                // this. Instead, we choose the default timeout for the first frame.
                let frames = pipeline.wait(None).unwrap();
                first_iter_time = begin.elapsed().unwrap().as_millis();
                frames
            } else {
                pipeline.wait(Some(Duration::from_millis(34))).unwrap()
            };
            nframes += frames.count();
        }

        let elapsed_time_ms = begin.elapsed().unwrap().as_millis();
        let expected_time_ms = 1000 * (number_of_seconds as u128);

        let absdiff_from_expected = if elapsed_time_ms > expected_time_ms {
            elapsed_time_ms - expected_time_ms
        } else {
            expected_time_ms - elapsed_time_ms
        };

        assert!(
            absdiff_from_expected <= first_iter_time + 200,
            "Difference in time from expected time: {}",
            absdiff_from_expected
        );

        assert_eq!(nframes, framerate * number_of_seconds * 2);
    }
}

#[test]
fn d400_streams_are_distinct() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::D400);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        // Gyro / accel streams not included here because they have a different framerate
        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Rgba8, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Infrared, Some(1), 0, 0, Rs2Format::Y8, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Infrared, Some(2), 0, 0, Rs2Format::Any, 30)
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();
        let mut pipeline = pipeline.start(Some(&config)).unwrap();

        let frames = pipeline.wait(None).unwrap();

        assert_eq!(frames.count(), 4);
        assert_eq!(frames.frames_of_type::<ColorFrame>().len(), 1);
        assert_eq!(frames.frames_of_type::<DepthFrame>().len(), 1);
        assert_eq!(frames.frames_of_type::<InfraredFrame>().len(), 2);
    }
}

#[test]
fn l515_streams_are_distinct() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::L500);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        // Gyro / accel streams not included here because they have a different framerate
        config
            .enable_device_from_serial(serial)
            .unwrap()
            .disable_all_streams()
            .unwrap()
            .enable_stream(Rs2StreamKind::Color, None, 0, 0, Rs2Format::Yuyv, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, 30)
            .unwrap()
            .enable_stream(Rs2StreamKind::Infrared, None, 0, 0, Rs2Format::Y8, 30)
            .unwrap();

        let pipeline = InactivePipeline::try_from(&context).unwrap();
        let mut pipeline = pipeline.start(Some(&config)).unwrap();

        let frames = pipeline.wait(None).unwrap();

        assert_eq!(frames.count(), 3);
        assert_eq!(frames.frames_of_type::<ColorFrame>().len(), 1);
        assert_eq!(frames.frames_of_type::<DepthFrame>().len(), 1);
        assert_eq!(frames.frames_of_type::<InfraredFrame>().len(), 1);
    }
}
