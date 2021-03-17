//! Tests for evaluating connectivity / configuration of sensors

#![cfg(feature = "test-single-device")]

use realsense_rust::{
    config::Config,
    context::Context,
    frame::{ColorFrame, DepthFrame, InfraredFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2Option, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{
    collections::{HashMap, HashSet},
    convert::TryFrom,
    time::Duration,
};

#[test]
fn l500_can_resolve_color_and_depth_and_infrared() {
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
fn l500_streams_at_expected_framerate() {
    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::L500);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();

        let usb_cstr = device.info(Rs2CameraInfo::UsbTypeDescriptor).unwrap();
        let usb_val: f32 = usb_cstr.to_str().unwrap().parse().unwrap();
        let framerate = 30;
        let stream_count: usize;
        if usb_val >= 3.0 {
            stream_count = 2;
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
        } else {
            stream_count = 1;
            config
                .enable_device_from_serial(serial)
                .unwrap()
                .disable_all_streams()
                .unwrap()
                .enable_stream(Rs2StreamKind::Depth, None, 0, 0, Rs2Format::Z16, framerate)
                .unwrap();
        }

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
                pipeline.wait(Some(Duration::from_millis(50))).unwrap()
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

        assert_eq!(nframes, framerate * number_of_seconds * stream_count);
    }
}

#[test]
fn l500_streams_are_distinct() {
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

// Options we will attempt to set
fn possible_options_and_vals_map() -> HashMap<Rs2Option, Option<f32>> {
    let mut options_set = HashMap::<Rs2Option, Option<f32>>::new();
    options_set.insert(Rs2Option::GlobalTimeEnabled, Some(1.0));
    options_set
}

// Options we know are ignored, and their actual returned values on `get_option`
fn supported_but_ignored_options_and_vals_map() -> HashMap<Rs2Option, Option<f32>> {
    let mut options_ignored = HashMap::<Rs2Option, Option<f32>>::new();
    options_ignored.insert(Rs2Option::GlobalTimeEnabled, Some(0.0));
    options_ignored
}

/// Check for supported but ignored sensor options.
///
/// This test is a direct result of decisions made in the Intel RealSense SDK to obfuscate the behavior of a few sensor
/// options. There are a few Options that are registered as "supported" by the sensor, but are actually just set to
/// their default values on runtime. These options are listed in `supported_but_ignored_options_and_vals_map()` above.
///
/// Currently, [Rs2Option::GlobalTimeEnabled] on the L500 is the only setting known to suffer from this. However, this
/// test has been written in a way that makes it easy to test more Options for this same behavior.
#[test]
fn l500_streams_check_supported_but_ignored_sensor_options() {
    let options_to_set = possible_options_and_vals_map();
    let options_ignored = supported_but_ignored_options_and_vals_map();

    let context = Context::new().unwrap();

    let mut queryable_set = HashSet::new();
    queryable_set.insert(Rs2ProductLine::L500);

    let devices = context.query_devices(queryable_set);

    if let Some(device) = devices.get(0) {
        // Grab the sensor list
        for mut sensor in device.sensors() {
            for (option, val) in &options_to_set {
                // We unwrap here because we don't care about the result of the set for this test. RealSense is pretty
                // tricky when it comes to what can be set and what can't; the best way to check this would be to use
                // `sensor.supports_option` or `sensor.is_option_read_only`.
                //
                // However, there are exceptions, as one can see from setting GlobalTimeEnabled on the L500 series.
                sensor.set_option(*option, val.unwrap()).unwrap();
            }
        }
        let serial = device.info(Rs2CameraInfo::SerialNumber).unwrap();
        let mut config = Config::new();
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
        let _pipeline = pipeline.start(Some(&config)).unwrap();

        for sensor in device.sensors() {
            for (option, val) in &options_to_set {
                // Check that the Options we wanted to set are
                // 1. Theoretically supported by the sensor, but
                // 2. Actually discarded when set.
                if options_ignored.contains_key(&option) {
                    assert!(sensor.supports_option(*option));
                    assert_ne!(
                        sensor.get_option(*option),
                        *options_ignored.get(&option).unwrap()
                    );
                }
                // If we get here, it means that the option should actually set successfully. Fail if it's not.
                else {
                    assert_eq!(sensor.get_option(*option), *val);
                }
            }
        }
    }
}
