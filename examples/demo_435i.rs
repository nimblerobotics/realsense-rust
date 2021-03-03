use anyhow::{ensure, Result};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{DepthFrame, GyroFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

pub fn main() -> Result<()> {
    // Check for depth or color-compatible devices.
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, 640, 0, Rs2Format::Z16, 30)?
        .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Rgb8, 30)?
        // RealSense doesn't seem to like index zero for the IR cameras on D435i
        //
        // Really not sure why? This seems like an implementation issue, but in practice most
        // won't be after the IR image directly.
        .enable_stream(Rs2StreamKind::Infrared, Some(1), 640, 0, Rs2Format::Y8, 30)?
        .enable_stream(Rs2StreamKind::Infrared, Some(2), 640, 0, Rs2Format::Y8, 30)?
        .enable_stream(Rs2StreamKind::Gyro, None, 0, 0, Rs2Format::Any, 0)?;
    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(&config))?;
    let mut distance = 0.0;
    let mut motion = [0.0, 0.0, 0.0];

    // process frames
    for i in 0..1000 {
        let timeout = Duration::from_millis(5000);
        let frames = pipeline.wait(Some(timeout))?;

        // Get depth
        let mut depth_frames = frames.frames_of_type::<DepthFrame>();
        if !depth_frames.is_empty() {
            let depth_frame = depth_frames.pop().unwrap();
            let tmp_distance =
                depth_frame.distance(depth_frame.width() / 2, depth_frame.height() / 2)?;
            if tmp_distance != 0.0 {
                distance = tmp_distance;
            }
        }

        if i % 10 == 0 {
            // Get gyro
            let motion_frames = frames.frames_of_type::<GyroFrame>();
            if !motion_frames.is_empty() {
                motion = *motion_frames[0].rotational_velocity();
            }
        }

        // Print our results
        println!(
            "Distance of center pixel: {:<10} m | Gyro reading: {:>15}, {:>15}, {:>15}",
            distance, motion[0], motion[1], motion[2]
        );
        //io::stdout().flush().unwrap();
    }

    Ok(())
}
