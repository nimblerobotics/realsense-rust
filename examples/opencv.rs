//! This example opens and streams from a sensor, copys the frame data to an OpenCV
//! Mat and shows the frame's contents in an OpenCV Window

use anyhow::{ensure, Result};
use opencv::{core, highgui, imgproc, prelude::*};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::PixelKind,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

/// Converts a RealSense ColorFrame with BGR8 color to an
/// OpenCV mat also with BGR8 color
fn mat_from_color(color_frame: &ColorFrame) -> core::Mat {
    let mut color_mat = unsafe {
        Mat::new_rows_cols(
            color_frame.height() as i32,
            color_frame.width() as i32,
            core::CV_8UC3,
        )
        .unwrap()
    };

    for (i, rs) in color_frame.iter().enumerate() {
        match rs {
            PixelKind::Bgr8 { b, g, r } => {
                *color_mat.at_mut::<core::Vec3<u8>>(i as i32).unwrap() = [*b, *g, *r].into();
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    color_mat
}

/// Converts a RealSense DepthFrame with 16-bit depth to a
/// floating point OpenCV mat with depth in meters.
fn mat_from_depth16(depth_frame: &DepthFrame) -> core::Mat {
    let mut depth_mat = unsafe {
        Mat::new_rows_cols(
            depth_frame.height() as i32,
            depth_frame.width() as i32,
            core::CV_32F,
        )
        .unwrap()
    };

    let depth_unit = depth_frame.depth_units().unwrap();
    for (i, rs) in depth_frame.iter().enumerate() {
        match rs {
            PixelKind::Z16 { depth } => {
                let depthf = *depth as f32 * depth_unit;
                *depth_mat.at_mut::<f32>(i as i32).unwrap() = depthf;
            }
            _ => panic!("We got our types wrong!"),
        }
    }

    depth_mat
}

/// Colorizes a single channel OpenCV mat. The mat's current
/// range will be mapped to a [0..255] range and then a color map
/// is applied.
fn colorized_mat(mat: &core::Mat) -> core::Mat {
    let mut normalized = Mat::default();
    core::normalize(
        &mat,
        &mut normalized,
        0.0,
        255.0,
        core::NORM_MINMAX,
        core::CV_8UC1,
        &core::no_array().unwrap(),
    )
    .unwrap();

    let mut colorized = Mat::default();
    imgproc::apply_color_map(&normalized, &mut colorized, imgproc::COLORMAP_JET).unwrap();

    colorized
}

pub fn main() -> Result<()> {
    // Check for depth or color-compatible devices.
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Bgr8, 30)?
        .enable_stream(Rs2StreamKind::Depth, None, 0, 240, Rs2Format::Z16, 30)
        .unwrap();

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    let color_window = "color";
    highgui::named_window(color_window, highgui::WINDOW_AUTOSIZE)?;
    let depth_window = "depth";
    highgui::named_window(depth_window, highgui::WINDOW_AUTOSIZE)?;

    println!("Press any key to quit.");

    // process frames
    let timeout = Duration::from_millis(2000);
    loop {
        let frames = pipeline.wait(Some(timeout)).unwrap();
        let color_frames = frames.frames_of_type::<ColorFrame>();
        let depth_frames = frames.frames_of_type::<DepthFrame>();

        if !color_frames.is_empty() {
            let color_frame = &color_frames[0];
            let color_mat = mat_from_color(color_frame);
            highgui::imshow(color_window, &color_mat).unwrap();
            if highgui::wait_key(2)? != -1 {
                break;
            }
        }
        if !depth_frames.is_empty() {
            let depth_frame = &depth_frames[0];

            let depth_mat = mat_from_depth16(depth_frame);
            let colorized_depth = colorized_mat(&depth_mat);

            highgui::imshow(depth_window, &colorized_depth).unwrap();

            // Show the depth value of the middle of the image in the window's title bar
            let center = depth_mat
                .at_2d::<f32>(depth_mat.rows() / 2, depth_mat.cols() / 2)
                .unwrap();
            let center_str = format!("CenterDepth: {} (m)", center);
            highgui::set_window_title(depth_window, &center_str).unwrap();

            if highgui::wait_key(2)? != -1 {
                break;
            }
        }
    }

    Ok(())
}
