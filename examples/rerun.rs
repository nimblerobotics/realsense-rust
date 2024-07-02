//! This example opens and streams from a sensor, and visualizes the data in Rerun.

use anyhow::{ensure, Result};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{ColorFrame, DepthFrame, PixelKind},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::{collections::HashSet, convert::TryFrom, io::stdout, time::Duration};

/// Convert an ColorImage to an image::DynamicImage
fn color_image_to_rgb_image(color_frame: &ColorFrame) -> rerun::external::image::DynamicImage {
    let width = color_frame.width() as u32;
    let height = color_frame.height() as u32;
    let pixels = color_frame
        .iter()
        .flat_map(|rs| match rs {
            PixelKind::Bgr8 { b, g, r } => [*r, *g, *b],
            _ => panic!("We got our types wrong!"),
        })
        .collect::<Vec<u8>>();
    rerun::external::image::RgbImage::from_vec(width as u32, height as u32, pixels)
        .unwrap()
        .into()
}

// Convert a Depth image to an image::DynamicImage
fn depth_to_rgb_image(depth_frame: &DepthFrame) -> rerun::external::image::DynamicImage {
    let width = depth_frame.width() as u32;
    let height = depth_frame.height() as u32;
    let units = depth_frame.depth_units().unwrap();
    let color = colorgrad::spectral();
    let pixels = depth_frame
        .iter()
        .flat_map(|rs| match rs {
            PixelKind::Z16 { depth } => {
                let tot_depth = *depth as f32 * units;
                // println!("Depth: {}", tot_depth);
                color.at((tot_depth / 3.0) as f64).to_rgba8()[..3].to_vec()
            }
            _ => panic!("We got our types wrong!"),
        })
        .collect::<Vec<u8>>();
    rerun::external::image::RgbImage::from_vec(width as u32, height as u32, pixels)
        .unwrap()
        .into()
}

fn main() -> Result<()> {
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
        .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Bgr8, 15)?
        .enable_stream(Rs2StreamKind::Depth, None, 0, 480, Rs2Format::Z16, 15)
        .unwrap();

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    // Configure Rerun
    let rec = rerun::RecordingStreamBuilder::new("align example").spawn()?;

    // process frames
    let timeout = Duration::from_millis(1000);
    let mut now = std::time::SystemTime::now();

    loop {
        let frames = pipeline.wait(Some(timeout)).unwrap();
        let color_frames = frames.frames_of_type::<ColorFrame>();
        let depth_frames = frames.frames_of_type::<DepthFrame>();

        if !color_frames.is_empty() {
            let image = color_image_to_rgb_image(&color_frames[0]);
            rec.log("color image", &rerun::Image::try_from(image)?)?;
        }
        if !depth_frames.is_empty() {
            let image = depth_to_rgb_image(&depth_frames[0]);
            rec.log("depth image", &rerun::Image::try_from(image)?)?;
        }

        // Print out our Hz for the loop
        print!(
            "\rCurrent hz: {:.4?} fps",
            1000.0 / (now.elapsed().unwrap().as_millis()) as f64
        );
        use std::io::Write;
        stdout().flush().unwrap();
        now = std::time::SystemTime::now();
    }
}
