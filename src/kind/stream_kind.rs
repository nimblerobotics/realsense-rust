//! Enumeration describing the possible kinds of streams a stream profile can describe.
//!
//! Streams are different types of data provided by RealSense devices.

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;

#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2StreamKind {
    /// Stream kind key to tell librealsense2 to pick the best suited stream kind.
    ///
    /// Unlike the other format entries, `Any` is used primarily when setting up streams in the
    /// config / pipeline. If you pass this, librealsense2 will pick the best suited (default)
    /// stream kinds for a given sensor.
    Any = sys::rs2_stream_RS2_STREAM_ANY,
    /// Native stream of depth data produced by RealSense device
    Depth = sys::rs2_stream_RS2_STREAM_DEPTH,
    /// Native stream of color data captured by RealSense device
    Color = sys::rs2_stream_RS2_STREAM_COLOR,
    /// Native stream of infrared data captured by RealSense device
    Infrared = sys::rs2_stream_RS2_STREAM_INFRARED,
    /// Native stream of fish-eye (wide) data captured from the dedicated motion camera
    Fisheye = sys::rs2_stream_RS2_STREAM_FISHEYE,
    /// Native stream of gyroscope motion data produced by RealSense device
    Gyro = sys::rs2_stream_RS2_STREAM_GYRO,
    /// Native stream of accelerometer motion data produced by RealSense device
    Accel = sys::rs2_stream_RS2_STREAM_ACCEL,
    /// Signals from external device connected through GPIO
    Gpio = sys::rs2_stream_RS2_STREAM_GPIO,
    /// 6DoF pose data, calculated by RealSense device
    Pose = sys::rs2_stream_RS2_STREAM_POSE,
    /// 4-bit per pixel depth confidence values
    Confidence = sys::rs2_stream_RS2_STREAM_CONFIDENCE,
    // Not included since this just tells us the total number stream types
    //
    // Count = sys::rs2_stream_RS2_STREAM_COUNT,
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_traits::FromPrimitive;

    #[test]
    fn all_variants_exist() {
        for i in 0..sys::rs2_stream_RS2_STREAM_COUNT {
            assert!(
                Rs2StreamKind::from_u32(i).is_some(),
                "Rs2StreamKind variant for ordinal {} does not exist.",
                i,
            );
        }
    }
}
