//! Possible interface extensions as an enumeration.
use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;

/// Enumeration of interface extensions
///
/// `Rs2Extension` is an enumeration type that lists all the possible underlying interfaces that
/// librealsense2 types can be extended to. Most of the types in librealsense2 are pointers to
/// opaque structs, and there is little in the way of type variety or distinction.
///
/// Instead, generic types are effectively type-tagged through the use of an extension enums which
/// represent the true ontological distinction between e.g. two frames or two sensors.
///
/// Here, rather than try to separate these into different types at the Rust level, we do a simple
/// mapping to the underlying C-enum values. The API does not try to expose these where possible
/// and attempts to hide this behind traits.
///
/// Extensions as listed are effectively the "types" of underlying data in the librealsense2
/// system. However, there is only one extension enum, whereas there are plenty of categories of
/// types available. We try to split these into sets of categories:
///
/// # Sensor extensions:
///
/// * [`Rs2Extension::ColorSensor`]
/// * [`Rs2Extension::MotionSensor`]
/// * [`Rs2Extension::FishEyeSensor`]
/// * [`Rs2Extension::DepthSensor`]
/// * [`Rs2Extension::DepthStereoSensor`]
/// * [`Rs2Extension::SoftwareSensor`]
/// * [`Rs2Extension::PoseSensor`]
/// * [`Rs2Extension::L500DepthSensor`]
/// * [`Rs2Extension::Tm2Sensor`]
/// * [`Rs2Extension::CalibratedSensor`]
/// * [`Rs2Extension::MaxUsableRangeSensor`]
/// * [`Rs2Extension::DebugStreamSensor`]
///
/// # Frame extensions:
///
/// * [`Rs2Extension::VideoFrame`]
/// * [`Rs2Extension::MotionFrame`]
/// * [`Rs2Extension::CompositeFrame`]
/// * [`Rs2Extension::DepthFrame`]
/// * [`Rs2Extension::DisparityFrame`]
/// * [`Rs2Extension::PoseFrame`]
/// * [`Rs2Extension::Points`]
///
/// # Filter (processing block) extensions:
///
/// * [`Rs2Extension::DecimationFilter`]
/// * [`Rs2Extension::ThresholdFilter`]
/// * [`Rs2Extension::DisparityFilter`]
/// * [`Rs2Extension::SpatialFilter`]
/// * [`Rs2Extension::TemporalFilter`]
/// * [`Rs2Extension::HoleFillingFilter`]
/// * [`Rs2Extension::ZeroOrderFilter`]
/// * [`Rs2Extension::RecommendedFilters`]
/// * [`Rs2Extension::AutoCalibrationFilter`]
/// * [`Rs2Extension::SequenceIdFilter`]
///
/// # Profile extensions:
///
/// * [`Rs2Extension::VideoProfile`]
/// * [`Rs2Extension::MotionProfile`]
/// * [`Rs2Extension::PoseProfile`]
///
/// # Device extensions:
///
/// * [`Rs2Extension::SoftwareDevice`]
/// * [`Rs2Extension::UpdateDevice`]
/// * [`Rs2Extension::AutoCalibratedDevice`]
///
/// # Miscellaneous extensions:
///
/// * [`Rs2Extension::AdvancedMode`]
/// * [`Rs2Extension::Record`]
/// * [`Rs2Extension::Playback`]
/// * [`Rs2Extension::Pose`]
/// * [`Rs2Extension::WheelOdometer`]
/// * [`Rs2Extension::GlobalTimer`]
/// * [`Rs2Extension::Updatable`]
/// * [`Rs2Extension::Tm2`]
/// * [`Rs2Extension::Unknown`]
/// * [`Rs2Extension::Debug`]
/// * [`Rs2Extension::Info`]
/// * [`Rs2Extension::Motion`]
/// * [`Rs2Extension::Options`]
/// * [`Rs2Extension::Video`]
/// * [`Rs2Extension::Roi`]
///
#[repr(i32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2Extension {
    // sensor
    ColorSensor = sys::rs2_extension_RS2_EXTENSION_COLOR_SENSOR as i32,
    MotionSensor = sys::rs2_extension_RS2_EXTENSION_MOTION_SENSOR as i32,
    FishEyeSensor = sys::rs2_extension_RS2_EXTENSION_FISHEYE_SENSOR as i32,
    DepthSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_SENSOR as i32,
    DepthStereoSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_STEREO_SENSOR as i32,
    SoftwareSensor = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_SENSOR as i32,
    PoseSensor = sys::rs2_extension_RS2_EXTENSION_POSE_SENSOR as i32,
    L500DepthSensor = sys::rs2_extension_RS2_EXTENSION_L500_DEPTH_SENSOR as i32,
    Tm2Sensor = sys::rs2_extension_RS2_EXTENSION_TM2_SENSOR as i32,
    CalibratedSensor = sys::rs2_extension_RS2_EXTENSION_CALIBRATED_SENSOR as i32,
    MaxUsableRangeSensor = sys::rs2_extension_RS2_EXTENSION_MAX_USABLE_RANGE_SENSOR as i32,
    DebugStreamSensor = sys::rs2_extension_RS2_EXTENSION_DEBUG_STREAM_SENSOR as i32,
    // frame
    VideoFrame = sys::rs2_extension_RS2_EXTENSION_VIDEO_FRAME as i32,
    MotionFrame = sys::rs2_extension_RS2_EXTENSION_MOTION_FRAME as i32,
    CompositeFrame = sys::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME as i32,
    DepthFrame = sys::rs2_extension_RS2_EXTENSION_DEPTH_FRAME as i32,
    DisparityFrame = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FRAME as i32,
    PoseFrame = sys::rs2_extension_RS2_EXTENSION_POSE_FRAME as i32,
    Points = sys::rs2_extension_RS2_EXTENSION_POINTS as i32,
    // filter
    DecimationFilter = sys::rs2_extension_RS2_EXTENSION_DECIMATION_FILTER as i32,
    ThresholdFilter = sys::rs2_extension_RS2_EXTENSION_THRESHOLD_FILTER as i32,
    DisparityFilter = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FILTER as i32,
    SpatialFilter = sys::rs2_extension_RS2_EXTENSION_SPATIAL_FILTER as i32,
    TemporalFilter = sys::rs2_extension_RS2_EXTENSION_TEMPORAL_FILTER as i32,
    HoleFillingFilter = sys::rs2_extension_RS2_EXTENSION_HOLE_FILLING_FILTER as i32,
    ZeroOrderFilter = sys::rs2_extension_RS2_EXTENSION_ZERO_ORDER_FILTER as i32,
    RecommendedFilters = sys::rs2_extension_RS2_EXTENSION_RECOMMENDED_FILTERS as i32,
    AutoCalibrationFilter = sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATION_FILTER as i32,
    SequenceIdFilter = sys::rs2_extension_RS2_EXTENSION_SEQUENCE_ID_FILTER as i32,
    // profile
    VideoProfile = sys::rs2_extension_RS2_EXTENSION_VIDEO_PROFILE as i32,
    MotionProfile = sys::rs2_extension_RS2_EXTENSION_MOTION_PROFILE as i32,
    PoseProfile = sys::rs2_extension_RS2_EXTENSION_POSE_PROFILE as i32,
    // device
    SoftwareDevice = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_DEVICE as i32,
    UpdateDevice = sys::rs2_extension_RS2_EXTENSION_UPDATE_DEVICE as i32,
    AutoCalibratedDevice = sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE as i32,
    // misc
    AdvancedMode = sys::rs2_extension_RS2_EXTENSION_ADVANCED_MODE as i32,
    Record = sys::rs2_extension_RS2_EXTENSION_RECORD as i32,
    Playback = sys::rs2_extension_RS2_EXTENSION_PLAYBACK as i32,
    Pose = sys::rs2_extension_RS2_EXTENSION_POSE as i32,
    WheelOdometer = sys::rs2_extension_RS2_EXTENSION_WHEEL_ODOMETER as i32,
    GlobalTimer = sys::rs2_extension_RS2_EXTENSION_GLOBAL_TIMER as i32,
    Updatable = sys::rs2_extension_RS2_EXTENSION_UPDATABLE as i32,
    Tm2 = sys::rs2_extension_RS2_EXTENSION_TM2 as i32,
    Unknown = sys::rs2_extension_RS2_EXTENSION_UNKNOWN as i32,
    Debug = sys::rs2_extension_RS2_EXTENSION_DEBUG as i32,
    Info = sys::rs2_extension_RS2_EXTENSION_INFO as i32,
    Motion = sys::rs2_extension_RS2_EXTENSION_MOTION as i32,
    Options = sys::rs2_extension_RS2_EXTENSION_OPTIONS as i32,
    Video = sys::rs2_extension_RS2_EXTENSION_VIDEO as i32,
    Roi = sys::rs2_extension_RS2_EXTENSION_ROI as i32,
    DepthHuffmanDecoder = sys::rs2_extension_RS2_EXTENSION_DEPTH_HUFFMAN_DECODER as i32,
    Serializable = sys::rs2_extension_RS2_EXTENSION_SERIALIZABLE as i32,
    FirmwareLogger = sys::rs2_extension_RS2_EXTENSION_FW_LOGGER as i32,
    DeviceCalibration = sys::rs2_extension_RS2_EXTENSION_DEVICE_CALIBRATION as i32,
    HdrMerge = sys::rs2_extension_RS2_EXTENSION_HDR_MERGE as i32,
    // Not included since this just tells us the total number of extensions
    //
    // Count = sys::rs2_extension_RS2_EXTENSION_COUNT,
}

pub const SENSOR_EXTENSIONS: [Rs2Extension; 12] = [
    Rs2Extension::ColorSensor,
    Rs2Extension::MotionSensor,
    Rs2Extension::FishEyeSensor,
    Rs2Extension::DepthSensor,
    Rs2Extension::DepthStereoSensor,
    Rs2Extension::SoftwareSensor,
    Rs2Extension::PoseSensor,
    Rs2Extension::L500DepthSensor,
    Rs2Extension::Tm2Sensor,
    Rs2Extension::CalibratedSensor,
    Rs2Extension::MaxUsableRangeSensor,
    Rs2Extension::DebugStreamSensor,
];

pub const FRAME_EXTENSIONS: [Rs2Extension; 7] = [
    Rs2Extension::VideoFrame,
    Rs2Extension::MotionFrame,
    Rs2Extension::CompositeFrame,
    Rs2Extension::DepthFrame,
    Rs2Extension::DisparityFrame,
    Rs2Extension::PoseFrame,
    Rs2Extension::Points,
];

pub const FILTER_EXTENSIONS: [Rs2Extension; 9] = [
    Rs2Extension::DecimationFilter,
    Rs2Extension::ThresholdFilter,
    Rs2Extension::DisparityFilter,
    Rs2Extension::SpatialFilter,
    Rs2Extension::TemporalFilter,
    Rs2Extension::HoleFillingFilter,
    Rs2Extension::ZeroOrderFilter,
    Rs2Extension::RecommendedFilters,
    Rs2Extension::AutoCalibrationFilter,
];

pub const PROFILE_EXTENSIONS: [Rs2Extension; 3] = [
    Rs2Extension::VideoProfile,
    Rs2Extension::MotionProfile,
    Rs2Extension::PoseProfile,
];

pub const DEVICE_EXTENSIONS: [Rs2Extension; 3] = [
    Rs2Extension::SoftwareDevice,
    Rs2Extension::UpdateDevice,
    Rs2Extension::AutoCalibratedDevice,
];

pub const MISC_EXTENSIONS: [Rs2Extension; 15] = [
    Rs2Extension::AdvancedMode,
    Rs2Extension::Record,
    Rs2Extension::Playback,
    Rs2Extension::Pose,
    Rs2Extension::WheelOdometer,
    Rs2Extension::GlobalTimer,
    Rs2Extension::Updatable,
    Rs2Extension::Tm2,
    Rs2Extension::Unknown,
    Rs2Extension::Debug,
    Rs2Extension::Info,
    Rs2Extension::Motion,
    Rs2Extension::Options,
    Rs2Extension::Video,
    Rs2Extension::Roi,
];

#[cfg(test)]
mod tests {
    use super::*;
    use num_traits::FromPrimitive;

    #[test]
    fn all_variants_exist() {
        for i in 0..sys::rs2_extension_RS2_EXTENSION_COUNT as i32 {
            assert!(
                Rs2Extension::from_i32(i).is_some(),
                "Rs2Extension variant for ordinal {} does not exist.",
                i,
            );
        }
    }
}
