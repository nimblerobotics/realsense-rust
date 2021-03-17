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
#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2Extension {
    // sensor
    ColorSensor = sys::rs2_extension_RS2_EXTENSION_COLOR_SENSOR,
    MotionSensor = sys::rs2_extension_RS2_EXTENSION_MOTION_SENSOR,
    FishEyeSensor = sys::rs2_extension_RS2_EXTENSION_FISHEYE_SENSOR,
    DepthSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_SENSOR,
    DepthStereoSensor = sys::rs2_extension_RS2_EXTENSION_DEPTH_STEREO_SENSOR,
    SoftwareSensor = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_SENSOR,
    PoseSensor = sys::rs2_extension_RS2_EXTENSION_POSE_SENSOR,
    L500DepthSensor = sys::rs2_extension_RS2_EXTENSION_L500_DEPTH_SENSOR,
    Tm2Sensor = sys::rs2_extension_RS2_EXTENSION_TM2_SENSOR,
    CalibratedSensor = sys::rs2_extension_RS2_EXTENSION_CALIBRATED_SENSOR,
    MaxUsableRangeSensor = sys::rs2_extension_RS2_EXTENSION_MAX_USABLE_RANGE_SENSOR,
    DebugStreamSensor = sys::rs2_extension_RS2_EXTENSION_DEBUG_STREAM_SENSOR,
    // frame
    VideoFrame = sys::rs2_extension_RS2_EXTENSION_VIDEO_FRAME,
    MotionFrame = sys::rs2_extension_RS2_EXTENSION_MOTION_FRAME,
    CompositeFrame = sys::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME,
    DepthFrame = sys::rs2_extension_RS2_EXTENSION_DEPTH_FRAME,
    DisparityFrame = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FRAME,
    PoseFrame = sys::rs2_extension_RS2_EXTENSION_POSE_FRAME,
    Points = sys::rs2_extension_RS2_EXTENSION_POINTS,
    // filter
    DecimationFilter = sys::rs2_extension_RS2_EXTENSION_DECIMATION_FILTER,
    ThresholdFilter = sys::rs2_extension_RS2_EXTENSION_THRESHOLD_FILTER,
    DisparityFilter = sys::rs2_extension_RS2_EXTENSION_DISPARITY_FILTER,
    SpatialFilter = sys::rs2_extension_RS2_EXTENSION_SPATIAL_FILTER,
    TemporalFilter = sys::rs2_extension_RS2_EXTENSION_TEMPORAL_FILTER,
    HoleFillingFilter = sys::rs2_extension_RS2_EXTENSION_HOLE_FILLING_FILTER,
    ZeroOrderFilter = sys::rs2_extension_RS2_EXTENSION_ZERO_ORDER_FILTER,
    RecommendedFilters = sys::rs2_extension_RS2_EXTENSION_RECOMMENDED_FILTERS,
    AutoCalibrationFilter = sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATION_FILTER,
    SequenceIdFilter = sys::rs2_extension_RS2_EXTENSION_SEQUENCE_ID_FILTER,
    // profile
    VideoProfile = sys::rs2_extension_RS2_EXTENSION_VIDEO_PROFILE,
    MotionProfile = sys::rs2_extension_RS2_EXTENSION_MOTION_PROFILE,
    PoseProfile = sys::rs2_extension_RS2_EXTENSION_POSE_PROFILE,
    // device
    SoftwareDevice = sys::rs2_extension_RS2_EXTENSION_SOFTWARE_DEVICE,
    UpdateDevice = sys::rs2_extension_RS2_EXTENSION_UPDATE_DEVICE,
    AutoCalibratedDevice = sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
    // misc
    AdvancedMode = sys::rs2_extension_RS2_EXTENSION_ADVANCED_MODE,
    Record = sys::rs2_extension_RS2_EXTENSION_RECORD,
    Playback = sys::rs2_extension_RS2_EXTENSION_PLAYBACK,
    Pose = sys::rs2_extension_RS2_EXTENSION_POSE,
    WheelOdometer = sys::rs2_extension_RS2_EXTENSION_WHEEL_ODOMETER,
    GlobalTimer = sys::rs2_extension_RS2_EXTENSION_GLOBAL_TIMER,
    Updatable = sys::rs2_extension_RS2_EXTENSION_UPDATABLE,
    Tm2 = sys::rs2_extension_RS2_EXTENSION_TM2,
    Unknown = sys::rs2_extension_RS2_EXTENSION_UNKNOWN,
    Debug = sys::rs2_extension_RS2_EXTENSION_DEBUG,
    Info = sys::rs2_extension_RS2_EXTENSION_INFO,
    Motion = sys::rs2_extension_RS2_EXTENSION_MOTION,
    Options = sys::rs2_extension_RS2_EXTENSION_OPTIONS,
    Video = sys::rs2_extension_RS2_EXTENSION_VIDEO,
    Roi = sys::rs2_extension_RS2_EXTENSION_ROI,
    DepthHuffmanDecoder = sys::rs2_extension_RS2_EXTENSION_DEPTH_HUFFMAN_DECODER,
    Serializable = sys::rs2_extension_RS2_EXTENSION_SERIALIZABLE,
    FirmwareLogger = sys::rs2_extension_RS2_EXTENSION_FW_LOGGER,
    DeviceCalibration = sys::rs2_extension_RS2_EXTENSION_DEVICE_CALIBRATION,
    HdrMerge = sys::rs2_extension_RS2_EXTENSION_HDR_MERGE,
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
        for i in 0..sys::rs2_extension_RS2_EXTENSION_COUNT {
            assert!(
                Rs2Extension::from_u32(i).is_some(),
                "Rs2Extension variant for ordinal {} does not exist.",
                i,
            );
        }
    }
}
