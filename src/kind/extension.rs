//! The enumeration of extensions.

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;

#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Extension {
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
    // Not included since this just tells us the total number of extensions
    //
    // Count = sys::rs2_extension_RS2_EXTENSION_COUNT,
}