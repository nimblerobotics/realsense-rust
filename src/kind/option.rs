//! Enumeration of RS2 sensor options.
//!
//! Not all options apply to every sensor. In order to retrieve the correct options,
//! one must iterate over the `sensor` object for option compatibility.
//!
//! Notice that this option refers to the `sensor`, not the device. However, the device
//! used also matters; sensors that are alike across devices are not guaranteed to share
//! the same sensor options. Again, it is up to the user to query whether an option
//! is supported by the sensor before attempting to set it. Failure to do so may cause
//! an error in operation.

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;
use std::ffi::CStr;
use thiserror::Error;

/// Occur when an option cannot be set.
#[derive(Error, Debug)]
pub enum OptionSetError {
    /// The requested option is not supported by this sensor.
    #[error("Option not supported on this sensor.")]
    OptionNotSupported,
    /// The requested option is read-only and cannot be set.
    #[error("Option is read only.")]
    OptionIsReadOnly,
    /// The requested option could not be set. Reason is reported by the sensor.
    #[error("Could not set option. Reason: {0}")]
    CouldNotSetOption(String),
}

/// The enumeration of options available in the RealSense SDK.
///
/// The majority of the options presented have a specific range of valid values. Run
/// `sensor.get_option_range(Rs2Option::_)` to retrieve possible values of an Option type for your sensor.
/// Setting a bad value will lead to a no-op at best, and a malfunction at worst.
///
/// # Deprecated Options
///
/// `AmbientLight`
///
/// - Equivalent to `RS2_OPTION_AMBIENT_LIGHT`
/// - Replacement: [Rs2Option::DigitalGain].
/// - Old Description: "Change the depth ambient light see rs2_ambient_light for values".
///
/// `ZeroOrderEnabled`
///
/// - Equivalent to `RS2_OPTION_ZERO_ORDER_ENABLED`
/// - Replacement: N/A.
/// - Old Description: "Toggle Zero-Order mode."
///
/// `ZeroOrderPointX`
///
/// - Equivalent to `RS2_OPTION_ZERO_ORDER_POINT_X`
/// - Replacement: N/A.
/// - Old Description: "Get the Zero order point x."
///
/// `ZeroOrderPointY`
///
/// - Equivalent to `RS2_OPTION_ZERO_ORDER_POINT_Y`
/// - Replacement: N/A.
/// - Old Description: "Get the Zero order point y."
///
#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2Option {
    /// Enable/disable color backlight compensation.
    BacklightCompensation = sys::rs2_option_RS2_OPTION_BACKLIGHT_COMPENSATION,
    /// Set color image brightness.
    Brightness = sys::rs2_option_RS2_OPTION_BRIGHTNESS,
    /// Set color image contrast.
    Contrast = sys::rs2_option_RS2_OPTION_CONTRAST,
    /// Set exposure time of color camera. Setting any value will disable auto exposure.
    Exposure = sys::rs2_option_RS2_OPTION_EXPOSURE,
    /// Set color image gain.
    Gain = sys::rs2_option_RS2_OPTION_GAIN,
    /// Set color image gamma setting.
    Gamma = sys::rs2_option_RS2_OPTION_GAMMA,
    /// Set color image hue.
    Hue = sys::rs2_option_RS2_OPTION_HUE,
    /// Set color image saturation.
    Saturation = sys::rs2_option_RS2_OPTION_SATURATION,
    /// Set color image sharpness.
    Sharpness = sys::rs2_option_RS2_OPTION_SHARPNESS,
    /// Set white balance of color image. Setting any value will disable auto white balance.
    WhiteBalance = sys::rs2_option_RS2_OPTION_WHITE_BALANCE,
    /// Enable/disable color image auto-exposure.
    EnableAutoExposure = sys::rs2_option_RS2_OPTION_ENABLE_AUTO_EXPOSURE,
    /// Enable/disable color image auto-white-balance
    EnableAutoWhiteBalance = sys::rs2_option_RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
    /// Set the visual preset on the sensor. `sensor.get_option_range()` provides
    /// access to several recommend sets of option presets for a depth camera. The preset
    /// selection varies between devices and sensors.
    VisualPreset = sys::rs2_option_RS2_OPTION_VISUAL_PRESET,
    /// Set the power of the laser emitter, with 0 meaning projector off.
    LaserPower = sys::rs2_option_RS2_OPTION_LASER_POWER,
    /// Set the number of patterns projected per frame. The higher the accuracy value,
    /// the more patterns projected. Increasing the number of patterns helps to achieve
    /// better accuracy. Note that this control affects Depth FPS.
    Accuracy = sys::rs2_option_RS2_OPTION_ACCURACY,
    /// Set the motion vs. range trade-off. Lower values allow for better motion sensitivity.
    /// Higher values allow for better depth range.
    MotionRange = sys::rs2_option_RS2_OPTION_MOTION_RANGE,
    /// Set the filter to apply to each depth frame. Each one of the filter is optimized per the
    /// application requirements.
    FilterOption = sys::rs2_option_RS2_OPTION_FILTER_OPTION,
    /// Set the confidence level threshold used by the Depth algorithm pipe.
    /// This determines whether a pixel will get a valid range or will be marked as invalid.
    ConfidenceThreshold = sys::rs2_option_RS2_OPTION_CONFIDENCE_THRESHOLD,
    /// Enable/disable emitters. Emitter selection:
    ///
    /// - `0`: disable all emitters
    /// - `1`: enable laser
    /// - `2`: enable auto laser
    /// - `3`: enable LED
    ///
    EmitterEnabled = sys::rs2_option_RS2_OPTION_EMITTER_ENABLED,
    /// Set the number of frames the user is allowed to keep per stream.
    /// Trying to hold on to more frames will cause frame drops.
    FramesQueueSize = sys::rs2_option_RS2_OPTION_FRAMES_QUEUE_SIZE,
    /// Get the total number of detected frame drops from all streams.
    TotalFrameDrops = sys::rs2_option_RS2_OPTION_TOTAL_FRAME_DROPS,
    /// Set the auto-exposure mode:
    ///
    /// - Static
    /// - Anti-Flicker
    /// - Hybrid
    ///
    AutoExposureMode = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_MODE,
    /// Set the power line frequency control for anti-flickering:
    ///
    /// - Off
    /// - 50Hz
    /// - 60Hz
    /// - Auto
    ///
    PowerLineFrequency = sys::rs2_option_RS2_OPTION_POWER_LINE_FREQUENCY,
    /// Get the current Temperature of the ASIC.
    AsicTemperature = sys::rs2_option_RS2_OPTION_ASIC_TEMPERATURE,
    /// Enable/disable error handling.
    ErrorPollingEnabled = sys::rs2_option_RS2_OPTION_ERROR_POLLING_ENABLED,
    /// Get the Current Temperature of the projector.
    ProjectorTemperature = sys::rs2_option_RS2_OPTION_PROJECTOR_TEMPERATURE,
    /// Enable/disable trigger to be outputed from the camera to any external device on
    /// every depth frame.
    OutputTriggerEnabled = sys::rs2_option_RS2_OPTION_OUTPUT_TRIGGER_ENABLED,
    /// Get the current Motion-Module Temperature.
    MotionModuleTemperature = sys::rs2_option_RS2_OPTION_MOTION_MODULE_TEMPERATURE,
    /// Set the number of meters represented by a single depth unit.
    DepthUnits = sys::rs2_option_RS2_OPTION_DEPTH_UNITS,
    /// Enable/Disable automatic correction of the motion data.
    EnableMotionCorrection = sys::rs2_option_RS2_OPTION_ENABLE_MOTION_CORRECTION,
    /// Allows sensor to dynamically ajust the frame rate depending on lighting conditions.
    AutoExposurePriority = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_PRIORITY,
    /// Set the color scheme for data visualization.
    ColorScheme = sys::rs2_option_RS2_OPTION_COLOR_SCHEME,
    /// Enable/disable histogram equalization post-processing on the depth data.
    HistogramEqualizationEnabled = sys::rs2_option_RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED,
    /// Set the Minimal distance to the target.
    MinDistance = sys::rs2_option_RS2_OPTION_MIN_DISTANCE,
    /// Set the Maximum distance to the target.
    MaxDistance = sys::rs2_option_RS2_OPTION_MAX_DISTANCE,
    /// Get the texture mapping stream unique ID.
    TextureSource = sys::rs2_option_RS2_OPTION_TEXTURE_SOURCE,
    /// Set the 2D-filter effect. The specific interpretation is given within the context of the filter.
    FilterMagnitude = sys::rs2_option_RS2_OPTION_FILTER_MAGNITUDE,
    /// Set the 2D-filter parameter that controls the weight/radius for smoothing.
    FilterSmoothAlpha = sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_ALPHA,
    /// Set the 2D-filter range/validity threshold.
    FilterSmoothDelta = sys::rs2_option_RS2_OPTION_FILTER_SMOOTH_DELTA,
    /// Enhance depth data post-processing with holes filling where appropriate.
    HolesFill = sys::rs2_option_RS2_OPTION_HOLES_FILL,
    /// Get the distance in mm between the first and the second imagers in stereo-based depth cameras.
    StereoBaseline = sys::rs2_option_RS2_OPTION_STEREO_BASELINE,
    /// Allows dynamically ajust the converge step value of the target exposure in
    /// the Auto-Exposure algorithm.
    AutoExposureConvergeStep = sys::rs2_option_RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP,
    /// Impose Inter-camera HW synchronization mode. Applicable for D400/L500/Rolling Shutter SKUs.
    InterCamSyncMode = sys::rs2_option_RS2_OPTION_INTER_CAM_SYNC_MODE,
    /// Select a stream to process.
    StreamFilter = sys::rs2_option_RS2_OPTION_STREAM_FILTER,
    /// Select a stream format to process.
    StreamFormatFilter = sys::rs2_option_RS2_OPTION_STREAM_FORMAT_FILTER,
    /// Select a stream index to process.
    StreamIndexFilter = sys::rs2_option_RS2_OPTION_STREAM_INDEX_FILTER,
    /// When supported, this option make the camera to switch the emitter state every frame.
    /// 0 for disabled, 1 for enabled.
    EmitterOnOff = sys::rs2_option_RS2_OPTION_EMITTER_ON_OFF,
    /// Get the LDD temperature.
    LldTemperature = sys::rs2_option_RS2_OPTION_LLD_TEMPERATURE,
    /// Get the MC temperature.
    McTemperature = sys::rs2_option_RS2_OPTION_MC_TEMPERATURE,
    /// Get the MA temperature.
    MaTemperature = sys::rs2_option_RS2_OPTION_MA_TEMPERATURE,
    /// Hardware stream configuration.
    HardwarePreset = sys::rs2_option_RS2_OPTION_HARDWARE_PRESET,
    /// Enable/disable global time.
    GlobalTimeEnabled = sys::rs2_option_RS2_OPTION_GLOBAL_TIME_ENABLED,
    /// Get the APD temperature.
    ApdTemperature = sys::rs2_option_RS2_OPTION_APD_TEMPERATURE,
    /// Enable/disable an internal map.
    EnableMapping = sys::rs2_option_RS2_OPTION_ENABLE_MAPPING,
    /// Enable/disable appearance-based relocalization.
    EnableRelocalization = sys::rs2_option_RS2_OPTION_ENABLE_RELOCALIZATION,
    /// Enable/disable position jumping.
    EnablePoseJumping = sys::rs2_option_RS2_OPTION_ENABLE_POSE_JUMPING,
    /// Enable/disable dynamic calibration.
    EnableDynamicCalibration = sys::rs2_option_RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION,
    /// Get the offset from sensor to depth origin in millimeters.
    DepthOffset = sys::rs2_option_RS2_OPTION_DEPTH_OFFSET,
    /// Set the power of the LED (light emitting diode), with 0 meaning off
    LedPower = sys::rs2_option_RS2_OPTION_LED_POWER,
    /// Preserve the previous map when starting.
    EnableMapPreservation = sys::rs2_option_RS2_OPTION_ENABLE_MAP_PRESERVATION,
    /// Enable/disable sensor shutdown when a free-fall is detected (on by default).
    FreefallDetectionEnabled = sys::rs2_option_RS2_OPTION_FREEFALL_DETECTION_ENABLED,
    /// Changes the exposure time of Avalanche Photo Diode in the receiver.
    AvalanchePhotoDiode = sys::rs2_option_RS2_OPTION_AVALANCHE_PHOTO_DIODE,
    /// Changes the amount of sharpening in the post-processed image.
    PostProcessingSharpening = sys::rs2_option_RS2_OPTION_POST_PROCESSING_SHARPENING,
    /// Changes the amount of sharpening in the pre-processed image.
    PreProcessingSharpening = sys::rs2_option_RS2_OPTION_PRE_PROCESSING_SHARPENING,
    /// Control edges and background noise.
    NoiseFiltering = sys::rs2_option_RS2_OPTION_NOISE_FILTERING,
    /// Enable/disable pixel invalidation.
    InvalidationBypass = sys::rs2_option_RS2_OPTION_INVALIDATION_BYPASS,
    /// Change the depth digital gain see rs2_digital_gain for values.
    DigitalGain = sys::rs2_option_RS2_OPTION_DIGITAL_GAIN,
    /// The resolution mode: see rs2_sensor_mode for values.
    SensoeMode = sys::rs2_option_RS2_OPTION_SENSOR_MODE,
    /// Enable/disable Laser On constantly (GS SKU Only).
    EmitterAlwaysOn = sys::rs2_option_RS2_OPTION_EMITTER_ALWAYS_ON,
    /// Depth Thermal Compensation for selected D400 SKUs.
    ThermalCompensation = sys::rs2_option_RS2_OPTION_THERMAL_COMPENSATION,
    /// Enable/disable depth & color frame sync with periodic calibration for proper alignment.
    TriggerCameraAccuracyHealth = sys::rs2_option_RS2_OPTION_TRIGGER_CAMERA_ACCURACY_HEALTH,
    /// Reset Camera Accuracy metric (if affected by TriggerCameraAccuracyHealth option).
    ResetCameraAccuracyHealth = sys::rs2_option_RS2_OPTION_RESET_CAMERA_ACCURACY_HEALTH,
    /// Set host performance mode to optimize device settings so host can keep up with workload.
    /// Take USB transaction granularity as an example. Setting option to low performance host leads
    /// to larger USB transaction sizes and a reduced number of transactions. This improves performance
    /// and stability if the host machine is relatively weak compared to the workload.
    HostPerformance = sys::rs2_option_RS2_OPTION_HOST_PERFORMANCE,
    /// Enable/disable HDR.
    HDREnabled = sys::rs2_option_RS2_OPTION_HDR_ENABLED,
    /// Get HDR Sequence name.
    SequenceName = sys::rs2_option_RS2_OPTION_SEQUENCE_NAME,
    /// Get HDR Sequence size.
    SequenceSize = sys::rs2_option_RS2_OPTION_SEQUENCE_SIZE,
    /// Get HDR Sequence ID - 0 is not HDR; sequence ID for HDR configuration starts from 1.
    SequenceId = sys::rs2_option_RS2_OPTION_SEQUENCE_ID,
    /// Get Humidity temperature [in Celsius].
    HumidityTemperature = sys::rs2_option_RS2_OPTION_HUMIDITY_TEMPERATURE,
    /// Enable/disable the maximum usable depth sensor range given the amount of ambient light in the scene.
    EnableMaxUsableRange = sys::rs2_option_RS2_OPTION_ENABLE_MAX_USABLE_RANGE,
    /// Enable/disable the alternate IR, When enabling alternate IR, the IR image is holding the amplitude of the depth correlation.
    AlternateIR = sys::rs2_option_RS2_OPTION_ALTERNATE_IR,
    /// Get an estimation of the noise on the IR image.
    NoiseEstimation = sys::rs2_option_RS2_OPTION_NOISE_ESTIMATION,
    /// Enable/disable data collection for calculating IR pixel reflectivity.
    EnableIRReflectivity = sys::rs2_option_RS2_OPTION_ENABLE_IR_REFLECTIVITY,
    // Not included since this just tells us the total number of options.
    //
    // Count = sys::rs2_option_RS2_OPTION_COUNT,
}

impl Rs2Option {
    /// Get the option as a CStr.
    pub fn to_cstr(&self) -> &'static CStr {
        unsafe {
            let ptr = sys::rs2_option_to_string(*self as sys::rs2_option);
            CStr::from_ptr(ptr)
        }
    }

    /// Get the option as a str.
    pub fn to_str(&self) -> &'static str {
        self.to_cstr().to_str().unwrap()
    }
}

impl ToString for Rs2Option {
    fn to_string(&self) -> String {
        self.to_str().to_owned()
    }
}

/// The range of available values of a supported option.
pub struct Rs2OptionRange {
    /// The minimum value which will be accepted for this option
    pub min: f32,
    /// The maximum value which will be accepted for this option
    pub max: f32,
    /// The granularity of options which accept discrete values, or zero if the option accepts
    /// continuous values
    pub step: f32,
    /// The default value of the option
    pub default: f32,
}
