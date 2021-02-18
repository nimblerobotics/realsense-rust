//! Enumeration of sensor and device information keys.

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;

/// A type describing the different keys used to access camera info from devices and sensors.
///
/// Each key corresponds to a particular type of device or sensor-specific metadata (known as
/// `info` in the librealsense2 API). Not all keys are supported on all devices or sensors.
///
/// All values that correspond to these keys are returned in the lower level API as `const char*`
/// types, or C-strings. We wrap these values in the `realsense-rust` API as `&CStr` types.
///
#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2CameraInfo {
    /// The name of the sensor or device.
    Name = sys::rs2_camera_info_RS2_CAMERA_INFO_NAME,
    /// The serial number of the device.
    SerialNumber = sys::rs2_camera_info_RS2_CAMERA_INFO_SERIAL_NUMBER,
    /// The firmware version that the device is running.
    FirmwareVersion = sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_VERSION,
    /// The recommended firmware version for a given device.
    ///
    /// The value that corresponds to this key may change depending on what version of
    /// librealsense2 this crate is built against!
    RecommendedFirmwareVersion = sys::rs2_camera_info_RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION,
    /// A description of the unique identifier of the physical port that the device is connected to.
    ///
    /// The format of the value associated with this key will be platform specific.
    PhysicalPort = sys::rs2_camera_info_RS2_CAMERA_INFO_PHYSICAL_PORT,
    /// If the device supports firmware logging, this is the command you send to get those logs.
    DebugOpCode = sys::rs2_camera_info_RS2_CAMERA_INFO_DEBUG_OP_CODE,
    /// Tells you whether the device is in advanced mode.
    AdvancedMode = sys::rs2_camera_info_RS2_CAMERA_INFO_ADVANCED_MODE,
    /// The product identifier for the device (as reported by its USB descriptor).
    ProductId = sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_ID,
    /// Tells you whether the EEPROM is locked.
    CameraLocked = sys::rs2_camera_info_RS2_CAMERA_INFO_CAMERA_LOCKED,
    /// Tells you the designated USB specification (i.e. USB2 or USB3).
    UsbTypeDescriptor = sys::rs2_camera_info_RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
    /// Device product line (e.g. D400 / SR300 / L500 / T200)
    ProductLine = sys::rs2_camera_info_RS2_CAMERA_INFO_PRODUCT_LINE,
    /// The ASIC serial number of the device.
    AsicSerialNumber = sys::rs2_camera_info_RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER,
    /// Provides the firmware update identifier for the device.
    FirmwareUpdateId = sys::rs2_camera_info_RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID,
    // Not included since this just tells us the total number of camera info options
    //
    // Count = sys::rs2_camera_info_RS2_CAMERA_INFO_COUNT,
}
