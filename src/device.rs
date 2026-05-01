//! A type for abstracting over the concept of a RealSense "device"
//!
//! A device in librealsense2 refers to a complete set of sensors that comprise e.g. a D400 / L500
//! / T200 unit. A D435 or D435i, for example, is a device, whereas the individual parts that
//! comprise that device (IR cameras, depth camera, color camera, IMU) are referred to as sensors.
//! See [`sensors`](crate::sensor) for more info.

use crate::{
    check_rs2_error,
    kind::{Rs2CameraInfo, Rs2Exception},
    sensor::Sensor,
};
use realsense_sys as sys;
use std::{
    convert::{From, TryFrom, TryInto},
    ffi::CStr,
    ptr::{self, NonNull},
};
use thiserror::Error;

/// Enumeration of possible errors that can occur during device construction
#[derive(Error, Debug)]
pub enum DeviceConstructionError {
    /// System was unable to get the device pointer that corresponds to a given [`Sensor`]
    #[error("Could not create device from sensor. Type: {0}; Reason: {1}")]
    CouldNotCreateDeviceFromSensor(Rs2Exception, String),
    /// Could not get device from device list
    #[error("Could not get device from device list. Type: {0}; Reason: {1}")]
    CouldNotGetDeviceFromDeviceList(Rs2Exception, String),
}

/// Errors returned by auto-calibration operations on [`Device`].
#[derive(Error, Debug)]
pub enum AutoCalibrationError {
    /// The device does not expose the `RS2_EXTENSION_AUTO_CALIBRATED_DEVICE` extension.
    #[error("device does not support auto-calibration (RS2_EXTENSION_AUTO_CALIBRATED_DEVICE)")]
    NotSupported,
    /// On-chip calibration (OCC) failed.
    #[error("on-chip calibration failed: {0}")]
    OccFailed(String),
    /// Applying the in-memory calibration table failed.
    #[error("set calibration table failed: {0}")]
    SetTableFailed(String),
    /// Tare calibration failed.
    #[error("tare calibration failed: {0}")]
    TareFailed(String),
    /// Writing calibration to EEPROM failed.
    #[error("write calibration failed: {0}")]
    WriteFailed(String),
    /// Restoring factory calibration failed.
    #[error("reset to factory calibration failed: {0}")]
    FactoryResetFailed(String),
}

/// Result of an OCC or tare calibration run.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Raw calibration table bytes, suitable for passing to
    /// [`Device::set_calibration_table`].
    pub table: Vec<u8>,
    /// Health score reported by the firmware.  A smaller `|health|` value
    /// indicates better calibration quality.
    pub health: f32,
}

/// A type representing a RealSense device.
///
/// A device in librealsense2 corresponds to a physical unit that connects to your computer
/// (usually via USB). Devices hold a list of sensors, which in turn are represented by a list of
/// streams producing frames.
///
/// Devices are usually acquired by the driver context.
///
#[derive(Debug)]
pub struct Device {
    /// A non-null pointer to the underlying librealsense device
    device_ptr: NonNull<sys::rs2_device>,
}

impl Drop for Device {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_device(self.device_ptr.as_ptr());
        }
    }
}

unsafe impl Send for Device {}

impl From<NonNull<sys::rs2_device>> for Device {
    /// Attempt to construct a Device from a non-null pointer to `rs2_device`.
    ///
    /// Constructs a device from a pointer to an `rs2_device` type from the C-FFI.
    ///
    fn from(device_ptr: NonNull<sys::rs2_device>) -> Self {
        Device { device_ptr }
    }
}

impl Device {
    /// Attempt to construct a Device given a device list and index into the device list.
    ///
    /// # Errors
    ///
    /// Returns [`DeviceConstructionError::CouldNotGetDeviceFromDeviceList`] if the device cannot
    /// be retrieved from the device list (e.g. if the index is invalid).
    ///
    pub(crate) fn try_create(
        device_list: &NonNull<sys::rs2_device_list>,
        index: i32,
    ) -> Result<Self, DeviceConstructionError> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();

            let device_ptr = sys::rs2_create_device(device_list.as_ptr(), index, &mut err);
            check_rs2_error!(
                err,
                DeviceConstructionError::CouldNotGetDeviceFromDeviceList
            )?;

            let nonnull_device_ptr = NonNull::new(device_ptr).unwrap();
            Ok(Device::from(nonnull_device_ptr))
        }
    }

    /// Gets a list of sensors associated with the device.
    ///
    /// Returns a vector of zero size if any error occurs while trying to read the sensor list.
    /// This can occur if the physical device is disconnected before this call is made.
    ///
    pub fn sensors(&self) -> Vec<Sensor> {
        unsafe {
            let mut sensors = Vec::new();

            let mut err = ptr::null_mut::<sys::rs2_error>();
            let sensor_list_ptr = sys::rs2_query_sensors(self.device_ptr.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                sys::rs2_free_error(err);
                return sensors;
            }

            let nonnull_sensor_list = NonNull::new(sensor_list_ptr).unwrap();

            let mut err = ptr::null_mut::<sys::rs2_error>();
            let len = sys::rs2_get_sensors_count(nonnull_sensor_list.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                sys::rs2_free_error(err);
                sys::rs2_delete_sensor_list(nonnull_sensor_list.as_ptr());
                return sensors;
            }

            sensors.reserve(len as usize);
            for i in 0..len {
                match Sensor::try_create(&nonnull_sensor_list, i) {
                    Ok(s) => {
                        sensors.push(s);
                    }
                    Err(_) => {
                        continue;
                    }
                }
            }
            sys::rs2_delete_sensor_list(nonnull_sensor_list.as_ptr());
            sensors
        }
    }

    /// Takes ownership of the device and forces a hardware reset on the device.
    ///
    /// Ownership of the device is taken as the underlying state can no longer be safely retained
    /// after resetting the device.
    ///
    pub fn hardware_reset(self) {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();

            // The only failure this can have is if device_ptr is null. This should not be the case
            // since we're storing a `NonNull` type.
            //
            // It's a bit weird, but we don't need to actually check the error. Because if the
            // device is null and this fails: you have an invalid device (so panic?) but if it
            // succeeds, the device is no longer valid and we need to drop it. This is why this
            // interface takes ownership of `self`.
            sys::rs2_hardware_reset(self.device_ptr.as_ptr(), &mut err);
            if !err.is_null() {
                sys::rs2_free_error(err);
            }
        }
    }

    /// Gets the value associated with the provided camera info key from the device.
    ///
    /// Returns some information value associated with the camera info key if the `camera_info` is
    /// supported by the device, else it returns `None`.
    ///
    pub fn info(&self, camera_info: Rs2CameraInfo) -> Option<&CStr> {
        if !self.supports_info(camera_info) {
            return None;
        }

        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();

            let val = sys::rs2_get_device_info(
                self.device_ptr.as_ptr(),
                #[allow(clippy::useless_conversion)]
                (camera_info as i32).try_into().unwrap(),
                &mut err,
            );

            if err.as_ref().is_none() {
                Some(CStr::from_ptr(val))
            } else {
                sys::rs2_free_error(err);
                None
            }
        }
    }

    /// Predicate for checking if `camera_info` is supported for this device.
    ///
    /// Returns true iff the device has a value associated with the `camera_info` key.
    ///
    pub fn supports_info(&self, camera_info: Rs2CameraInfo) -> bool {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();
            let supports_info = sys::rs2_supports_device_info(
                self.device_ptr.as_ptr(),
                #[allow(clippy::useless_conversion)]
                (camera_info as i32).try_into().unwrap(),
                &mut err,
            );

            if err.as_ref().is_none() {
                supports_info != 0
            } else {
                sys::rs2_free_error(err);
                false
            }
        }
    }

    /// Run on-chip calibration (OCC).
    ///
    /// Blocks until calibration completes or `timeout_ms` elapses.
    ///
    /// `config_json` is a JSON string, e.g. `{"speed": 3, "accuracy": 0}`.
    ///
    /// # Errors
    ///
    /// - [`AutoCalibrationError::NotSupported`] — device lacks the auto-calibration extension.
    /// - [`AutoCalibrationError::OccFailed`] — the C API reported a calibration failure.
    pub fn run_on_chip_calibration(
        &self,
        config_json: &str,
        timeout_ms: u32,
    ) -> Result<CalibrationResult, AutoCalibrationError> {
        self.check_auto_calibration_supported()?;
        let json_bytes = config_json.as_bytes();
        let json_len = i32::try_from(json_bytes.len())
            .map_err(|_| AutoCalibrationError::OccFailed("config_json too large".to_string()))?;
        let timeout = i32::try_from(timeout_ms).unwrap_or(i32::MAX);
        let mut health: f32 = 0.0;
        let mut err = ptr::null_mut::<sys::rs2_error>();
        let raw_data = unsafe {
            sys::rs2_run_on_chip_calibration(
                self.device_ptr.as_ptr(),
                json_bytes.as_ptr().cast(),
                json_len,
                &mut health,
                Some(noop_progress),
                ptr::null_mut(),
                timeout,
                &mut err,
            )
        };
        if !err.is_null() {
            // Defensively free raw_data if C unexpectedly returns both err and data.
            if !raw_data.is_null() {
                unsafe { sys::rs2_delete_raw_data(raw_data) };
            }
            return Err(AutoCalibrationError::OccFailed(drain_rs2_error(err)));
        }
        if raw_data.is_null() {
            return Err(AutoCalibrationError::OccFailed(
                "OCC returned a null calibration table".to_string(),
            ));
        }
        let table = drain_raw_data_buffer(raw_data).map_err(AutoCalibrationError::OccFailed)?;
        Ok(CalibrationResult { table, health })
    }

    /// Apply a calibration table to the device in memory.
    ///
    /// This does **not** persist the table to EEPROM.  Call
    /// [`write_calibration`](Self::write_calibration) afterwards to make the change permanent.
    ///
    /// # Errors
    ///
    /// - [`AutoCalibrationError::NotSupported`] — device lacks the auto-calibration extension.
    /// - [`AutoCalibrationError::SetTableFailed`] — the C API rejected the calibration table.
    pub fn set_calibration_table(&self, table: &[u8]) -> Result<(), AutoCalibrationError> {
        if table.is_empty() {
            return Err(AutoCalibrationError::SetTableFailed(
                "calibration table must not be empty".to_string(),
            ));
        }
        self.check_auto_calibration_supported()?;
        let table_len = i32::try_from(table.len()).map_err(|_| {
            AutoCalibrationError::SetTableFailed("calibration table too large".to_string())
        })?;
        let mut err = ptr::null_mut::<sys::rs2_error>();
        unsafe {
            sys::rs2_set_calibration_table(
                self.device_ptr.as_ptr(),
                table.as_ptr().cast(),
                table_len,
                &mut err,
            );
        }
        if !err.is_null() {
            return Err(AutoCalibrationError::SetTableFailed(drain_rs2_error(err)));
        }
        Ok(())
    }

    /// Run tare calibration against a flat target at a known distance.
    ///
    /// Blocks until calibration completes or `timeout_ms` elapses.
    ///
    /// `ground_truth_mm` is the distance from the camera to the target in millimetres.
    /// Valid range is 60–10 000 mm, as specified by the Intel RealSense D400 Series
    /// Self-Calibration white paper and enforced by the `rs2_run_tare_calibration` C API.
    ///
    /// `config_json` is a JSON string, e.g. `{"speed": 3, "accuracy": 0}`.
    ///
    /// # Errors
    ///
    /// - [`AutoCalibrationError::NotSupported`] — device lacks the auto-calibration extension.
    /// - [`AutoCalibrationError::TareFailed`] — the C API reported a calibration failure.
    pub fn run_tare_calibration(
        &self,
        ground_truth_mm: f32,
        config_json: &str,
        timeout_ms: u32,
    ) -> Result<CalibrationResult, AutoCalibrationError> {
        if !(60.0..=10_000.0).contains(&ground_truth_mm) {
            return Err(AutoCalibrationError::TareFailed(format!(
                "ground_truth_mm {ground_truth_mm} is outside the valid range [60, 10000]"
            )));
        }
        self.check_auto_calibration_supported()?;
        let json_bytes = config_json.as_bytes();
        let json_len = i32::try_from(json_bytes.len())
            .map_err(|_| AutoCalibrationError::TareFailed("config_json too large".to_string()))?;
        let timeout = i32::try_from(timeout_ms).unwrap_or(i32::MAX);
        let mut health: f32 = 0.0;
        let mut err = ptr::null_mut::<sys::rs2_error>();
        let raw_data = unsafe {
            sys::rs2_run_tare_calibration(
                self.device_ptr.as_ptr(),
                ground_truth_mm,
                json_bytes.as_ptr().cast(),
                json_len,
                &mut health,
                Some(noop_progress),
                ptr::null_mut(),
                timeout,
                &mut err,
            )
        };
        if !err.is_null() {
            // Defensively free raw_data if C unexpectedly returns both err and data.
            if !raw_data.is_null() {
                unsafe { sys::rs2_delete_raw_data(raw_data) };
            }
            return Err(AutoCalibrationError::TareFailed(drain_rs2_error(err)));
        }
        if raw_data.is_null() {
            return Err(AutoCalibrationError::TareFailed(
                "tare returned a null calibration table".to_string(),
            ));
        }
        let table = drain_raw_data_buffer(raw_data).map_err(AutoCalibrationError::TareFailed)?;
        Ok(CalibrationResult { table, health })
    }

    /// Flush the current in-memory calibration table to device EEPROM.
    ///
    /// # Errors
    ///
    /// - [`AutoCalibrationError::NotSupported`] — device lacks the auto-calibration extension.
    /// - [`AutoCalibrationError::WriteFailed`] — the C API reported a write failure.
    pub fn write_calibration(&self) -> Result<(), AutoCalibrationError> {
        self.check_auto_calibration_supported()?;
        let mut err = ptr::null_mut::<sys::rs2_error>();
        unsafe {
            sys::rs2_write_calibration(self.device_ptr.as_ptr(), &mut err);
        }
        if !err.is_null() {
            return Err(AutoCalibrationError::WriteFailed(drain_rs2_error(err)));
        }
        Ok(())
    }

    /// Reset the device calibration to its factory defaults.
    ///
    /// # Errors
    ///
    /// - [`AutoCalibrationError::NotSupported`] — device lacks the auto-calibration extension.
    /// - [`AutoCalibrationError::FactoryResetFailed`] — the C API reported a failure.
    pub fn reset_to_factory_calibration(&self) -> Result<(), AutoCalibrationError> {
        self.check_auto_calibration_supported()?;
        let mut err = ptr::null_mut::<sys::rs2_error>();
        unsafe {
            sys::rs2_reset_to_factory_calibration(self.device_ptr.as_ptr(), &mut err);
        }
        if !err.is_null() {
            return Err(AutoCalibrationError::FactoryResetFailed(drain_rs2_error(
                err,
            )));
        }
        Ok(())
    }

    /// Get the underlying low-level pointer to the device object.
    ///
    /// # Safety
    ///
    /// This method is not intended to be called or used outside of the crate itself. Be warned, it
    /// is _undefined behaviour_ to delete or try to drop this pointer in any context. If you do,
    /// you risk a double-free or use-after-free error.
    pub(crate) unsafe fn get_raw(&self) -> NonNull<sys::rs2_device> {
        self.device_ptr
    }

    /// Returns `Ok(())` if the device supports `RS2_EXTENSION_AUTO_CALIBRATED_DEVICE`, else `NotSupported`.
    fn check_auto_calibration_supported(&self) -> Result<(), AutoCalibrationError> {
        let mut err = ptr::null_mut::<sys::rs2_error>();
        let supported = unsafe {
            sys::rs2_is_device_extendable_to(
                self.device_ptr.as_ptr(),
                sys::rs2_extension_RS2_EXTENSION_AUTO_CALIBRATED_DEVICE,
                &mut err,
            )
        };
        if !err.is_null() {
            // If the extension check itself fails we cannot confirm support.
            free_rs2_error(err);
            return Err(AutoCalibrationError::NotSupported);
        }
        if supported == 0 {
            Err(AutoCalibrationError::NotSupported)
        } else {
            Ok(())
        }
    }
}

/// No-op progress callback passed to calibration C API calls that require a function pointer.
unsafe extern "C" fn noop_progress(_progress: f32, _user_data: *mut std::os::raw::c_void) {}

/// Extract the error message from an `rs2_error`, free it, and return the message.
fn drain_rs2_error(err: *mut sys::rs2_error) -> String {
    let msg = unsafe {
        let msg_ptr = sys::rs2_get_error_message(err);
        if msg_ptr.is_null() {
            "unknown librealsense2 error".to_string()
        } else {
            CStr::from_ptr(msg_ptr).to_string_lossy().into_owned()
        }
    };
    free_rs2_error(err);
    msg
}

/// Free an `rs2_error` object.
fn free_rs2_error(err: *mut sys::rs2_error) {
    unsafe { sys::rs2_free_error(err) };
}

/// Copy the bytes out of an `rs2_raw_data_buffer` then free it.
fn drain_raw_data_buffer(buffer: *const sys::rs2_raw_data_buffer) -> Result<Vec<u8>, String> {
    let mut err = ptr::null_mut::<sys::rs2_error>();
    let size_raw = unsafe { sys::rs2_get_raw_data_size(buffer, &mut err) };
    if !err.is_null() {
        let msg = drain_rs2_error(err);
        unsafe { sys::rs2_delete_raw_data(buffer) };
        return Err(msg);
    }
    // A size of 0 is valid but rs2_get_raw_data may return null for an empty buffer,
    // so handle it early to avoid passing a potentially-null pointer to from_raw_parts.
    if size_raw == 0 {
        unsafe { sys::rs2_delete_raw_data(buffer) };
        return Ok(vec![]);
    }
    let size = usize::try_from(size_raw).map_err(|_| {
        unsafe { sys::rs2_delete_raw_data(buffer) };
        format!("rs2_get_raw_data_size returned invalid size: {size_raw}")
    })?;
    let data_ptr = unsafe { sys::rs2_get_raw_data(buffer, &mut err) };
    if !err.is_null() {
        let msg = drain_rs2_error(err);
        unsafe { sys::rs2_delete_raw_data(buffer) };
        return Err(msg);
    }
    if data_ptr.is_null() {
        unsafe { sys::rs2_delete_raw_data(buffer) };
        return Err("rs2_get_raw_data returned a null pointer".to_string());
    }
    let data = unsafe { std::slice::from_raw_parts(data_ptr, size).to_vec() };
    unsafe { sys::rs2_delete_raw_data(buffer) };
    Ok(data)
}
