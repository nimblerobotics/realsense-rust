//! Logging utilities for the realsense-rust crate.
//!
//! This module wrap the unsafe logging functions in the realsense-sys crate.

use crate::{check_rs2_error, kind::Rs2Exception};
use realsense_sys as sys;
use std::ffi::CString;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Represents the severity level of a log message.
pub enum Rs2LogSeverity {
    /// Detailed information about ordinary operations
    Debug,
    /// Terse information about ordinary operations
    Info,
    /// Indication of possible failure
    Warn,
    /// Indication of definite failure
    Error,
    /// Indication of unrecoverable failure
    Fatal,
    /// No logging will occur
    None,
    /// Include any/all log messages
    All,
}

impl From<Rs2LogSeverity> for sys::rs2_log_severity {
    fn from(severity: Rs2LogSeverity) -> Self {
        match severity {
            Rs2LogSeverity::Debug => sys::rs2_log_severity_RS2_LOG_SEVERITY_DEBUG,
            Rs2LogSeverity::Info => sys::rs2_log_severity_RS2_LOG_SEVERITY_INFO,
            Rs2LogSeverity::Warn => sys::rs2_log_severity_RS2_LOG_SEVERITY_WARN,
            Rs2LogSeverity::Error => sys::rs2_log_severity_RS2_LOG_SEVERITY_ERROR,
            Rs2LogSeverity::Fatal => sys::rs2_log_severity_RS2_LOG_SEVERITY_FATAL,
            Rs2LogSeverity::None => sys::rs2_log_severity_RS2_LOG_SEVERITY_NONE,
            Rs2LogSeverity::All => sys::rs2_log_severity_RS2_LOG_SEVERITY_ALL,
        }
    }
}

impl From<sys::rs2_log_severity> for Rs2LogSeverity {
    fn from(severity: sys::rs2_log_severity) -> Self {
        match severity {
            sys::rs2_log_severity_RS2_LOG_SEVERITY_DEBUG => Rs2LogSeverity::Debug,
            sys::rs2_log_severity_RS2_LOG_SEVERITY_INFO => Rs2LogSeverity::Info,
            sys::rs2_log_severity_RS2_LOG_SEVERITY_WARN => Rs2LogSeverity::Warn,
            sys::rs2_log_severity_RS2_LOG_SEVERITY_ERROR => Rs2LogSeverity::Error,
            sys::rs2_log_severity_RS2_LOG_SEVERITY_FATAL => Rs2LogSeverity::Fatal,
            sys::rs2_log_severity_RS2_LOG_SEVERITY_NONE => Rs2LogSeverity::None,
            _ => Rs2LogSeverity::All, // All others map to All
        }
    }
}

impl From<Rs2LogSeverity> for log::Level {
    fn from(severity: Rs2LogSeverity) -> Self {
        match severity {
            Rs2LogSeverity::Debug => log::Level::Debug,
            Rs2LogSeverity::Info => log::Level::Info,
            Rs2LogSeverity::Warn => log::Level::Warn,
            Rs2LogSeverity::Error => log::Level::Error,
            Rs2LogSeverity::Fatal => log::Level::Error,
            Rs2LogSeverity::None => log::Level::Error,
            Rs2LogSeverity::All => log::Level::Trace,
        }
    }
}

impl From<log::Level> for Rs2LogSeverity {
    fn from(level: log::Level) -> Self {
        match level {
            log::Level::Debug => Rs2LogSeverity::Debug,
            log::Level::Info => Rs2LogSeverity::Info,
            log::Level::Warn => Rs2LogSeverity::Warn,
            log::Level::Error => Rs2LogSeverity::Error,
            log::Level::Trace => Rs2LogSeverity::All,
        }
    }
}

#[derive(Error, Debug, PartialEq)] // `PartialEq` is used to compare the error types implemented in `kind::Rs2Exception`.
/// Type describing all possible errors that can occur when trying to log.
pub enum LogError {
    /// Failed to log to callback.
    #[error("Failed to log to callback.")]
    LogToCallbackFailed(Rs2Exception, String),
    /// Failed to create the log directory.
    #[error("Failed to create the log directory.")]
    CreateLogDirectoryFailed(String),
    /// Failed to log to console.
    #[error("Failed to log to console.")]
    LogToConsoleFailed(Rs2Exception, String),
    /// Failed to log to file.
    #[error("Failed to log to file.")]
    LogToFileFailed(Rs2Exception, String),
    /// Failed to reset logger.
    #[error("Failed to reset logger.")]
    ResetLoggerFailed(Rs2Exception, String),
    /// Failed to enable rolling log file.
    #[error("Failed to enable rolling log file.")]
    EnableRollingLogFileFailed(Rs2Exception, String),
    /// Failed to log a message.
    #[error("Failed to log a message.")]
    LogFailed(Rs2Exception, String),
    /// Failed to get full log message.
    #[error("Failed to get full log message.")]
    GetFullLogMessageFailed(Rs2Exception, String),
}

/// Log to callback.
///
/// # Arguments
///
/// * `min_severity` - The minimum severity of the message to log.
fn log_to_callback(level: Rs2LogSeverity) -> Result<(), LogError> {
    /// Callback function for logging.
    unsafe extern "C" fn callback(
        severity: sys::rs2_log_severity,
        message: *const sys::rs2_log_message,
        _: *mut std::os::raw::c_void,
    ) {
        if !message.is_null() {
            // Safety: message pointer is checked for null
            if let Ok(msg) = get_full_log_message(message) {
                let severity = Rs2LogSeverity::from(severity);
                // We set the target to `librealsense::<filepath>::<line_number>` to
                // make the log message appear in the same line in the console output.
                log::log!(
                    target: format!("librealsense::{}:{}", msg.filename, msg.line_number).as_str(),
                    severity.into(),
                    "{}",
                    msg.message
                );
            } else {
                log::error!("Failed to get full log message.");
            }
        }
    }

    unsafe {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        sys::rs2_log_to_callback(level.into(), Some(callback), std::ptr::null_mut(), &mut err);
        check_rs2_error!(err, LogError::LogToCallbackFailed)?;
    }
    Ok(())
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
/// Data structure for log message.
pub struct LogMessage {
    /// Filename of the log message.
    pub filename: String,
    /// Line number of the log message.
    pub line_number: u32,
    /// Message of the log message.
    pub message: String,
}

/// Get full log message.
fn get_full_log_message(message: *const sys::rs2_log_message) -> Result<LogMessage, LogError> {
    unsafe {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        let msg = sys::rs2_get_raw_log_message(message, &mut err);
        let filename = sys::rs2_get_log_message_filename(message, &mut err);
        check_rs2_error!(err, LogError::GetFullLogMessageFailed)?;
        let line_number = sys::rs2_get_log_message_line_number(message, &mut err);
        check_rs2_error!(err, LogError::GetFullLogMessageFailed)?;
        Ok(LogMessage {
            filename: std::ffi::CStr::from_ptr(filename)
                .to_string_lossy()
                .into_owned(),
            line_number,
            message: std::ffi::CStr::from_ptr(msg).to_string_lossy().into_owned(),
        })
    }
}

/// Inject realsense logging to rust log frontend.
pub fn inject_rs_log_to_rust(level: log::Level) -> Result<(), LogError> {
    log_to_callback(level.into())
}

/// Log to console.
///
/// # Arguments
///
/// * `min_severity` - The minimum severity of the message to log.
pub fn log_to_console(min_severity: Rs2LogSeverity) -> Result<(), LogError> {
    unsafe {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        sys::rs2_log_to_console(min_severity.into(), &mut err);
        check_rs2_error!(err, LogError::LogToConsoleFailed)?;
    }
    Ok(())
}

/// Log to file.
///
/// # Arguments
///
/// * `min_severity` - The minimum severity of the message to log.
/// * `file_path` - The path to the file to log to.
pub fn log_to_file(min_severity: Rs2LogSeverity, file_path: &str) -> Result<(), LogError> {
    // create the log directory if it doesn't exist. somehow this doesn't throw
    // error in rs2 sdk
    let dir = std::path::Path::new(file_path)
        .parent()
        .ok_or(LogError::CreateLogDirectoryFailed(file_path.to_string()))?;
    std::fs::create_dir_all(dir).map_err(|e| LogError::CreateLogDirectoryFailed(e.to_string()))?;

    unsafe {
        let filepath = CString::new(file_path).map_err(|_| {
            LogError::LogToFileFailed(
                Rs2Exception::InvalidValue,
                "Failed to convert file path to CString.".to_string(),
            )
        })?;

        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        sys::rs2_log_to_file(min_severity.into(), filepath.as_ptr(), &mut err);
        check_rs2_error!(err, LogError::LogToFileFailed)?;
    }
    Ok(())
}

/// Reset the logger.
pub fn reset_logger() -> Result<(), LogError> {
    unsafe {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        sys::rs2_reset_logger(&mut err);
        check_rs2_error!(err, LogError::ResetLoggerFailed)?;
    }
    Ok(())
}

/// Enable rolling log file when used with rs2_log_to_file:
/// Upon reaching (max_size/2) bytes, the log will be renamed with an ".old"
/// suffix and a new log created. Any previous .old file will be erased.
/// Must have permissions to remove/rename files in log file directory.
///
/// # Arguments
///
/// * `max_size` - The maximum size of the log file.
pub fn enable_rolling_log_file(max_size: u32) -> Result<(), LogError> {
    unsafe {
        let mut err = std::ptr::null_mut::<sys::rs2_error>();
        sys::rs2_enable_rolling_log_file(max_size, &mut err);
        check_rs2_error!(err, LogError::EnableRollingLogFileFailed)?;
    }
    Ok(())
}
