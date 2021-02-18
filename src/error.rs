//! Defines the error type used by the crate.

use crate::common::*;

#[doc(hidden)]
#[macro_export]
macro_rules! check_rs2_error {
    ($rs2_error:expr, $result:expr) => {
        // We make this alias here to type check $rs2_error.
        {
            let err: *mut sys::rs2_error = $rs2_error;
            if NonNull::new(err).is_some() {
                Err($result(
                    std::ffi::CStr::from_ptr(sys::rs2_get_error_message(err))
                        .to_str()
                        .unwrap()
                        .to_string(),
                ))
            } else {
                Ok(())
            }
        }
    };
}

#[derive(Debug)]
pub(crate) struct ErrorChecker {
    checked: bool,
    ptr: *mut sys::rs2_error,
}

impl ErrorChecker {
    pub fn new() -> ErrorChecker {
        ErrorChecker {
            checked: false,
            ptr: ptr::null_mut(),
        }
    }

    pub fn inner_mut_ptr(&mut self) -> *mut *mut sys::rs2_error {
        &mut self.ptr as *mut _
    }

    pub fn check(mut self) -> Result<()> {
        self.checked = true;
        match NonNull::new(self.ptr) {
            Some(nonnull) => {
                let msg = get_error_message(nonnull);
                let err = if msg.starts_with("Frame didn't arrive within ") {
                    Error::Timeout(nonnull)
                } else if msg.starts_with("object doesn\'t support option #") {
                    Error::UnsupportedOption(nonnull)
                } else {
                    Error::Other(nonnull)
                };
                Err(err)
            }
            None => Ok(()),
        }
    }
}

impl Drop for ErrorChecker {
    fn drop(&mut self) {
        if !self.checked {
            panic!("internal error: forget to call check()");
        }
    }
}

/// The error type wraps around underlying error thrown by librealsense library.
pub enum Error {
    ToCStrConversion(&'static str),
    Timeout(NonNull<sys::rs2_error>),
    UnsupportedOption(NonNull<sys::rs2_error>),
    Other(NonNull<sys::rs2_error>),
}

impl Error {
    pub fn error_message(&self) -> &str {
        match (self, self.ptr()) {
            (_, Some(ptr)) => get_error_message(ptr),
            (Self::ToCStrConversion(reason), None) => reason,
            _ => unreachable!(),
        }
    }

    pub fn into_raw(self) -> Option<*mut sys::rs2_error> {
        let ptr = self.ptr()?.as_ptr();
        mem::forget(self);
        Some(ptr)
    }

    pub(crate) fn ptr(&self) -> Option<NonNull<sys::rs2_error>> {
        let ptr = match *self {
            Error::ToCStrConversion(_reason) => return None,
            Error::Timeout(ptr) => ptr,
            Error::UnsupportedOption(ptr) => ptr,
            Error::Other(ptr) => ptr,
        };
        Some(ptr)
    }
}

impl Display for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = self.error_message();
        write!(formatter, "RealSense error: {}", message)
    }
}

impl Debug for Error {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        let message = self.error_message();
        write!(formatter, "RealSense error: {}", message)
    }
}

impl StdError for Error {}

unsafe impl Send for Error {}

unsafe impl Sync for Error {}

impl Drop for Error {
    fn drop(&mut self) {
        if let Some(ptr) = self.ptr() {
            unsafe {
                sys::rs2_free_error(ptr.as_ptr());
            }
        }
    }
}

/// A convenient alias Result type.
pub type Result<T> = result::Result<T, Error>;

fn get_error_message<'a>(ptr: NonNull<sys::rs2_error>) -> &'a str {
    unsafe {
        let ptr = sys::rs2_get_error_message(ptr.as_ptr());
        CStr::from_ptr(ptr).to_str().unwrap()
    }
}
