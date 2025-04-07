//! Type that defines a RealSense context used by the rest of the API

use crate::{
    base::from_path,
    check_rs2_error,
    device::Device,
    device_hub::DeviceHub,
    kind::{Rs2Exception, Rs2ProductLine},
};
use anyhow::Result;
use num_traits::ToPrimitive;
use realsense_sys as sys;
use std::{collections::HashSet, convert::From, path::Path, ptr::NonNull};
use thiserror::Error;

use parking_lot::{Condvar, Mutex};
use std::sync::Arc;

/// Type describing a RealSense context, used by the rest of the API.
#[derive(Debug)]
pub struct Context {
    /// A non-null pointer to the underlying librealsense context.
    context_ptr: NonNull<sys::rs2_context>,
}

/// An error type describing failure to construct a context.
#[derive(Error, Debug, PartialEq)]
#[error("Could not construct the context. Type: {0}; Reason: {1}")]
pub struct ContextConstructionError(pub Rs2Exception, pub String);

/// An error type describing failure to get the device hub from a context.
#[derive(Error, Debug)]
#[error("Could not get the device hub from the context. Type: {0}; Reason: {1}")]
pub struct CouldNotGetDeviceHubError(pub Rs2Exception, pub String);

/// An error type describing failure to add a device from a file.
#[derive(Error, Debug)]
#[error("Could not add a device from file. Type: {0}; Reason: {1}")]
pub struct CouldNotAddDeviceError(pub Rs2Exception, pub String);

/// An error type describing failure to remove a device from a file.
#[derive(Error, Debug)]
#[error("Could not remove device from file. Type: {0}; Reason: {1}")]
pub struct CouldNotRemoveDeviceError(pub Rs2Exception, pub String);

/// An error type describing failure to get the device count.
#[derive(Error, Debug)]
#[error("Could not get the device count. Type: {0}; Reason: {1}")]
pub struct CouldNotGetDeviceCountError(pub Rs2Exception, pub String);

/// An error type describing failure to set the devices changed notifier.
#[derive(Error, Debug)]
#[error("Could not set the devices changed notifier. Type: {0}; Reason: {1}")]
pub struct CouldNotSetDevicesChangedNotifierError(pub Rs2Exception, pub String);

impl Drop for Context {
    fn drop(&mut self) {
        unsafe { sys::rs2_delete_context(self.context_ptr.as_ptr()) }
    }
}

#[repr(C)]
/// A struct that contains a mutex and a condition variable.
/// This is used to notify the main thread when devices are connected.
/// Enforcing C ABI to avoid issues with Rust optimizations.
pub struct DeviceReadyNotifier {
    /// A mutex to lock the ready state.
    ready: Mutex<bool>,
    /// A condition variable to wait for the ready state.
    condition: Condvar,
}

impl DeviceReadyNotifier {
    /// Create a new device ready notifier.
    pub fn new() -> Self {
        Self {
            ready: Mutex::new(false),
            condition: Condvar::new(),
        }
    }

    /// Notify the device ready notifier that a device has been added.
    pub fn notify(&self) {
        let mut ready = self.ready.lock();
        *ready = true;
        self.condition.notify_one();
    }

    /// Wait for the device ready notifier to be notified. Return true if
    /// the device is ready within the timeout, false otherwise.
    pub fn wait_until_timeout(&self, timeout: std::time::Duration) -> bool {
        let mut ready = self.ready.lock();
        if *ready {
            return true;
        }
        !self.condition.wait_for(&mut ready, timeout).timed_out()
    }
}

unsafe impl Send for Context {}

impl Context {
    /// Construct a new context.
    ///
    /// # Errors
    ///
    /// Returns [`ContextConstructionError`] if the context cannot be created.
    ///
    pub fn new() -> Result<Self, ContextConstructionError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let ptr = sys::rs2_create_context(sys::RS2_API_VERSION as i32, &mut err);
            check_rs2_error!(err, ContextConstructionError)?;

            Ok(Self {
                context_ptr: NonNull::new(ptr).unwrap(),
            })
        }
    }

    /// Creates a device hub from the context.
    ///
    /// # Errors
    ///
    /// Returns [`CouldNotGetDeviceHubError`] if the device hub cannot be created.
    ///
    pub fn create_device_hub(&self) -> Result<DeviceHub, CouldNotGetDeviceHubError> {
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let devicehub_ptr = sys::rs2_create_device_hub(self.context_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, CouldNotGetDeviceHubError)?;

            Ok(DeviceHub::from(NonNull::new(devicehub_ptr).unwrap()))
        }
    }

    /// Get a list of devices that are already connected to the host.
    pub fn query_devices(&self, product_mask: HashSet<Rs2ProductLine>) -> Vec<Device> {
        // TODO/TEST: Make sure that an empty mask (therefore giving no filter) gives
        // us _all_ devices, not _no_ devices.

        let mask = if product_mask.is_empty() {
            Rs2ProductLine::Any.to_i32().unwrap()
        } else {
            product_mask.iter().fold(0, |k, v| k | v.to_u32().unwrap()) as i32
        };

        let mut devices = Vec::new();
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let device_list_ptr =
                sys::rs2_query_devices_ex(self.context_ptr.as_ptr(), mask, &mut err);

            if err.as_ref().is_some() {
                sys::rs2_free_error(err);
                return devices;
            }

            let device_list = NonNull::new(device_list_ptr).unwrap();

            let len = sys::rs2_get_device_count(device_list.as_ptr(), &mut err);

            if err.as_ref().is_some() {
                sys::rs2_free_error(err);
                sys::rs2_delete_device_list(device_list.as_ptr());
                return devices;
            }

            for i in 0..len {
                match Device::try_create(&device_list, i) {
                    Ok(d) => {
                        devices.push(d);
                    }
                    Err(_) => {
                        continue;
                    }
                }
            }

            sys::rs2_delete_device_list(device_list.as_ptr());
        }
        devices
    }

    /// Create a new device and add it to the context.
    ///
    /// This adds a "device" at a particular file on the system to the RealSense context. Returns a
    /// handle to the device, or an error if this call fails.
    ///
    /// # Errors
    ///
    /// Returns [`NulError`](std::ffi::NulError) if the provided file path cannot be cleanly
    /// represented as a [`CString`](std::ffi::CString). This usually only occurs if you have null
    /// characters in the path. Constructing a path using the utilties in Rust's [`std::fs`] are
    /// expected to work.
    ///
    /// Returns [`CouldNotAddDeviceError`] if the device cannot be added.
    ///
    pub fn add_device<P>(&mut self, file: P) -> Result<Device>
    where
        P: AsRef<Path>,
    {
        let path = from_path(file)?;
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let device_ptr =
                sys::rs2_context_add_device(self.context_ptr.as_ptr(), path.as_ptr(), &mut err);
            check_rs2_error!(err, CouldNotAddDeviceError)?;

            Ok(Device::from(NonNull::new(device_ptr).unwrap()))
        }
    }

    /// Removes a playback device from the context, if it exists
    ///
    /// This removes a "device" at a particular file on the system from the RealSense context.
    /// Returns nothing (null tuple) or an Error if the device cannot be removed.
    ///
    /// # Errors
    ///
    /// Returns [`CouldNotRemoveDeviceError`] if the device cannot be removed for any reason.
    ///
    pub fn remove_device<P>(&mut self, file: P) -> Result<()>
    where
        P: AsRef<Path>,
    {
        let path = from_path(file)?;
        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            sys::rs2_context_remove_device(self.context_ptr.as_ptr(), path.as_ptr(), &mut err);
            check_rs2_error!(err, CouldNotRemoveDeviceError)?;

            Ok(())
        }
    }

    /// Get the underlying low-level pointer to the context object.
    ///
    /// # Safety
    ///
    /// This method is not intended to be called or used outside of the crate itself. Be warned, it
    /// is _undefined behaviour_ to call [`realsense_sys::rs2_delete_context`] on this pointer. If
    /// you do, you risk a double-free error when the [`Context`] struct itself is dropped.
    ///
    pub(crate) unsafe fn get_raw(&self) -> NonNull<sys::rs2_context> {
        self.context_ptr
    }

    /// Set a callback notifier to be called when devices connected.
    ///
    /// # Safety
    ///
    /// This function is not intended to be called or used outside of the crate itself.
    /// It is safe because it calls the binding of librealsense directly, and
    /// the error handling is done in the crate itself.
    pub fn get_devices_connected_notifier(
        &self,
    ) -> Result<Arc<DeviceReadyNotifier>, CouldNotSetDevicesChangedNotifierError> {
        // Create a boxed copy of is_ready that can be passed to the callback
        let is_ready = Arc::new(DeviceReadyNotifier::new());
        let is_ready_ptr = Arc::into_raw(is_ready.clone());

        unsafe extern "C" fn callback(
            removed: *mut sys::rs2_device_list,
            added: *mut sys::rs2_device_list,
            is_ready_ptr: *mut std::os::raw::c_void,
        ) {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let removed_count = sys::rs2_get_device_count(removed, &mut err);
            if let Err(e) = check_rs2_error!(err, CouldNotGetDeviceCountError) {
                log::error!("Error getting removed device count: {}", e);
                return;
            }
            if removed_count > 0 {
                log::info!("Removed {} RealSense devices", removed_count);
                return;
            }

            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            let added_count = sys::rs2_get_device_count(added, &mut err);
            if let Err(e) = check_rs2_error!(err, CouldNotGetDeviceCountError) {
                log::error!("Error getting added device count: {}", e);
                return;
            }
            if added_count > 0 {
                log::info!("Added {} RealSense devices. Notifying...", added_count);
                let is_ready: Arc<DeviceReadyNotifier> =
                    Arc::from_raw(is_ready_ptr as *const DeviceReadyNotifier);
                is_ready.notify();
            }
        }

        unsafe {
            let mut err = std::ptr::null_mut::<sys::rs2_error>();
            sys::rs2_set_devices_changed_callback(
                self.context_ptr.as_ptr(),
                Some(callback),
                is_ready_ptr as *mut std::os::raw::c_void,
                &mut err,
            );

            // If there's an error, we need to free the heap data to avoid memory leak
            if let Err(err) = check_rs2_error!(err, CouldNotSetDevicesChangedNotifierError) {
                drop(Arc::from_raw(is_ready_ptr as *const DeviceReadyNotifier));
                return Err(err);
            }
        }

        Ok(is_ready)
    }
}
