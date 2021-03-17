//! Defines the iterable list of devices.

use crate::{
    device::Device,
    error::{ErrorChecker, Result},
};

/// An iterable list of devices.
#[derive(Debug)]
pub struct DeviceList {
    ptr: NonNull<sys::rs2_device_list>,
}

impl DeviceList {
    /// Gets the device at given index.
    ///
    /// The method returns error if index is out of bound given by [DeviceList::len].
    pub fn get(&self, index: usize) -> Result<Device> {
        let device = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr =
                sys::rs2_create_device(self.ptr.as_ptr(), index as c_int, checker.inner_mut_ptr());
            checker.check()?;
            Device::from_raw(ptr)
        };
        Ok(device)
    }

    /// Gets the length of the list.
    pub fn len(&self) -> Result<usize> {
        let len = unsafe {
            let mut checker = ErrorChecker::new();
            let len = sys::rs2_get_device_count(self.ptr.as_ptr(), checker.inner_mut_ptr());
            checker.check()?;
            len
        };
        Ok(len as usize)
    }

    /// Turns into [DeviceListIntoIter] instance that implements [IntoIterator] trait.
    pub fn try_into_iter(self) -> Result<DeviceListIntoIter> {
        let len = self.len()?;
        let ptr = self.into_raw();
        let iter = DeviceListIntoIter {
            index: 0,
            len,
            ptr: NonNull::new(ptr).unwrap(),
            fused: len == 0,
        };
        Ok(iter)
    }

    /// Checks if the device list is empty.
    pub fn is_empty(&self) -> Result<bool> {
        Ok(self.len()? == 0)
    }

    pub fn into_raw(self) -> *mut sys::rs2_device_list {
        let ptr = self.ptr;
        mem::forget(self);
        ptr.as_ptr()
    }

    pub unsafe fn from_raw(ptr: *mut sys::rs2_device_list) -> Self {
        Self {
            ptr: NonNull::new(ptr).unwrap(),
        }
    }
}

impl IntoIterator for DeviceList {
    type Item = Result<Device>;
    type IntoIter = DeviceListIntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.try_into_iter().unwrap()
    }
}

impl Drop for DeviceList {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_device_list(self.ptr.as_ptr());
        }
    }
}

#[derive(Debug)]
pub struct DeviceListIntoIter {
    index: usize,
    len: usize,
    ptr: NonNull<sys::rs2_device_list>,
    fused: bool,
}

impl Iterator for DeviceListIntoIter {
    type Item = Result<Device>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.fused {
            return None;
        }

        let ptr = unsafe {
            let mut checker = ErrorChecker::new();
            let ptr = sys::rs2_create_device(
                self.ptr.as_ptr(),
                self.index as c_int,
                checker.inner_mut_ptr(),
            );
            match checker.check() {
                Ok(()) => ptr,
                Err(err) => {
                    self.fused = true;
                    return Some(Err(err));
                }
            }
        };

        self.index += 1;
        if self.index >= self.len {
            self.fused = true;
        }

        let device = Device {
            ptr: NonNull::new(ptr).unwrap(),
        };
        Some(Ok(device))
    }
}

impl FusedIterator for DeviceListIntoIter {}

unsafe impl Send for DeviceList {}

impl Drop for DeviceListIntoIter {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_delete_device_list(self.ptr.as_ptr());
        }
    }
}
