//! Common types and functions.

use crate::common::*;

pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(sys::RS2_DEFAULT_TIMEOUT as u64);

// Thanks, Tenders McChiken.
// https://stackoverflow.com/questions/38948669/whats-the-most-direct-way-to-convert-a-path-to-a-c-char
pub(crate) fn from_path<P>(path: P) -> anyhow::Result<CString>
where
    P: AsRef<std::path::Path>,
{
    let mut buf = Vec::new();

    #[cfg(unix)]
    {
        use std::os::unix::ffi::OsStrExt;
        buf.extend(path.as_ref().as_os_str().as_bytes());
        buf.push(0);
    };

    #[cfg(windows)]
    {
        use std::os::windows::ffi::OsStrExt;
        buf.extend(
            path.as_ref()
                .as_os_str()
                .encode_wide()
                .chain(Some(0))
                .map(|b| {
                    let b = b.to_ne_bytes();
                    b.get(0).map(|s| *s).into_iter().chain(b.get(1).map(|s| *s))
                })
                .flatten(),
        );
    };

    Ok(CString::new(buf)?)
}

/// The intrinsic parameters for motion devices.
pub struct MotionIntrinsics(pub sys::rs2_motion_device_intrinsic);

impl Deref for MotionIntrinsics {
    type Target = sys::rs2_motion_device_intrinsic;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotionIntrinsics {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<sys::rs2_motion_device_intrinsic> for MotionIntrinsics {
    fn as_ref(&self) -> &sys::rs2_motion_device_intrinsic {
        &self.0
    }
}

impl AsMut<sys::rs2_motion_device_intrinsic> for MotionIntrinsics {
    fn as_mut(&mut self) -> &mut sys::rs2_motion_device_intrinsic {
        &mut self.0
    }
}

unsafe impl Send for MotionIntrinsics {}
unsafe impl Sync for MotionIntrinsics {}

/// The intrinsic parameters of stream.
pub struct Intrinsics(pub sys::rs2_intrinsics);

impl Deref for Intrinsics {
    type Target = sys::rs2_intrinsics;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Intrinsics {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<sys::rs2_intrinsics> for Intrinsics {
    fn as_ref(&self) -> &sys::rs2_intrinsics {
        &self.0
    }
}

impl AsMut<sys::rs2_intrinsics> for Intrinsics {
    fn as_mut(&mut self) -> &mut sys::rs2_intrinsics {
        &mut self.0
    }
}

unsafe impl Send for Intrinsics {}
unsafe impl Sync for Intrinsics {}

/// The extrinsic parameters of stream.
pub struct Extrinsics(pub sys::rs2_extrinsics);

#[cfg(feature = "with-nalgebra")]
impl Extrinsics {
    pub fn to_isometry(&self) -> Isometry3<f32> {
        let rotation = {
            let matrix = MatrixMN::<f32, U3, U3>::from_iterator(self.0.rotation.iter().copied());
            UnitQuaternion::from_matrix(&matrix)
        };
        let translation = {
            let [x, y, z] = self.0.translation;
            Translation3::new(x, y, z)
        };
        Isometry3::from_parts(translation, rotation)
    }
}

impl Deref for Extrinsics {
    type Target = sys::rs2_extrinsics;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Extrinsics {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl AsRef<sys::rs2_extrinsics> for Extrinsics {
    fn as_ref(&self) -> &sys::rs2_extrinsics {
        &self.0
    }
}

impl AsMut<sys::rs2_extrinsics> for Extrinsics {
    fn as_mut(&mut self) -> &mut sys::rs2_extrinsics {
        &mut self.0
    }
}

unsafe impl Send for Extrinsics {}
unsafe impl Sync for Extrinsics {}
