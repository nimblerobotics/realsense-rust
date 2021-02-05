//! Type for representing a pose frame taken from an IMU or pose-sensor.

use super::frame_traits::FrameConstructionError;
use super::kind::Kind;
use crate::{check_rs2_error, common::*, stream};

struct PoseFrame {
    frame_ptr: NonNull<sys::rs2_frame>,
    frame_stream_profile: stream::Profile,
    data: sys::rs2_pose,
}

pub enum Confidence {
    Failed,
    Low,
    Medium,
    High,
}

impl PoseFrame {
    pub fn profile(&self) -> &stream::Profile {
        &self.frame_stream_profile
    }

    pub fn translation(&self) -> [f32; 3] {
        let sys::rs2_vector { x, y, z } = self.data.translation;
        [x, y, z]
    }

    pub fn velocity(&self) -> [f32; 3] {
        let sys::rs2_vector { x, y, z } = self.data.velocity;
        [x, y, z]
    }

    pub fn acceleration(&self) -> [f32; 3] {
        let sys::rs2_vector { x, y, z } = self.data.acceleration;
        [x, y, z]
    }

    pub fn rotation(&self) -> [f32; 4] {
        let sys::rs2_quaternion { x, y, z, w } = self.data.rotation;
        [x, y, z, w]
    }

    pub fn angular_velocity(&self) -> [f32; 3] {
        let sys::rs2_vector { x, y, z } = self.data.angular_velocity;
        [x, y, z]
    }

    pub fn angular_acceleration(&self) -> [f32; 3] {
        let sys::rs2_vector { x, y, z } = self.data.angular_acceleration;
        [x, y, z]
    }

    pub fn tracker_confidence(&self) -> Confidence {
        match self.data.tracker_confidence {
            0x0 => Confidence::Failed,
            0x1 => Confidence::Low,
            0x2 => Confidence::Medium,
            0x3 => Confidence::High,
            _ => panic!("Unknown confidence, please report a bug!"),
        }
    }

    pub fn mapper_confidence(&self) -> Confidence {
        match self.data.tracker_confidence {
            0x0 => Confidence::Failed,
            0x1 => Confidence::Low,
            0x2 => Confidence::Medium,
            0x3 => Confidence::High,
            _ => panic!("Unknown confidence, please report a bug!"),
        }
    }
}

impl Drop for PoseFrame {
    fn drop(&mut self) {
        unsafe {
            sys::rs2_release_frame(self.frame_ptr.as_ptr());
        }
    }
}

impl Kind for PoseFrame {
    fn extension() -> sys::rs2_extension {
        sys::rs2_extension_RS2_EXTENSION_POSE_FRAME
    }
}

impl<'a> std::convert::TryFrom<NonNull<sys::rs2_frame>> for PoseFrame {
    type Error = anyhow::Error;

    fn try_from(frame_ptr: NonNull<sys::rs2_frame>) -> Result<Self, Self::Error> {
        unsafe {
            let mut err = ptr::null_mut::<sys::rs2_error>();

            let profile_ptr = sys::rs2_get_frame_stream_profile(frame_ptr.as_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetFrameStreamProfile)?;

            let nonnull_profile_ptr =
                NonNull::new(profile_ptr as *mut sys::rs2_stream_profile).unwrap();
            let profile = stream::Profile::new(nonnull_profile_ptr)?;

            let mut pose_data = MaybeUninit::uninit();
            sys::rs2_pose_frame_get_pose_data(frame_ptr.as_ptr(), pose_data.as_mut_ptr(), &mut err);
            check_rs2_error!(err, FrameConstructionError::CouldNotGetData)?;

            Ok(PoseFrame {
                frame_ptr,
                frame_stream_profile: profile,
                data: pose_data.assume_init(),
            })
        }
    }
}