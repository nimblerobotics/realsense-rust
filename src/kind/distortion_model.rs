//! Enumeration describing the possible kinds of distortions present in a stream profile.
//!
//! TODO

use num_derive::{FromPrimitive, ToPrimitive};
use realsense_sys as sys;

#[repr(u32)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Rs2DistortionModel {
    /// Rectilinear images. No distortion compensation required.    
    None = sys::rs2_distortion_RS2_DISTORTION_NONE,
    /// Unmodified Brown-Conrady distortion model
    BrownConrady = sys::rs2_distortion_RS2_DISTORTION_BROWN_CONRADY,
    /// Equivalent to Brown-Conrady distortion, except that tangential distortion is applied to radially distorted points
    BrownConradyModified = sys::rs2_distortion_RS2_DISTORTION_MODIFIED_BROWN_CONRADY,
    /// Equivalent to Brown-Conrady distortion, except undistorts image instead of distorting it
    BrownConradyInverse = sys::rs2_distortion_RS2_DISTORTION_INVERSE_BROWN_CONRADY,
    /// F-Theta fish-eye distortion model
    FThetaFisheye = sys::rs2_distortion_RS2_DISTORTION_FTHETA,
    /// Four parameter Kannala Brandt distortion model
    KannalaBrandt = sys::rs2_distortion_RS2_DISTORTION_KANNALA_BRANDT4,
    // Number of enumeration values. Not included.
    //
    // Count = sys::rs2_distortion_RS2_DISTORTION_COUNT
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_traits::FromPrimitive;

    #[test]
    fn all_variants_exist() {
        for i in 0..sys::rs2_distortion_RS2_DISTORTION_COUNT {
            assert!(
                Rs2DistortionModel::from_u32(i).is_some(),
                "DistortionModel variant for ordinal {} does not exist.",
                i,
            );
        }
    }
}