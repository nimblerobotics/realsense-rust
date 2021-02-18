//! Color scheme choices used by colorizer processing blocks.

use num_derive::{FromPrimitive, ToPrimitive};

/// A type describing the various color scheme choices for colorizer processing blocks.
///
/// This name of this type is not preceded with `Rs2` because this does not map to a librealsense2
/// type from the low-level `realsense-sys` crate. Instead, this is just a selection of better
/// names when setting the `Rs2Option::ColorScheme` option in a processing block that colorizes
/// depth output.
///
/// This enum, much like many others in the `kind` module maps integers of some form to the enum
/// values, and inherits both [`FromPrimitive`](num_traits::FromPrimitive) and
/// [`ToPrimitive`](num_traits::ToPrimitive) from the `num_traits` crate. This is because we want
/// to be able to take advantage of the [`to_f32()`](num_traits::ToPrimitive::to_f32()) function in
/// the low-level API, but use actual color scheme names at a higher level.
///
#[repr(usize)]
#[derive(FromPrimitive, ToPrimitive, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ColorScheme {
    Jet = 0,
    Classic = 1,
    WhiteToBlack = 2,
    BlackToWhite = 3,
    Bio = 4,
    Cold = 5,
    Warm = 6,
    Quantized = 7,
    Pattern = 8,
    Hue = 9,
}
