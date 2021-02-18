//! Types for representing data related to streams
//!
//! In librealsense2, a stream is an abstraction around a "sensor stream." In most applications,
//! every sensor in the API will probably only have one stream that you actively care about (but
//! not always!). That stream is tied to both the sensor it is configured on as well as the frames
//! that are produced from the stream. The stream contains metadata such as the stream kind, the
//! format of the incoming data, etc.
//!
//! There are two ways you can get streams in the system: from a stream profile list returned from
//! the `Sensor`, or the frame stream from a frame type. In both cases the stream profile has an
//! explicit lifetime that depends on the object you obtained it from. This is because stream
//! profile ownership is not the same depending on how you obtained it.
//!
//! # Things to watch out for
//!
//! Extrinsics and intrinsics between different streams are obtained via the stream profile.
//! However, your stream profile may not be valid if you get it from the frame. This is because
//! stream profiles for the frame at a low level are owned by librealsense2. Calls to
//! `rs2_get_frame_stream_profile` return a `*const rs2_stream_profile`, which is obtained by
//! a call to C++ `shared_ptr::get()` internally to librealsense2. Since this is owned and managed
//! by librealsense2, and calls to `shared_ptr::get()` aren't tracked by the internal ref-count on
//! the `shared_ptr`, this means that the pointer could be invalidated at any time. In practice,
//! this means that if you disconnect your device mid-way through streaming (i.e. yank the cable
//! out), you could be holding onto a `StreamProfile` with an invalid internal pointer.
//!
//! We attempt to cache some aspects of the stream profile ahead of time, but in some cases this is
//! not feasible (e.g. extrinsics). This is why some interfaces return `Result`s and others do not.
//! The interfaces that do not return `Result` types are cached or otherwise safe-to-assume as
//! such. The ones that do return `Result` types are will check the pointer at runtime and may
//! return an error if it is no longer valid.
//!
//! We recommend that you get extrinsic or intrinsic information ahead of time when the device is
//! connected (via `Device::sensor()` and then `Sensor::stream_profiles()`), rather than relying on
//! the stream profile obtained via the frame types. The streams will have the same unique
//! identifier if they correspond to the same stream.
//!
//! See [the `StreamProfile` type](crate::stream::StreamProfile) for more information.
//!

mod profile;

pub use profile::{DataError, StreamConstructionError, StreamProfile};
