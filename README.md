# RealSense bindings for Rust

The project provides high-level bindings (crate `realsense_rust`) to librealsense2 library as well as low-level FFI
(crate `realsense_sys`) interface.

This project is hosted on both [Github](https://github.com/Tangram-Vision/realsense-rust) and
[Gitlab](https://gitlab.com/tangram-vision-oss/realsense-rust/). While we're happy to receive pull / merge requests on
either platform, we focus most of our work on Gitlab, so please submit an issue there if you've found something we need
to improve, or if you have a question regarding how the software works!

## Use this crate in your project

Make sure **librealsense 2.41.0** is installed on your system. You may visit [RealSense official
repository](https://github.com/IntelRealSense/librealsense).

Add this crate to your `Cargo.toml`:

```toml
[dependencies]
realsense-rust = "0.5"
```

If you're using older librealsense for reasons. You may enable
`buildtime-bindgen` to re-generate bindings and good luck.

```toml
[dependencies]
realsense-rust = { version = "0.5", features = ["buildtime-bindgen"] }
```

## Cargo Features

- **with-nalgebra** (default): Enable [nalgebra](https://github.com/rustsim/nalgebra) support.
- **buildtime-bindgen**: Generate Rust bindings during build time.

## Get Started

You can start by using `InactivePipeline`. This is the minimal example to capture color and depth images.

```rust
use anyhow::Result;
use realsense_rust::{
    config::Config,
    kind::{Rs2Format, Rs2StreamKind},
    pipeline::{
        InactivePipeline,
        ActivePipeline
    },
};
use std::convert::TryFrom;

fn main() -> Result<()> {
    let context = Context::new()?;
    let pipeline = InactivePipeline::try_from(context)?;

    let config = Config::new()?;
    config
        .enable_stream(Rs2StreamKind::Depth, 0, 640, 0, Rs2Format::Z16, 30)?
        .enable_stream(Rs2StreamKind::Color, 0, 640, 0, Rs2Format::Rgb8, 30)?;

    let mut pipeline = pipeline.start(&config)?;

    let frames = pipeline.wait(None)?;
    let video_frames = frames.frames_of_extension::<VideoFrame>();
    let depth_frames = frames.frames_of_extension::<DepthFrame>();

    for f in video_frames {
        // process video / color frames
    }

    for d in depth_frames {
        // process depth frames
    }

    Ok(())
}
```

## Examples

To capture image with your RealSense device,

```sh
cargo run --release --example capture_images
```

More examples can be found in [examples](examples) directory.

## Contributing to this project

First, check out our [contributing guidelines](CONTRIBUTING.md). After that, make sure that you read through the
documentation in [lib.rs](src/lib.rs) as well as any of the modules you might be interested in contributing to! If you
find documentation missing, this is considered a bug, so please submit a bug report!

### Work with realsense-sys low-level API

The realsense-sys crate provides C bindings generated from librealsense headers. The reference can be found on RealSense
official [documentation](https://github.com/IntelRealSense/librealsense/tree/master/doc).

Import realsense-sys to your `Cargo.toml`.

```toml
[dependencies]
realsense-sys = "0.3"
```
and you can call low level C functions.

### Generate documents from source code

The API changes may not be found on docs.rs. To generate document from the most
recent commit,

```sh
cargo doc --open
```

## License

Apache 2.0. See [LICENSE](LICENSE) file.
