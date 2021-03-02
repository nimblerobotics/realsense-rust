# RealSense Bindings for Rust

The project provides high-level bindings (crate `realsense_rust`) to librealsense2 library as well as low-level FFI
(crate `realsense_sys`) interface.

**Current librealsense version: 2.41.0**

This project is hosted on both [Github](https://github.com/Tangram-Vision/realsense-rust) and
[Gitlab](https://gitlab.com/tangram-vision-oss/realsense-rust/). While we're happy to receive pull / merge requests on
either platform, we focus most of our work on Gitlab, so please submit an issue there if you've found something we need
to improve or have a question regarding how things work.

## Hardware Considerations

- **USB Current Draw**: Many RealSense devices draw more current than a standard USB cable can provide. For example,
  standard USB can run 0.9 amps, while the RealSense 435i draws 2 amps. Using a USB cable that doesn't have the right
  current capability will interfere with the USB connection on the host, and the device will seem to disconnect. A
  device power cycle doesn't always remedy this, either. In many cases, the host USB hub itself will need a reset. Make
  sure any USB cables used are able to draw at least 2 amps. Read more on the issue
  [here](https://support.intelrealsense.com/hc/en-us/community/posts/360033595714-D435-USB-connection-issues).

## API Use

Make sure librealsense 2.41.0 is installed on your system. Visit the [RealSense official
repository](https://github.com/IntelRealSense/librealsense) to download and install this on the host machine.

Once that's done, add this crate to your project's `Cargo.toml`:

```toml
[dependencies]
realsense-rust = "0.5"
```

...and you should be good to go!

**Backwards compatibility**: If you're using an older librealsense version, you may enable `buildtime-bindgen` to
re-generate the bindings. We make no claims of backwards compatibility; good luck.

```toml
[dependencies]
realsense-rust = { version = "0.5", features = ["buildtime-bindgen"] }
```

## Cargo Features

- **with-nalgebra** (default): Enable [nalgebra](https://github.com/rustsim/nalgebra) support.
- **buildtime-bindgen**: Generate Rust bindings during build time.

## Getting started + Examples

Check out the examples folder for minimal configurations that fit your device. We have included a README.md there that
explains the functionality that one can get from this API. For more explanation, see the crate documentation.

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

The API changes may not be found on docs.rs. To generate document from the most recent commit,

```sh
cargo doc --open
```

## License

Apache 2.0. See [LICENSE](LICENSE) file.
