# Running tests

By default the Rust test runner executes in parallel, trying to run as many tests concurrently as possible. This is a
problem because the underlying librealsense2 API doesn't make any guarantees surrounding behaviour in parallel
environments. In particular, if you try to run pipelines to the same device (or even different devices) on parallel
threads, you may get some contention and random crashing at the libusb / kernel level. Unit tests can typically avoid
this, as full streaming setups would be more in the realm of integration tests (in the `tests/` directory). The
integration tests aim to setup streaming as a normal program might, so it is necessary to run these tests on a single
thread of execution (otherwise, you'll get random crashes and failures, and the signal : noise on that isn't worth the
time to investigate).

## Running unit tests WITH NO DEVICE connected

`cargo test`

## Running integration tests WITH DEVICE(S) connected

`RUST_TEST_THREADS=1 cargo test --all-features`

Here, `--all-features` is just a quick way to enable the `test-single-device` feature (See [Cargo.toml](Cargo.toml)).

Some of the integration tests will be hardware specific, and will do nothing if a device of expected category (e.g.
D400, L500) is not connected. These tests are not run on CI checks, and will have to be run manually.

# Testing

Testing a project that incorporates hardware is going to be difficult by default, because the end-user expectations
hinge on the hardware <-> software interaction, but we cannot always guarantee full availability of every possible
hardware combination.

What's more, we can't even always guarantee that a single device is plugged in. This makes it difficult to unit-test
parts of the API that rely on device-specific behaviour. There are ways to mock around this, but mocking such behaviour
is more often than not going to be uninteresting and unhelpful in evaluating whether or not the hardware and software
are operating as intended.

Lastly, unit-tests specifically become difficult to write because many parts of the underlying librealsense2 API are
interconnected. It is not possible to test a pipeline object without a context, for example, because the underlying
C-API requires that we construct a pipeline from a context. If we mock this out, we're not testing much at all that the
compiler should not already guarantee in safe Rust. Primarily, remember that the most interesting things to test will be
the `unsafe` blocks and ensuring that no resources are leaked (and that the abstractions encourage good Rust-like
programming!).

## Device support

Unfortunately, we do not own the full suite of sensors that RealSense purports to support. In some ways this isn't
possible because RealSense will also detect and configure webcams, and those will each have their own set of supported
interfaces, resolutions, etc. Fortunately, this isn't actually a show-stopper for most of the code. The majority of the
crate is just a thin-wrapper around the semantics of the C-API in librealsense2. Additionally, most of the abstractions
in that API (pipelines, contexts, etc.) are more or less the same for all devices. Some device-specific aspects exist
(you can filter devices by product category, and different sensors support different options); however, the majority of
the code is expected to operate in much the same way.

The main feature that we aim to provide with realsense-rust is better memory safety, and providing a better model of the
underlying process in which you acquire data from librealsense2. To that end, we have spent time to understand the
underlying C and C++ ownership model for every pointer type in librealsense2 as best we can. Where possible, we reflect
this in the crate itself. This is all to say that you should not expect the existing abstractions to leak memory or
violate ownership. If you do find this though, you've found a bug! We encourage you to report it to our [issue
tracker](https://gitlab.com/tangram-vision-oss/realsense-rust/-/issues) to help us fix it for the community at large!

With all that said, we explicitly have tested for the following devices:

- [RealSense D435i](https://www.intelrealsense.com/lidar-camera-l515/)
- [RealSense L515](https://www.intelrealsense.com/lidar-camera-l515/)


# Other things to be aware of

## Linting

All merge requests and all code must be linted and approved by `cargo clippy` before it is merged in. This is enforced
through the CI pipeline. If you want to run what the pipeline does locally, run:

```
cargo clippy --all-targets --all-features -- -D warnings
```

## `rustfmt` requirements

We use `rustfmt` to ensure that our code is always consistent in style. If you are contributing any tests, please do so
as well.
