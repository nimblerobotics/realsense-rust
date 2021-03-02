# Examples

To run any examples, use cargo: `$cargo run --example <demo name>`

## For all devices

- [enumerate\_devices.rs](enumerate_devices.rs): Enumerates all devices connected to the host that are compatible with
  the Intel RealSense library.

## 435i

- [demo\_435i.rs](demo_435i.rs): Runs a subset of streams on the 435i. Prints out the center pixel value of the depth
  image and the current gyro value.

## L515

- [demo\_l515.rs](demo_l515.rs): Runs a subset of streams on the L515. Prints out the center pixel value of the depth
  image.
