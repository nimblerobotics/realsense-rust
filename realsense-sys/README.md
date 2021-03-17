# realsense-sys

Generate and use RealSense C library bindings as a Rust crate. This crate is used as a base layer in the more
user-friendly [realsense-rust](https://gitlab.com/tangram-vision-oss/realsense-rust) crate; we recommend use of
realsense-rust if possible in order to better maintain Rust memory safety.

Compatible with RealSense SDK v2.0 and up.

## Usage

This crate finds and links the RealSense SDK. Though one can use the generated bindings directly, this crate is meant as
a base layer for [realsense-rust](https://gitlab.com/tangram-vision-oss/realsense-rust). 

To use this crate, add this line in your `Cargo.toml`.

```toml
realsense-sys = "0.2"
```

# OS Installation Notes

## Linux

No action needed. 

If the realsense2 SDK is installed, pkg-config will detect the [realsense2.pc](./realsense2.pc) config file automatically. This will load
the necessary headers and libraries. 

## Windows 

**NOTE**: The current bindings are formatted for Linux. Users must run with the `buildtime-bindgen` feature active to
reformat the bindings for Windows platforms. 

This installation process assumes that the RealSense SDK was installed through the .exe wizard downloadable from [the
librealsense asset page](https://github.com/IntelRealSense/librealsense/releases/tag/v2.42.0). This process will install
the SDK in `C:/Program Files (x86)/Intel RealSense SDK 2.0`. If your installation is in another place, modify the
`prefix` line in [realsense2.pc](./realsense2.pc) to the right path. 

### Install Pkg-config and Clang

Install pkg-config via Chocolatey: 

1. https://chocolatey.org/install (if not already on the system)
2. `choco install pkgconfiglite`
3. `choco install llvm` for bindgen (if not already installed)

### Guide Pkg-config to realsense2.pc

Set the pkg-config path in Powershell to the realsense-sys directory. One can do this in two ways: 

**First Option: Modify pkg-config's environment variables**

To do this, run 

`$Env:PKG_CONFIG_PATH="C:\Users\< path_to_repo >\realsense-rust\realsense-sys\"`

This will help pkg-config find the [realsense2.pc](./realsense2.pc) file located in this directory. This file tells pkg-config where to
locate the headers and libraries necessary for RealSense operation. The Windows wizard does not provide this file, so we
provide it ourselves. 

It's a good idea to set the `PKG_CONFIG_PATH` Environment Variable globally as well via the System Properties. *BUT
NOTE*: Environment Variables set through the Windows System Properties will not apply until the host machine is power
cycled. Yep. That's a thing. 

**Second Option: Add [realsense2.pc](./realsense2.pc) to pkg-config's search directory**

Run the following command...

```pkg-config --variable pc_path pkg-config```

...to identify the directory (or directories) that pkg-config uses to find *.pc files. Copy [realsense2.pc](./realsense2.pc) to that
directory. Boom, done. 