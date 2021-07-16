fn main() {
    if cfg!(feature = "docs-only") {
        return;
    }

    // Probe libary
    let library = pkg_config::probe_library("realsense2")
        .expect("pkg-config failed to find realsense2 package");
    let major_version = library
        .version
        .find('.')
        .map(|i| &library.version[..i])
        .expect("failed to determine librealsense major version");

    if major_version != "2" {
        panic!(
            "librealsense2 version {} is not supported, expected major version 2",
            library.version
        )
    }

    // generate bindings
    #[cfg(feature = "buildtime-bindgen")]
    {
        let cargo_manifest_dir = std::env::current_dir().unwrap();

        // The function below will leave us with the directory <SDKHome>/include/librealsense2/
        let include_dir = library
            .include_paths
            .iter()
            .filter_map(|path| {
                let dir = std::path::Path::new(path).join("librealsense2");
                if dir.is_dir() {
                    Some(dir)
                } else {
                    None
                }
            })
            .next()
            .expect("fail find librealsense2 include directory");

        // pop the last item off of the include_dir to get `/include`, which we'll need to build rsutil_delegate.h
        let mut top_include = include_dir.clone();
        top_include.pop();
        let bindings = bindgen::Builder::default()
            .clang_arg("-fno-inline-functions")
            // Include... `<SDKHome>/include`
            // Again, this is just so that we can compile rsutil_delegate.h
            .clang_arg(String::from("-I") + top_include.to_str().unwrap())
            .header(include_dir.join("rs.h").to_str().unwrap())
            .header(
                include_dir
                    .join("h")
                    .join("rs_pipeline.h")
                    .to_str()
                    .unwrap(),
            )
            .header(
                include_dir
                    .join("h")
                    .join("rs_advanced_mode_command.h")
                    .to_str()
                    .unwrap(),
            )
            .header(include_dir.join("h").join("rs_config.h").to_str().unwrap())
            .whitelist_var("RS2_.*")
            .whitelist_type("rs2_.*")
            .whitelist_function("rs2_.*")
            .whitelist_function("_rs2_.*")
            .generate()
            .expect("Unable to generate bindings");

        // Write the bindings to file
        let bindings_dir = cargo_manifest_dir.join("bindings");
        let bindings_file = bindings_dir.join("bindings.rs");

        if let Err(e) = std::fs::create_dir_all(&bindings_dir) {
            panic!(
                "failed to create directory {}: {}",
                bindings_dir.display(),
                e
            );
        }
        bindings
            .write_to_file(bindings_file)
            .expect("Couldn't write bindings!");
    }

    // link the libraries specified by pkg-config.
    for dir in &library.link_paths {
        println!("cargo:rustc-link-search=native={}", dir.to_str().unwrap());
    }
    for lib in &library.libs {
        println!("cargo:rustc-link-lib={}", lib);
    }

    #[cfg(target_os = "windows")]
    if let Some(dll_loc) = &library.defines["DLL_FOLDER"] {
        // Move DLL from DLL_FOLDER location to the deps folder for this executable.
        //
        // The current_exe() function returns the directory:
        //
        // `<topLevel>/target/<buildType>/build/realsense-sys<hash>/executable.exe`
        //
        // ...however, the proper place for the DLL is actually in
        //
        // `<topLevel>/target/<buildType>/deps`
        //
        // So, pop three times, add two strings, and we're good to go with the right location.
        // Is it pretty? No. But it'll work for now.
        let mut exe_path = std::env::current_exe().unwrap();
        exe_path.pop();
        exe_path.pop();
        exe_path.pop();
        exe_path.push("deps");
        exe_path.push("realsense2.dll");
        let dll_dest = exe_path.to_str().unwrap();
        let mut dll_src = std::path::PathBuf::from(dll_loc);
        dll_src.push("realsense2.dll");
        match std::fs::copy(dll_src.clone(), dll_dest) {
            Ok(_) => println!("DLL successfully copied to deps folder."),
            Err(e) => panic!("{}; attempting from source {:#?}", e, dll_src),
        }
    }
}
