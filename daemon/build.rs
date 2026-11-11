use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::var_os("CARGO_FEATURE_HARDWARE").is_some() {
        build_ad9361()?;
    }
    build_proto()?;
    Ok(())
}

/// Compiles the control service with protox, so the build needs no `protoc`.
fn build_proto() -> Result<(), Box<dyn std::error::Error>> {
    const PROTO_ROOT: &str = "../proto";
    const PROTO: &str = "plutosky/v1/control.proto";
    let descriptors = protox::compile([PROTO], [PROTO_ROOT])?;
    tonic_prost_build::configure()
        .build_client(true)
        .build_server(true)
        .compile_fds(descriptors)?;
    println!("cargo:rerun-if-changed={PROTO_ROOT}/{PROTO}");
    Ok(())
}

fn build_ad9361() -> Result<(), Box<dyn std::error::Error>> {
    let vendor = PathBuf::from("vendor/ad9361-noos");
    let mut build = cc::Build::new();
    build
        .include(&vendor)
        .warnings(false)
        .flag_if_supported("-std=gnu11")
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-unused-variable")
        .flag_if_supported("-Wno-sign-compare")
        .flag_if_supported("-Wno-macro-redefined")
        // UG-570 Table 1 runs the TX quadrature calibration at initialization
        // and on LO moves above 100 MHz only. Each run radiates its test tone
        // even at maximum TX attenuation, so it must not repeat on every rate
        // or bandwidth change.
        .define("AD9361_TX_QUAD_CAL_ON_LO_CHANGE_ONLY", None);
    for src in ["ad9361.c", "ad9361_api.c", "ad9361_conv.c", "util.c"] {
        build.file(vendor.join(src));
    }
    build.file("csrc/ad9361_shim.c");
    build.compile("ad9361noos");
    println!("cargo:rerun-if-changed=vendor/ad9361-noos");
    println!("cargo:rerun-if-changed=csrc/ad9361_shim.c");
    Ok(())
}
