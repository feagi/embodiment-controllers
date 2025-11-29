use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Get the build profile
    let _target = env::var("TARGET").unwrap();
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=memory.x");

    // Generate device-specific configuration
    let config_path = out_dir.join("config.rs");
    let mut config_file = File::create(&config_path).unwrap();

    writeln!(config_file, "// Auto-generated device configuration").unwrap();
    writeln!(config_file, "").unwrap();
    writeln!(config_file, "pub const DEVICE_VERSION: &str = \"v2\";").unwrap();
    writeln!(config_file, "pub const CHIP_NAME: &str = \"nRF52833\";").unwrap();
    writeln!(config_file, "pub const FLASH_SIZE: u32 = 512 * 1024;").unwrap();
    writeln!(config_file, "pub const RAM_SIZE: u32 = 128 * 1024;").unwrap();
    writeln!(config_file, "pub const CPU_FREQ_MHZ: u32 = 64;").unwrap();

    // Default configuration (can be overridden by config.json at build time)
    writeln!(config_file, "").unwrap();
    writeln!(config_file, "// Default FEAGI configuration").unwrap();
    writeln!(config_file, "pub const BLUETOOTH_NAME: &str = \"FEAGI-microbit\";").unwrap();
    writeln!(config_file, "pub const SAMPLING_RATE_HZ: u32 = 10;").unwrap();
    writeln!(config_file, "").unwrap();
    writeln!(config_file, "// Feature flags").unwrap();
    writeln!(config_file, "pub const SENSOR_ACCEL_ENABLED: bool = true;").unwrap();
    writeln!(config_file, "pub const SENSOR_MAG_ENABLED: bool = true;").unwrap();
    writeln!(config_file, "pub const SENSOR_TEMP_ENABLED: bool = true;").unwrap();
    writeln!(config_file, "pub const SENSOR_BUTTONS_ENABLED: bool = true;").unwrap();
    writeln!(config_file, "pub const OUTPUT_LED_MATRIX_ENABLED: bool = true;").unwrap();

    println!("cargo:rustc-env=CONFIG_RS={}", config_path.display());

    // Link memory.x - tell rustc where to find it
    println!("cargo:rustc-link-search=native={}", env::var("CARGO_MANIFEST_DIR").unwrap());

    // Rebuild if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rerun-if-env-changed=FEAGI_CONFIG");
}


