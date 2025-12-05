/*
 * Copyright 2025 Neuraville Inc.
 */

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Tell cargo to rerun this script if config.json changes
    println!("cargo:rerun-if-changed=config.json");
    
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let config_path = PathBuf::from(&manifest_dir).join("config.json");
    
    // Read configuration
    let config = if config_path.exists() {
        let config_str = fs::read_to_string(&config_path)
            .expect("Failed to read config.json");
        serde_json::from_str::<serde_json::Value>(&config_str)
            .expect("Failed to parse config.json")
    } else {
        // Default config if file doesn't exist (for development)
        serde_json::json!({
            "mode": "standalone",
            "model": "esp32-devkit-v1",
            "burst_frequency": 100,
            "gpio": []
        })
    };
    
    let out_dir = env::var("OUT_DIR").unwrap();
    let config_rs = PathBuf::from(&out_dir).join("config.rs");
    
    // Extract configuration values
    let burst_frequency = config.get("burst_frequency")
        .and_then(|v| v.as_u64())
        .unwrap_or(100);
    
    let model = config.get("model")
        .and_then(|v| v.as_str())
        .unwrap_or("esp32-devkit-v1");
    
    // Generate GPIO configuration
    let gpio_config = config.get("gpio")
        .and_then(|v| v.as_array())
        .unwrap_or(&vec![]);
    
    // Generate Rust code for config
    let mut config_code = String::new();
    config_code.push_str("// Auto-generated configuration\n");
    config_code.push_str(&format!("pub const BURST_FREQUENCY_HZ: u32 = {};\n", burst_frequency));
    config_code.push_str(&format!("pub const MODEL: &str = \"{}\";\n", model));
    
    // Generate GPIO pin configuration
    config_code.push_str("\npub const GPIO_CONFIG: &[GpioPinConfig] = &[\n");
    for gpio in gpio_config {
        if let Some(pin) = gpio.get("pin").and_then(|v| v.as_u64()) {
            if let Some(mode) = gpio.get("mode").and_then(|v| v.as_str()) {
                if mode != "disabled" {
                    let cortical_mapping = gpio.get("cortical_mapping")
                        .and_then(|v| v.as_str())
                        .unwrap_or("");
                    
                    let mode_const = match mode {
                        "digital_input" => "GpioMode::DigitalInput",
                        "digital_output" => "GpioMode::DigitalOutput",
                        "analog_input" => "GpioMode::AnalogInput",
                        "pwm_output" => "GpioMode::PwmOutput",
                        _ => "GpioMode::Disabled",
                    };
                    
                    config_code.push_str(&format!(
                        "    GpioPinConfig {{ pin: {}, mode: {}, cortical_mapping: \"{}\" }},\n",
                        pin, mode_const, cortical_mapping
                    ));
                }
            }
        }
    }
    config_code.push_str("];\n");
    
    // Write generated config
    fs::write(&config_rs, config_code)
        .expect("Failed to write config.rs");
}

