/*
 * Copyright 2025 Neuraville Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

//! # FEAGI ESP32 Controller Firmware
//!
//! Controller mode: ESP32 acts as an I/O interface, communicating with FEAGI
//! running on a separate device via Serial/WiFi/Bluetooth.

#![no_std]
#![no_main]

use esp_idf_svc::sys;
use core::ffi::{c_char, CStr};

// ESP32-specific imports
use esp_idf_svc::hal::{
    gpio::{Input, Output, PinDriver, AnyIOPin},
    peripherals::Peripherals,
    uart::{config::Config as UartConfig, UartDriver},
    delay::FreeRtos,
    units::Hertz,
};
use heapless::{Vec, String, Fmt};

// Include build-time configuration
include!(concat!(env!("OUT_DIR"), "/config.rs"));

// GPIO pin configuration structure
#[derive(Debug, Clone, Copy)]
pub enum GpioMode {
    Disabled,
    DigitalInput,
    DigitalOutput,
    AnalogInput,
    PwmOutput,
}

#[derive(Debug, Clone, Copy)]
pub struct GpioPinConfig {
    pub pin: u32,
    pub mode: GpioMode,
    pub cortical_mapping: &'static str,
}

// Helper function to parse neuron ID from cortical mapping
// Format: "cortical_area:neuron_id" or just "neuron_id"
fn parse_neuron_id(mapping: &str) -> Option<u32> {
    if let Ok(id) = mapping.parse::<u32>() {
        return Some(id);
    }
    if let Some(idx) = mapping.rfind(':') {
        if let Ok(id) = mapping[(idx + 1)..].parse::<u32>() {
            return Some(id);
        }
    }
    None
}

// Helper function to convert u32 to string
fn u32_to_string<const N: usize>(n: u32, buf: &mut String<N>) {
    buf.clear();
    if n == 0 {
        let _ = buf.push('0');
        return;
    }
    let mut digits: Vec<u8, 16> = Vec::new();
    let mut num = n;
    while num > 0 {
        let _ = digits.push((b'0' + ((num % 10) as u8)));
        num /= 10;
    }
    for d in digits.iter().rev() {
        let _ = buf.push(*d as char);
    }
}

// Helper function to convert u64 to string
fn u64_to_string<const N: usize>(n: u64, buf: &mut String<N>) {
    buf.clear();
    if n == 0 {
        let _ = buf.push('0');
        return;
    }
    let mut digits: Vec<u8, 16> = Vec::new();
    let mut num = n;
    while num > 0 {
        let _ = digits.push((b'0' + ((num % 10) as u8)));
        num /= 10;
    }
    for d in digits.iter().rev() {
        let _ = buf.push(*d as char);
    }
}

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    unsafe {
        sys::esp_rom_printf(b"[FEAGI] Starting ESP32 Controller Firmware\r\n\0".as_ptr() as *const c_char);
        sys::esp_rom_printf(b"[FEAGI] Transport: %s\r\n\0".as_ptr() as *const c_char, TRANSPORT_TYPE.as_ptr() as *const c_char);
    }
    
    sys::link_patches();
    
    // Initialize logging
    unsafe {
        use esp_idf_svc::sys::{esp_log_level_set, esp_log_level_t_ESP_LOG_INFO};
        esp_log_level_set(
            CStr::from_bytes_with_nul_unchecked(b"*\0").as_ptr(),
            esp_log_level_t_ESP_LOG_INFO,
        );
    }
    
    // Get peripherals
    let peripherals = Peripherals::take()
        .map_err(|_| anyhow::anyhow!("Failed to take peripherals"))?;
    
    // Configure status LED (GPIO2 is commonly the on-board LED)
    let mut led = PinDriver::output(peripherals.pins.gpio2)
        .map_err(|e| anyhow::anyhow!("Failed to configure LED: {:?}", e))?;
    
    // Initialize transport based on configuration
    let mut uart: Option<UartDriver<'static>> = None;
    
    match TRANSPORT_TYPE {
        "serial" => {
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] Configuring Serial/UART transport (115200 baud)\r\n\0".as_ptr() as *const c_char);
            }
            
            // Initialize UART0 for serial communication (USB serial on most ESP32 boards)
            // TX=GPIO1, RX=GPIO3 for UART0 (default USB serial)
            let uart_config = UartConfig::default()
                .baudrate(Hertz(115200))
                .data_bits(esp_idf_svc::hal::uart::config::DataBits::DataBits8)
                .parity_none()
                .stop_bits(esp_idf_svc::hal::uart::config::StopBits::STOP1)
                .flow_control_none();
            
            match UartDriver::new(
                peripherals.uart0,
                peripherals.pins.gpio1,
                peripherals.pins.gpio3,
                Option::<AnyIOPin>::None,
                Option::<AnyIOPin>::None,
                &uart_config,
            ) {
                Ok(driver) => {
                    uart = Some(driver);
                    unsafe {
                        sys::esp_rom_printf(b"[FEAGI] Serial/UART transport ready\r\n\0".as_ptr() as *const c_char);
                    }
                }
                Err(_e) => {
                    unsafe {
                        sys::esp_rom_printf(b"[FEAGI] Warning: Failed to initialize UART, continuing with console only\r\n\0".as_ptr() as *const c_char);
                    }
                }
            }
        }
        "wifi" => {
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] WiFi transport not yet implemented\r\n\0".as_ptr() as *const c_char);
            }
            return Err(anyhow::anyhow!("WiFi transport not yet implemented"));
        }
        "bluetooth" => {
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] Bluetooth transport not yet implemented\r\n\0".as_ptr() as *const c_char);
            }
            return Err(anyhow::anyhow!("Bluetooth transport not yet implemented"));
        }
        _ => {
            return Err(anyhow::anyhow!("Unknown transport type: {}", TRANSPORT_TYPE));
        }
    }
    
    unsafe {
        sys::esp_rom_printf(b"[FEAGI] Configuring GPIO pins...\r\n\0".as_ptr() as *const c_char);
    }
    
    // Collect GPIO pin configurations
    let mut digital_input_configs: Vec<(u32, &'static str), 32> = Vec::new();
    let mut digital_output_configs: Vec<(u32, &'static str), 32> = Vec::new();
    let mut analog_input_configs: Vec<(u32, &'static str), 32> = Vec::new();
    let mut pwm_output_configs: Vec<(u32, &'static str), 32> = Vec::new();
    
    for gpio_config in GPIO_CONFIG {
        match gpio_config.mode {
            GpioMode::DigitalInput => {
                let _ = digital_input_configs.push((gpio_config.pin, gpio_config.cortical_mapping));
                unsafe {
                    sys::esp_rom_printf(b"[FEAGI] GPIO %d: Digital Input -> %s\r\n\0".as_ptr() as *const c_char,
                        gpio_config.pin as i32, gpio_config.cortical_mapping.as_ptr() as *const c_char);
                }
            }
            GpioMode::DigitalOutput => {
                let _ = digital_output_configs.push((gpio_config.pin, gpio_config.cortical_mapping));
                unsafe {
                    sys::esp_rom_printf(b"[FEAGI] GPIO %d: Digital Output -> %s\r\n\0".as_ptr() as *const c_char,
                        gpio_config.pin as i32, gpio_config.cortical_mapping.as_ptr() as *const c_char);
                }
            }
            GpioMode::AnalogInput => {
                let _ = analog_input_configs.push((gpio_config.pin, gpio_config.cortical_mapping));
                unsafe {
                    sys::esp_rom_printf(b"[FEAGI] GPIO %d: Analog Input -> %s (ADC support coming soon)\r\n\0".as_ptr() as *const c_char,
                        gpio_config.pin as i32, gpio_config.cortical_mapping.as_ptr() as *const c_char);
                }
            }
            GpioMode::PwmOutput => {
                let _ = pwm_output_configs.push((gpio_config.pin, gpio_config.cortical_mapping));
                unsafe {
                    sys::esp_rom_printf(b"[FEAGI] GPIO %d: PWM Output -> %s (PWM support coming soon)\r\n\0".as_ptr() as *const c_char,
                        gpio_config.pin as i32, gpio_config.cortical_mapping.as_ptr() as *const c_char);
                }
            }
            GpioMode::Disabled => {}
        }
    }
    
    unsafe {
        sys::esp_rom_printf(b"[FEAGI] GPIO configuration complete\r\n\0".as_ptr() as *const c_char);
        sys::esp_rom_printf(b"[FEAGI] Initialization complete\r\n\0".as_ptr() as *const c_char);
        sys::esp_rom_printf(b"[FEAGI] Burst frequency: %d Hz\r\n\0".as_ptr() as *const c_char, BURST_FREQUENCY_HZ as i32);
    }
    
    // Main loop: I/O communication with FEAGI
    let sampling_period_ms = 1000 / BURST_FREQUENCY_HZ;
    let mut frame_number: u64 = 0;
    let mut rx_buffer: [u8; 512] = [0; 512];
    let mut rx_accumulator: Vec<u8, 512> = Vec::new();
    
    // Helper function to get pin from peripherals by number
    // This is a simplified version - in production, use a pin mapping function
    macro_rules! get_pin {
        ($pin_num:expr, $pins:expr) => {
            match $pin_num {
                0 => Some($pins.gpio0),
                2 => Some($pins.gpio2),
                4 => Some($pins.gpio4),
                5 => Some($pins.gpio5),
                12 => Some($pins.gpio12),
                13 => Some($pins.gpio13),
                14 => Some($pins.gpio14),
                15 => Some($pins.gpio15),
                16 => Some($pins.gpio16),
                17 => Some($pins.gpio17),
                18 => Some($pins.gpio18),
                19 => Some($pins.gpio19),
                21 => Some($pins.gpio21),
                22 => Some($pins.gpio22),
                23 => Some($pins.gpio23),
                25 => Some($pins.gpio25),
                26 => Some($pins.gpio26),
                27 => Some($pins.gpio27),
                32 => Some($pins.gpio32),
                33 => Some($pins.gpio33),
                _ => None,
            }
        };
    }
    
    loop {
        // Blink LED to show activity
        led.set_high().ok();
        FreeRtos::delay_ms(10);
        led.set_low().ok();
        
        // 1. Read sensor inputs (GPIO)
        let mut sensory_data: Vec<(u32, f32), 64> = Vec::new();  // (neuron_id, potential)
        
        // Read digital inputs dynamically
        for (pin_num, mapping) in digital_input_configs.iter() {
            if let Some(pin) = get_pin!(*pin_num, peripherals.pins) {
                // Create temporary driver to read pin state
                if let Ok(mut driver) = PinDriver::input(pin) {
                    if let Ok(level) = driver.get_level() {
                        let potential = if level == esp_idf_svc::hal::gpio::Level::High { 1.0 } else { 0.0 };
                        if let Some(neuron_id) = parse_neuron_id(mapping) {
                            let _ = sensory_data.push((neuron_id, potential));
                        }
                    }
                    // Driver goes out of scope here, pin is released
                }
            }
        }
        
        // TODO: Read analog inputs and add to sensory_data (ADC implementation)
        
        // 2. Format and send sensory data to FEAGI via Serial
        if !sensory_data.is_empty() && uart.is_some() {
            // Build JSON message: {"np":[[id,pot],...],"id":"esp32","f":N}
            let mut json: String<512> = String::from("{\"np\":[");
            
            for (i, (id, pot)) in sensory_data.iter().enumerate() {
                if i > 0 {
                    let _ = json.push_str(",");
                }
                
                // Convert neuron ID to string
                let mut id_str: String<16> = String::new();
                u32_to_string(*id, &mut id_str);
                
                // Convert potential to string (binary for now: 0 or 1)
                let pot_int = if *pot > 0.5 { 1 } else { 0 };
                let mut pot_str: String<16> = String::new();
                u32_to_string(pot_int as u32, &mut pot_str);
                
                let _ = json.push_str("[");
                let _ = json.push_str(id_str.as_str());
                let _ = json.push_str(",");
                let _ = json.push_str(pot_str.as_str());
                let _ = json.push_str("]");
            }
            
            let _ = json.push_str("],\"id\":\"esp32\",\"f\":");
            let mut frame_str: String<16> = String::new();
            u64_to_string(frame_number, &mut frame_str);
            let _ = json.push_str(frame_str.as_str());
            let _ = json.push_str("}\n");
            
            // Send over UART
            if let Some(ref mut u) = uart {
                if let Err(_e) = u.write(json.as_bytes()) {
                    unsafe {
                        sys::esp_rom_printf(b"[FEAGI] Failed to send sensory data\r\n\0".as_ptr() as *const c_char);
                    }
                }
            }
        }
        
        // 3. Receive motor commands from FEAGI via Serial (non-blocking)
        if let Some(ref mut u) = uart {
            match u.read(&mut rx_buffer, 10) {  // 10ms timeout
                Ok(count) if count > 0 => {
                    // Accumulate received data
                    for i in 0..count {
                        if let Err(_) = rx_accumulator.push(rx_buffer[i]) {
                            // Buffer full, process what we have
                            break;
                        }
                    }
                    
                    // Check if we have a complete JSON message (ends with \n)
                    if let Some(newline_idx) = rx_accumulator.iter().position(|&b| b == b'\n') {
                        // Extract message (build string manually for heapless)
                        let mut message_str: String<512> = String::new();
                        for &byte in rx_accumulator.iter().take(newline_idx) {
                            if byte.is_ascii() {
                                let _ = message_str.push(byte as char);
                            }
                        }
                        rx_accumulator.clear();
                        
                        // Parse JSON motor command (simplified parsing)
                        // Format: {"mc":[[neuron_id,value],...]} or {"motor_commands":[...]}
                        // Simple parsing: look for neuron_id and value pairs
                        // TODO: Use proper JSON parser (serde-json-core)
                        
                        // For now, implement simple pattern matching
                        // Look for patterns like "neuron_id":N or "value":V
                        let mut neuron_id: Option<u32> = None;
                        let mut value: Option<f32> = None;
                        
                        // Try to extract neuron_id and value from JSON
                        // This is a very simple parser - in production use serde-json-core
                        // Split by non-alphanumeric characters
                        let mut words: Vec<&str, 64> = Vec::new();
                        let mut word_start = 0;
                        let message_bytes = message_str.as_bytes();
                        for (i, &byte) in message_bytes.iter().enumerate() {
                            let c = byte as char;
                            if !c.is_alphanumeric() && c != '.' && c != '-' {
                                if i > word_start {
                                    if let Ok(word) = core::str::from_utf8(&message_bytes[word_start..i]) {
                                        if !word.is_empty() {
                                            let _ = words.push(word);
                                        }
                                    }
                                }
                                word_start = i + 1;
                            }
                        }
                        if word_start < message_bytes.len() {
                            if let Ok(word) = core::str::from_utf8(&message_bytes[word_start..]) {
                                if !word.is_empty() {
                                    let _ = words.push(word);
                                }
                            }
                        }
                            
                            for i in 0..words.len().saturating_sub(1) {
                                if words[i] == "neuron_id" || words[i] == "id" {
                                    if let Some(id_str) = words.get(i + 1) {
                                        if let Ok(id) = id_str.parse::<u32>() {
                                            neuron_id = Some(id);
                                        }
                                    }
                                }
                                if words[i] == "value" || words[i] == "v" {
                                    if let Some(val_str) = words.get(i + 1) {
                                        if let Ok(val) = val_str.parse::<f32>() {
                                            value = Some(val);
                                        }
                                    }
                                }
                            }
                            
                            // Apply motor command to GPIO outputs
                            if let (Some(nid), Some(val)) = (neuron_id, value) {
                                // Find GPIO output with matching neuron ID
                                for (pin_num, mapping) in digital_output_configs.iter() {
                                    if let Some(neuron_id_from_map) = parse_neuron_id(mapping) {
                                        if neuron_id_from_map == nid {
                                            if let Some(pin) = get_pin!(*pin_num, peripherals.pins) {
                                                if let Ok(mut driver) = PinDriver::output(pin) {
                                                    if val > 0.5 {
                                                        let _ = driver.set_high();
                                                    } else {
                                                        let _ = driver.set_low();
                                                    }
                                                    // Driver goes out of scope, pin released
                                                }
                                            }
                                        }
                                    }
                                }
                                
                                unsafe {
                                    sys::esp_rom_printf(b"[FEAGI] Motor: neuron %d -> value %.2f\r\n\0".as_ptr() as *const c_char,
                                        nid as i32, val as f64);
                                }
                            }
                        }
                    }
                }
                Ok(_) => {
                    // No data available, continue
                }
                Err(_) => {
                    // Read error, continue
                }
            }
        }
        
        // 4. Write motor outputs (GPIO)
        // This is handled in the receive section above
        
        frame_number = frame_number.wrapping_add(1);
        
        // Wait for next sampling period
        let elapsed = 10; // LED blink time + processing time estimate
        if sampling_period_ms > elapsed {
            FreeRtos::delay_ms(sampling_period_ms - elapsed);
        }
    }
}
