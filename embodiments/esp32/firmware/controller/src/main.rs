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
    gpio::PinDriver,
    peripherals::Peripherals,
    uart::{config::Config as UartConfig, UartDriver},
    delay::FreeRtos,
};

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
    match TRANSPORT_TYPE {
        "serial" => {
            // Configure UART for serial communication
            // TODO: Initialize UART driver
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] Initialized Serial/UART transport\r\n\0".as_ptr() as *const c_char);
            }
        }
        "wifi" => {
            // TODO: Initialize WiFi and TCP connection
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] WiFi transport not yet implemented\r\n\0".as_ptr() as *const c_char);
            }
        }
        "bluetooth" => {
            // TODO: Initialize Bluetooth
            unsafe {
                sys::esp_rom_printf(b"[FEAGI] Bluetooth transport not yet implemented\r\n\0".as_ptr() as *const c_char);
            }
        }
        _ => {
            return Err(anyhow::anyhow!("Unknown transport type: {}", TRANSPORT_TYPE));
        }
    }
    
    // Initialize GPIO pins from configuration
    // TODO: Configure GPIO pins based on GPIO_CONFIG
    
    unsafe {
        sys::esp_rom_printf(b"[FEAGI] Initialization complete\r\n\0".as_ptr() as *const c_char);
        sys::esp_rom_printf(b"[FEAGI] Burst frequency: %d Hz\r\n\0".as_ptr() as *const c_char, BURST_FREQUENCY_HZ as i32);
    }
    
    // Main loop: I/O communication with FEAGI
    let sampling_period_ms = 1000 / BURST_FREQUENCY_HZ;
    
    loop {
        // Blink LED to show activity
        led.set_high().ok();
        FreeRtos::delay_ms(50);
        led.set_low().ok();
        
        // TODO: Controller mode main loop
        // 1. Read sensor inputs (GPIO)
        // 2. Send sensor data to FEAGI via transport
        // 3. Receive motor commands from FEAGI via transport
        // 4. Write motor outputs (GPIO)
        
        // Wait for next sampling period
        FreeRtos::delay_ms(sampling_period_ms - 50);
    }
}

