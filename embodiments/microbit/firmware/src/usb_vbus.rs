//! Simple VBUS detection for micro:bit USB
//!
//! micro:bit v2 USB is always powered when cable is connected,
//! so we implement a simple always-on VBUS detector.

#![cfg(feature = "transport-usb")]

use embassy_nrf::usb::vbus_detect::VbusDetect;

/// Simple VBUS detector that always reports connected
pub struct AlwaysOnVbus;

impl AlwaysOnVbus {
    pub const fn new() -> Self {
        Self
    }
}

impl VbusDetect for AlwaysOnVbus {
    fn is_usb_detected(&self) -> bool {
        true // USB cable provides power, so if we're running, USB is connected
    }

    async fn wait_power_ready(&mut self) -> Result<(), ()> {
        // USB is always ready on micro:bit (if we're running, USB powers us)
        // So this just returns immediately
        Ok(())
    }
}

// Also implement for references, as embassy-nrf Driver may use &VbusDetect or &mut VbusDetect
impl VbusDetect for &AlwaysOnVbus {
    fn is_usb_detected(&self) -> bool {
        true
    }

    async fn wait_power_ready(&mut self) -> Result<(), ()> {
        // Always ready
        Ok(())
    }
}

impl VbusDetect for &mut AlwaysOnVbus {
    fn is_usb_detected(&self) -> bool {
        true
    }

    async fn wait_power_ready(&mut self) -> Result<(), ()> {
        // Always ready
        Ok(())
    }
}

