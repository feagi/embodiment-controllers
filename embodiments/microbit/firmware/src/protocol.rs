//! FEAGI Transport Protocol (no_std, embedded-friendly)
//!
//! This is a minimal copy of feagi-embedded's protocol layer for embedded use.
//! It's transport-agnostic and works with BLE, USB CDC, UART, etc.

#![allow(dead_code)]

use heapless::Vec;

/// FEAGI commands (parsed from binary packets)
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Set a GPIO pin to high or low
    SetGpio { pin: u8, value: bool },
    /// Set PWM duty cycle (0-255) on a pin
    SetPwm { pin: u8, duty: u8 },
    /// Set full LED matrix (5x5 = 25 bytes, brightness 0-255)
    SetLedMatrix { data: [u8; 25] },
    /// Neuron firing coordinates for LED matrix visualization
    NeuronFiring { coordinates: Vec<(u8, u8), 25> },
    /// Request device capabilities JSON
    GetCapabilities,
}

/// FEAGI protocol handler
pub struct FeagiProtocol {
    rx_buffer: Vec<u8, 256>,
    commands: Vec<Command, 8>,
}

impl FeagiProtocol {
    pub fn new() -> Self {
        Self {
            rx_buffer: Vec::new(),
            commands: Vec::new(),
        }
    }
    
    /// Process received data (adds to buffer and parses packets)
    pub fn process_received_data(&mut self, data: &[u8]) {
        // Add to buffer
        for &byte in data {
            let _ = self.rx_buffer.push(byte);
        }
        
        // Try to parse packets
        self.parse_packets();
    }
    
    /// Get next parsed command (if any)
    pub fn receive_command(&mut self) -> Option<Command> {
        if self.commands.is_empty() {
            None
        } else {
            Some(self.commands.remove(0))
        }
    }
    
    /// Parse packets from buffer
    fn parse_packets(&mut self) {
        while self.rx_buffer.len() >= 2 {
            let cmd_id = self.rx_buffer[0];
            let payload_len = self.rx_buffer[1] as usize;
            
            // Check if full packet is available
            if self.rx_buffer.len() < 2 + payload_len {
                break; // Need more data
            }
            
            // Extract payload
            let payload = &self.rx_buffer[2..2 + payload_len];
            
            // Parse command
            match cmd_id {
                0x01 => {
                    // NeuronFiring
                    if payload_len >= 1 && payload_len % 2 == 1 {
                        let count = payload[0] as usize;
                        let mut coords = Vec::new();
                        for i in 0..count {
                            if 1 + i * 2 + 1 < payload.len() {
                                let x = payload[1 + i * 2];
                                let y = payload[1 + i * 2 + 1];
                                let _ = coords.push((x, y));
                            }
                        }
                        let _ = self.commands.push(Command::NeuronFiring { coordinates: coords });
                    }
                }
                0x02 => {
                    // SetGpio
                    if payload_len == 2 {
                        let pin = payload[0];
                        let value = payload[1] != 0;
                        let _ = self.commands.push(Command::SetGpio { pin, value });
                    }
                }
                0x03 => {
                    // SetPwm
                    if payload_len == 2 {
                        let pin = payload[0];
                        let duty = payload[1];
                        let _ = self.commands.push(Command::SetPwm { pin, duty });
                    }
                }
                0x04 => {
                    // SetLedMatrix
                    if payload_len == 25 {
                        let mut data = [0u8; 25];
                        data.copy_from_slice(payload);
                        let _ = self.commands.push(Command::SetLedMatrix { data });
                    }
                }
                0x05 => {
                    // GetCapabilities
                    let _ = self.commands.push(Command::GetCapabilities);
                }
                _ => {
                    // Unknown command - skip
                }
            }
            
            // Remove processed packet from buffer
            for _ in 0..(2 + payload_len) {
                self.rx_buffer.remove(0);
            }
        }
    }
}

impl Default for FeagiProtocol {
    fn default() -> Self {
        Self::new()
    }
}

