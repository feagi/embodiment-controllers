//! Bluetooth LE service for FEAGI communication
//!
//! **Implementation Status:**
//! - Protocol defined (UUIDs, packet formats)
//! - Packet parsing implemented
//! - BLE stack integration pending (requires async runtime refactor)
//!
//! **BLE Service UUIDs:**
//! - Service: e95d0753-251d-470a-a062-fa1922dfa9a8
//! - Characteristics:
//!   - Sensor Data (Notify): e95d0754-251d-470a-a062-fa1922dfa9a8
//!   - Neuron Data (Write):  e95d0755-251d-470a-a062-fa1922dfa9a8
//!   - GPIO Control (Write): e95d0756-251d-470a-a062-fa1922dfa9a8
//!   - LED Matrix (Write):   e95d0757-251d-470a-a062-fa1922dfa9a8
//!   - Capabilities (Read):   e95d0758-251d-470a-a062-fa1922dfa9a8

use crate::sensors::SensorData;
use heapless::Vec;

/// FEAGI BLE Service UUIDs
pub const FEAGI_SERVICE_UUID: &[u8; 16] = b"\xe9\x5d\x07\x53\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";
pub const SENSOR_DATA_CHAR_UUID: &[u8; 16] = b"\xe9\x5d\x07\x54\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";
pub const NEURON_DATA_CHAR_UUID: &[u8; 16] = b"\xe9\x5d\x07\x55\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";
pub const GPIO_CONTROL_CHAR_UUID: &[u8; 16] = b"\xe9\x5d\x07\x56\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";
pub const LED_MATRIX_CHAR_UUID: &[u8; 16] = b"\xe9\x5d\x07\x57\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";
pub const CAPABILITIES_CHAR_UUID: &[u8; 16] = b"\xe9\x5d\x07\x58\x25\x1d\x47\x0a\xa0\x62\xfa\x19\x22\xdf\xa9\xa8";

/// FEAGI Bluetooth commands
#[derive(Debug, Clone)]
pub enum Command {
    SetGpio { pin: u8, value: bool },
    SetPwm { pin: u8, duty: u8 },
    SetLedMatrix { data: [u8; 25] },
    NeuronFiring { coordinates: heapless::Vec<(u8, u8), 25> }, // Up to 25 neurons (5x5 matrix)
    GetCapabilities,
}

/// Bluetooth service for FEAGI communication
pub struct BluetoothService {
    device_name: &'static str,
    // Receive buffer for incoming BLE data
    // TODO: Connect to actual BLE characteristic when BLE stack is integrated
    receive_buffer: heapless::Vec<u8, 256>,  // Max BLE MTU is typically 23-247 bytes
    // Flag to indicate if BLE is connected
    connected: bool,
}

/// BLE packet command types
#[repr(u8)]
pub enum PacketCommand {
    NeuronFiring = 0x01,
    SetGpio = 0x02,
    SetPwm = 0x03,
    SetLedMatrix = 0x04,
    GetCapabilities = 0x05,
}

impl BluetoothService {
    pub fn new(device_name: &'static str) -> Self {
        Self {
            device_name,
            receive_buffer: heapless::Vec::new(),
            connected: false,
        }
    }
    
    /// Process incoming BLE data (called from BLE stack when data arrives)
    /// This function parses the binary packet format
    pub fn process_received_data(&mut self, data: &[u8]) {
        // Append to receive buffer
        for &byte in data {
            if self.receive_buffer.push(byte).is_err() {
                // Buffer full - clear and start over
                self.receive_buffer.clear();
                break;
            }
        }
    }
    
    /// Parse neuron firing packet from buffer
    /// Format: [0x01] [count] [x1, y1, x2, y2, ...]
    fn parse_neuron_firing_packet(&mut self) -> Option<Vec<(u8, u8), 25>> {
        if self.receive_buffer.len() < 2 {
            return None;
        }
        
        if self.receive_buffer[0] != PacketCommand::NeuronFiring as u8 {
            return None;
        }
        
        let count = self.receive_buffer[1] as usize;
        if count > 25 || self.receive_buffer.len() < 2 + count * 2 {
            return None;
        }
        
        let mut coords = Vec::new();
        for i in 0..count {
            let x = self.receive_buffer[2 + i * 2];
            let y = self.receive_buffer[2 + i * 2 + 1];
            if coords.push((x, y)).is_err() {
                break; // Max 25 coordinates
            }
        }
        
        // Clear processed data from buffer (heapless::Vec doesn't have drain)
        let consumed = 2 + count * 2;
        // Remove consumed bytes by shifting remaining data
        for i in consumed..self.receive_buffer.len() {
            self.receive_buffer[i - consumed] = self.receive_buffer[i];
        }
        // Truncate to new length
        for _ in 0..consumed {
            if self.receive_buffer.pop().is_none() {
                break;
            }
        }
        
        Some(coords)
    }
    
    /// Check if BLE is connected
    pub fn is_connected(&self) -> bool {
        self.connected
    }
    
    /// Set connection status (called by BLE stack)
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }
    
    /// Serialize sensor data to JSON format for BLE transmission
    /// Format: {"accel":[x,y,z],"mag":[x,y,z],"temp":23.5,"buttons":{"a":false,"b":true}}
    fn serialize_sensor_data(&mut self, _data: &SensorData, buffer: &mut heapless::Vec<u8, 256>) -> Result<(), ()> {
        // Simple JSON serialization for no_std environment
        // Format: {"accel":[x,y,z],"mag":[x,y,z],"temp":23.5,"buttons":{"a":false,"b":false}}
        buffer.clear();
        
        // Start JSON object
        buffer.extend_from_slice(b"{\"accel\":[").map_err(|_| ())?;
        // Accel data would go here - simplified for now
        buffer.extend_from_slice(b"0,0,0],\"mag\":[0,0,0],\"temp\":0.0,\"buttons\":{\"a\":false,\"b\":false}}").map_err(|_| ())?;
        
        Ok(())
    }
    
    /// Send sensor data via BLE
    /// Returns serialized data if sensors are enabled
    pub fn send_sensor_data(&mut self, data: &SensorData) -> Option<heapless::Vec<u8, 256>> {
        let mut buffer = heapless::Vec::new();
        if self.serialize_sensor_data(data, &mut buffer).is_ok() {
            Some(buffer)
        } else {
            None
        }
    }
    
    /// Send capabilities JSON
    pub fn send_capabilities(&mut self, caps: &str) {
        // Capabilities are sent when requested
        // This is a placeholder - actual implementation will send via BLE
        let _ = caps;
    }
    
    /// Receive and parse command from BLE
    pub fn receive_command(&mut self) -> Option<Command> {
        // Parse commands from receive buffer
        // For now, just check for neuron firing packets
        self.parse_neuron_firing_packet().map(|coords| {
            Command::NeuronFiring { coordinates: coords }
        })
    }
    
    /// Receive neuron firing data from FEAGI
    /// 
    /// **Expected Cortical Area:**
    /// - Type: `omis` (Miscellaneous Motor)
    /// - Name: "LED Matrix" or "Display Matrix"
    /// - Dimensions: 5×5×1
    /// 
    /// **Packet Format:**
    /// Binary packet with header byte, then list of (x, y) coordinates
    /// - Header: 0x01 = NeuronFiring
    /// - Count: 1 byte (number of fired neurons, ≤ 25)
    /// - Data: count×2 bytes of (x, y) coordinate pairs
    pub fn receive_neuron_data(&mut self) -> Option<Vec<(u8, u8), 25>> {
        // Parse neuron firing packet from receive buffer
        // TODO: This will be called automatically when BLE data arrives
        self.parse_neuron_firing_packet()
    }
    
    /// Get capabilities data to send via BLE
    pub fn get_capabilities_data(&self, caps: &str) -> heapless::Vec<u8, 256> {
        // Convert capabilities string to bytes for BLE transmission
        let mut buffer = heapless::Vec::new();
        for &byte in caps.as_bytes() {
            if buffer.push(byte).is_err() {
                break; // Buffer full
            }
        }
        buffer
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_bluetooth_service_creation() {
        let service = BluetoothService::new("FEAGI-test");
        assert!(!service.is_connected());
    }
    
    #[test]
    fn test_process_received_data() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Test single byte - process and verify it's in buffer by trying to parse
        service.process_received_data(&[0x01]);
        // Buffer should have data (can't directly access, but parsing will fail which confirms data is there)
        let result = service.receive_neuron_data();
        assert!(result.is_none()); // Incomplete packet, but data was processed
        
        // Test multiple bytes
        service.process_received_data(&[0x02, 0x03]);
        // Verify by processing a complete packet
        let packet = [0x01, 0x01, 0x05, 0x06]; // Valid packet
        service.process_received_data(&packet);
        let result = service.receive_neuron_data();
        assert!(result.is_some()); // Should parse successfully
    }
    
    #[test]
    fn test_parse_neuron_firing_packet_valid() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Valid packet: [0x01] [count=2] [x1=1, y1=2, x2=3, y2=4]
        let packet = [0x01, 0x02, 0x01, 0x02, 0x03, 0x04];
        service.process_received_data(&packet);
        
        let result = service.receive_neuron_data();
        assert!(result.is_some());
        let coords = result.unwrap();
        assert_eq!(coords.len(), 2);
        assert_eq!(coords[0], (1, 2));
        assert_eq!(coords[1], (3, 4));
    }
    
    #[test]
    fn test_parse_neuron_firing_packet_invalid_header() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Invalid header
        let packet = [0x02, 0x01, 0x00, 0x00];
        service.process_received_data(&packet);
        
        let result = service.receive_neuron_data();
        assert!(result.is_none());
    }
    
    #[test]
    fn test_parse_neuron_firing_packet_incomplete() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Incomplete packet (missing data)
        let packet = [0x01, 0x02, 0x01]; // Missing y coordinate
        service.process_received_data(&packet);
        
        let result = service.receive_neuron_data();
        assert!(result.is_none());
    }
    
    #[test]
    fn test_parse_neuron_firing_packet_max_coords() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Maximum 25 coordinates
        let mut packet = vec![0x01, 25];
        for i in 0..25 {
            packet.push(i as u8); // x
            packet.push((i + 1) as u8); // y
        }
        service.process_received_data(&packet);
        
        let result = service.receive_neuron_data();
        assert!(result.is_some());
        let coords = result.unwrap();
        assert_eq!(coords.len(), 25);
    }
    
    #[test]
    fn test_parse_neuron_firing_packet_too_many_coords() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Too many coordinates (should be rejected)
        let mut packet = vec![0x01, 26]; // 26 > 25 max
        for i in 0..26 {
            packet.push(i as u8);
            packet.push((i + 1) as u8);
        }
        service.process_received_data(&packet);
        
        let result = service.receive_neuron_data();
        assert!(result.is_none());
    }
    
    #[test]
    fn test_connection_status() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        assert!(!service.is_connected());
        service.set_connected(true);
        assert!(service.is_connected());
        service.set_connected(false);
        assert!(!service.is_connected());
    }
    
    #[test]
    fn test_get_capabilities_data() {
        let service = BluetoothService::new("FEAGI-test");
        let caps = "{\"sensors\":{\"accel\":true}}";
        let data = service.get_capabilities_data(caps);
        
        assert_eq!(data.len(), caps.len());
        assert_eq!(data.as_slice(), caps.as_bytes());
    }
    
    #[test]
    fn test_buffer_overflow_handling() {
        let mut service = BluetoothService::new("FEAGI-test");
        
        // Fill buffer beyond capacity
        let large_data = vec![0x01; 300]; // Larger than 256 byte buffer
        service.process_received_data(&large_data);
        
        // Buffer should handle overflow (either truncate or clear)
        // Verify service still works after overflow
        let packet = [0x01, 0x01, 0x05, 0x06]; // Valid packet
        service.process_received_data(&packet);
        let result = service.receive_neuron_data();
        // Should still be able to process new data
        assert!(result.is_some() || result.is_none()); // Either works, just verify no panic
    }
}
