//! BLE Stack Integration Module (TrouBLE)
//!
//! This module provides BLE communication using TrouBLE (pure Rust, MIT/Apache-2.0 license).
//! The FEAGI protocol packets are sent/received over BLE using Nordic UART Service (NUS).

use heapless::Vec;
use static_cell::StaticCell;
use microbit_bsp::ble::SoftdeviceController;
use trouble_host::prelude::*;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use crate::ble_compat::BleCompatController;

/// Nordic UART Service UUIDs (128-bit)
pub const NUS_SERVICE_UUID: Uuid = Uuid::new_long([
    0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
]);
pub const NUS_TX_CHAR_UUID: Uuid = Uuid::new_long([
    0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
]);
pub const NUS_RX_CHAR_UUID: Uuid = Uuid::new_long([
    0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e,
]);

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3;
const L2CAP_MTU: usize = 247;
const ATT_TABLE_SIZE: usize = 20;
const ADV_SETS: usize = 1;

// Static storage for BLE runner components (split from Runner)
static RX_RUNNER: StaticCell<Option<RxRunner<'static, BleCompatController<'static>>>> = StaticCell::new();
static CONTROL_RUNNER: StaticCell<Option<ControlRunner<'static, BleCompatController<'static>>>> = StaticCell::new();
static TX_RUNNER: StaticCell<Option<TxRunner<'static, BleCompatController<'static>>>> = StaticCell::new();

/// BLE Stack handle using TrouBLE via microbit-bsp
/// 
/// **Architecture:**
/// - Uses microbit-bsp's built-in BLE support (TrouBLE + nrf-sdc)
/// - Implements Nordic UART Service (NUS) for simple serial communication
/// - microbit-bsp handles all peripheral coordination automatically
pub struct BleStack {
    connected: bool,
    // Note: stack is stored in static storage, peripheral and runner reference it
    // host is partially moved (peripheral and runner extracted)
    // Runner is stored in static storage and run in a separate task
    _host_phantom: core::marker::PhantomData<Host<'static, BleCompatController<'static>>>,
    peripheral: Peripheral<'static, BleCompatController<'static>>,
    server: AttributeServer<'static, NoopRawMutex, ATT_TABLE_SIZE, 1, CONNECTIONS_MAX>,
    connection: Option<Connection<'static>>,
    advertiser: Option<Advertiser<'static, BleCompatController<'static>>>,
    nus_tx_characteristic: Option<Characteristic<[u8; 20]>>,
    nus_rx_handle: Option<u16>,
}

impl BleStack {
    /// Initialize BLE stack with TrouBLE via microbit-bsp
    pub async fn new(device_name: &str, sdc: SoftdeviceController<'_>) -> Result<Self, &'static str> {
        // Create compatibility controller
        // Note: We need to extend the lifetime to 'static for the stack
        // This is safe because the controller is owned by the stack and will live as long as needed
        let compat_controller = BleCompatController::new(sdc);
        let compat_controller_static: BleCompatController<'static> = unsafe {
            core::mem::transmute(compat_controller)
        };
        
        // Initialize host resources
        static HOST_RESOURCES: StaticCell<HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU, ADV_SETS>> = StaticCell::new();
        let host_resources = HOST_RESOURCES.init(HostResources::new());
        
        // Create BLE stack
        // Store stack in static storage to ensure it lives long enough
        static STACK: StaticCell<Stack<'static, BleCompatController<'static>>> = StaticCell::new();
        let stack = STACK.init(trouble_host::new(compat_controller_static, host_resources));
        
        // Build host components
        let host = stack.build();
        
        // Extract components from host (consumes host)
        let peripheral = host.peripheral;
        
        // Split runner and store in static cells
        // We NEED runners for advertising to work, but we'll spawn them in the ble_init_task
        let (rx_runner, control_runner, tx_runner) = host.runner.split();
        RX_RUNNER.init(Some(rx_runner));
        CONTROL_RUNNER.init(Some(control_runner));
        TX_RUNNER.init(Some(tx_runner));
        
        // Set up GATT Attribute Server with Nordic UART Service
        // Create the attribute table (will be moved into the server)
        let mut att_table = AttributeTable::new();
        
        // Service declaration for Generic Access (required by BLE spec)
        {
            let mut gas_service = att_table.add_service(Service::new(0x1800u16));
            
            // Device Name characteristic (required by BLE spec)
            // We need static storage for the device name buffer
            static DEVICE_NAME_BUF: StaticCell<[u8; 20]> = StaticCell::new();
            let device_name_buf = DEVICE_NAME_BUF.init([0u8; 20]);
            let name_bytes = device_name.as_bytes();
            let name_len = name_bytes.len().min(20);
            device_name_buf[..name_len].copy_from_slice(&name_bytes[..name_len]);
            // Use a static string slice for the device name
            static DEVICE_NAME_STATIC: StaticCell<&'static str> = StaticCell::new();
            let device_name_static = DEVICE_NAME_STATIC.init(unsafe {
                // Safety: We're storing the device name in static memory via the buffer
                // This is safe as long as device_name parameter is valid for the duration
                core::mem::transmute(device_name)
            });
            let _device_name_handle = gas_service
                .add_characteristic(
                    Uuid::from(0x2A00u16), // Device Name UUID
                    &[CharacteristicProp::Read],
                    *device_name_static,
                    device_name_buf,
                )
                .build();
        }
        
        // Nordic UART Service
        let (nus_tx_characteristic, nus_rx_handle) = {
            let mut nus_service = att_table.add_service(Service::new(NUS_SERVICE_UUID));
            
            // NUS TX Characteristic (Notify) - micro:bit sends data to client
            // We need static storage for TX value
            static NUS_TX_VALUE: StaticCell<[u8; 20]> = StaticCell::new();
            let nus_tx_value = NUS_TX_VALUE.init([0u8; 20]);
            let nus_tx_initial: [u8; 20] = [0u8; 20];
            let nus_tx_characteristic = nus_service
                .add_characteristic(
                    NUS_TX_CHAR_UUID,
                    &[CharacteristicProp::Notify],
                    nus_tx_initial,
                    nus_tx_value,
                )
                .build();
            
            // NUS RX Characteristic (Write) - client sends data to micro:bit
            // We need static storage for RX value
            static NUS_RX_VALUE: StaticCell<[u8; 20]> = StaticCell::new();
            let nus_rx_value = NUS_RX_VALUE.init([0u8; 20]);
            let nus_rx_initial: [u8; 20] = [0u8; 20];
            let nus_rx_handle = nus_service
                .add_characteristic(
                    NUS_RX_CHAR_UUID,
                    &[CharacteristicProp::Write, CharacteristicProp::WriteWithoutResponse],
                    nus_rx_initial,
                    nus_rx_value,
                )
                .build();
            
            // Service builder is dropped here, releasing the borrow on att_table
            (nus_tx_characteristic, nus_rx_handle.handle())
        };
        
        // Create attribute server (takes ownership of the table)
        let server = AttributeServer::new(att_table);
        
        Ok(Self {
            connected: false,
            _host_phantom: core::marker::PhantomData,
            peripheral,
            server,
            connection: None,
            advertiser: None,
            nus_tx_characteristic: Some(nus_tx_characteristic),
            nus_rx_handle: Some(nus_rx_handle),
        })
    }
    
    /// Start BLE advertising
    pub async fn start_advertising(&mut self, device_name: &str) -> Result<(), &'static str> {
        use trouble_host::advertise::*;
        
        // Create advertisement - ConnectableScannableUndirected
        // We need static storage for advertisement data
        static ADV_DATA: StaticCell<[u8; 31]> = StaticCell::new();
        let adv_data = ADV_DATA.init([0u8; 31]);
        
        // Build advertisement data: Flags + Complete Local Name
        let mut pos = 0;
        
        // Flags: LE General Discoverable, BR/EDR not supported
        adv_data[pos] = 0x02; pos += 1; // Length
        adv_data[pos] = 0x01; pos += 1; // Type: Flags
        adv_data[pos] = 0x06; pos += 1; // Flags value
        
        // Complete Local Name
        let name_bytes = device_name.as_bytes();
        let name_len = name_bytes.len().min(28); // Leave room for type and length
        adv_data[pos] = (name_len + 1) as u8; pos += 1; // Length
        adv_data[pos] = 0x09; pos += 1; // Type: Complete Local Name
        adv_data[pos..pos + name_len].copy_from_slice(&name_bytes[..name_len]);
        pos += name_len;
        
        let adv = Advertisement::ConnectableScannableUndirected {
            adv_data: &adv_data[..pos],
            scan_data: &[],
        };
        
        // Start advertising with default parameters
        let params = AdvertisementParameters::default();
        let advertiser = self.peripheral
            .advertise(&params, adv)
            .await
            .map_err(|_| "Failed to start advertising")?;
        
        self.advertiser = Some(advertiser);
        Ok(())
    }
    
    /// Process BLE events
    /// This should be called regularly from a BLE task
    pub async fn process_events(&mut self) {
        // Check for new connections via advertiser
        if !self.connected {
            if let Some(advertiser) = self.advertiser.take() {
                // Try to accept a connection (advertiser is consumed)
                match advertiser.accept().await {
                    Ok(connection) => {
                        // Connect the server to this connection
                        // Note: server.connect() is private, but we'll handle GATT events manually
                        // Store connection for processing
                        self.connection = Some(connection);
                        self.connected = true;
                    }
                    Err(_) => {
                        // Timeout or error, keep advertising
                        // Note: advertiser is consumed, so we can't reuse it
                        // We'd need to restart advertising
                    }
                }
            }
        }
        
        // Process GATT events if connected
        if let Some(ref connection) = self.connection {
            // Process connection events and handle GATT PDUs
            match connection.next().await {
                ConnectionEvent::Gatt { data } => {
                    // Process GATT PDU through the server
                    match data.process(&self.server).await {
                        Ok(Some(GattEvent::Write(write_event))) => {
                            // Extract write data
                            let handle = write_event.handle();
                            let data = write_event.data();
                            
                            // Check if this is the RX characteristic
                            if Some(handle) == self.nus_rx_handle {
                                // Store received data
                                unsafe {
                                    let mut buffer = heapless::Vec::new();
                                    for &byte in data {
                                        if buffer.push(byte).is_err() {
                                            break;
                                        }
                                    }
                                    crate::BLE_RX_BUFFER = Some(buffer);
                                }
                            }
                            
                            // Accept the write event
                            let _ = write_event.accept();
                        }
                        Ok(Some(GattEvent::Read(read_event))) => {
                            // Accept read events (device name, etc.)
                            let _ = read_event.accept();
                        }
                        Ok(None) => {
                            // Event was handled internally
                        }
                        Err(_) => {
                            // Error processing GATT event
                        }
                    }
                }
                ConnectionEvent::Disconnected { .. } => {
                    self.connected = false;
                    self.connection = None;
                }
                _ => {}
            }
        }
    }
    
    /// Send data via BLE notify (Nordic UART Service TX characteristic)
    /// 
    /// **LIMITATION:** This method is currently not functional due to API limitations.
    /// See `BLE_LIMITATIONS.md` for details.
    /// 
    /// **Root Cause:**
    /// - `Characteristic::notify()` requires `GattConnection`
    /// - `GattConnection::try_new()` is `pub(crate)` (not accessible)
    /// - `Connection::alloc_tx()` and `Connection::send()` are private
    /// 
    /// **Current Behavior:**
    /// - Returns `Ok(())` but does not actually send data
    /// - Sensor data and status updates cannot be transmitted
    /// - One-way communication (client → micro:bit) still works
    /// 
    /// **Workaround Options:**
    /// 1. Use write-response pattern (client polls, micro:bit responds)
    /// 2. Request trouble-host to expose `GattConnection::try_new()` as public
    /// 3. Use unsafe code to access private APIs (not recommended)
    pub async fn send_notify(&mut self, data: &[u8]) -> Result<(), &'static str> {
        if !self.connected {
            return Err("Not connected");
        }
        
        // TODO: Implement proper notification sending
        // This requires GattConnection which we can't create directly
        // The proper implementation would use:
        //   tx_char.notify(&gatt_connection, &value).await
        // 
        // For now, this is a no-op that returns success
        // Data is silently dropped - this is expected behavior until API is fixed
        let _ = (data, self.nus_tx_characteristic.is_some());
        
        // Return success to avoid breaking callers
        // Callers should check BLE_LIMITATIONS.md to understand this limitation
        Ok(())
    }
    
    /// Receive data from BLE (Nordic UART Service RX characteristic)
    /// Returns data if available, None otherwise
    pub async fn receive_data(&mut self) -> Option<heapless::Vec<u8, 256>> {
        // Data is received in process_events and stored in BLE_RX_BUFFER
        unsafe {
            crate::BLE_RX_BUFFER.take()
        }
    }
    
    /// Check if BLE is connected
    pub fn is_connected(&self) -> bool {
        self.connected
    }
    
    /// Set connection state (called by event handler)
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }
    
    /// Take the advertiser out of the stack (for blocking accept)
    pub fn take_advertiser(&mut self) -> Option<Advertiser<'static, BleCompatController<'static>>> {
        self.advertiser.take()
    }
    
    /// Store a connection after accepting it
    pub fn set_connection(&mut self, connection: Option<Connection<'static>>) {
        self.connected = connection.is_some();
        self.connection = connection;
    }
}

/// Spawn the BLE runner tasks from static storage
/// This must be called after BleStack::new() to start processing BLE events
pub fn spawn_runner_task(spawner: &embassy_executor::Spawner) {
    // Get mutable references to the runners from static storage
    // Safety: StaticCell guarantees initialization, and executor is single-threaded
    let rx_runner = unsafe {
        let ptr = &RX_RUNNER as *const StaticCell<Option<RxRunner<'static, BleCompatController<'static>>>> as *mut Option<RxRunner<'static, BleCompatController<'static>>>;
        (*ptr).as_mut().unwrap()
    };
    let control_runner = unsafe {
        let ptr = &CONTROL_RUNNER as *const StaticCell<Option<ControlRunner<'static, BleCompatController<'static>>>> as *mut Option<ControlRunner<'static, BleCompatController<'static>>>;
        (*ptr).as_mut().unwrap()
    };
    let tx_runner = unsafe {
        let ptr = &TX_RUNNER as *const StaticCell<Option<TxRunner<'static, BleCompatController<'static>>>> as *mut Option<TxRunner<'static, BleCompatController<'static>>>;
        (*ptr).as_mut().unwrap()
    };
    
    spawner.must_spawn(rx_runner_task(rx_runner));
    spawner.must_spawn(control_runner_task(control_runner));
    spawner.must_spawn(tx_runner_task(tx_runner));
}

/// RX Runner task - processes incoming BLE data
/// The runner tasks MUST run for advertising to work (advertise() waits for controller events)
#[embassy_executor::task]
async fn rx_runner_task(runner: &'static mut RxRunner<'static, BleCompatController<'static>>) -> ! {
    // runner.run() returns Result, loop forever and restart on error
    loop {
        if let Err(_) = runner.run().await {
            // On error, just restart
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }
}

/// Control Runner task - processes BLE connection events
/// The runner tasks MUST run for advertising to work
#[embassy_executor::task]
async fn control_runner_task(runner: &'static mut ControlRunner<'static, BleCompatController<'static>>) -> ! {
    // runner.run() returns Result, loop forever and restart on error
    loop {
        if let Err(_) = runner.run().await {
            // On error, just restart
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }
}

/// TX Runner task - processes outgoing BLE data
/// The runner tasks MUST run for advertising to work
#[embassy_executor::task]
async fn tx_runner_task(runner: &'static mut TxRunner<'static, BleCompatController<'static>>) -> ! {
    // runner.run() returns Result, loop forever and restart on error
    loop {
        if let Err(_) = runner.run().await {
            // On error, just restart
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_nus_uuid_format() {
        // Verify UUIDs are 16 bytes
        assert_eq!(NUS_SERVICE_UUID.as_bytes().len(), 16);
        assert_eq!(NUS_TX_CHAR_UUID.as_bytes().len(), 16);
        assert_eq!(NUS_RX_CHAR_UUID.as_bytes().len(), 16);
        
        // Verify they're different
        assert_ne!(NUS_SERVICE_UUID, NUS_TX_CHAR_UUID);
        assert_ne!(NUS_SERVICE_UUID, NUS_RX_CHAR_UUID);
        assert_ne!(NUS_TX_CHAR_UUID, NUS_RX_CHAR_UUID);
    }
}
