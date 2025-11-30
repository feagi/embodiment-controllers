#![no_std]
#![no_main]

use panic_halt as _;

// Minimal defmt implementation (required by embassy-executor/nrf-sdc)
#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        // No-op: we're not using defmt for logging
    }
    unsafe fn release() {
        // No-op
    }
    unsafe fn flush() {
        // No-op
    }
    unsafe fn write(_bytes: &[u8]) {
        // No-op: discard defmt output
    }
}

// Required by defmt for panic handling
#[defmt::panic_handler]
fn panic() -> ! {
    // Use panic-halt's panic handler
    cortex_m::asm::bkpt();
    loop {
        cortex_m::asm::wfi();
    }
}
use microbit_bsp::Microbit;

// BLE-specific imports (only when transport-ble is enabled)
#[cfg(feature = "transport-ble")]
use microbit_bsp::ble::{MultiprotocolServiceLayer, SoftdeviceController};

// BLE-specific modules (only compiled when transport-ble is enabled)
#[cfg(feature = "transport-ble")]
mod ble_compat;
#[cfg(feature = "transport-ble")]
mod ble_stack;

// USB-specific modules (only compiled when transport-usb is enabled)
#[cfg(feature = "transport-usb")]
mod usb_vbus;
#[cfg(feature = "transport-usb")]
mod protocol;

// Common modules (always compiled)
mod bluetooth;
mod gpio_controller;
mod sensors;

use bluetooth::BluetoothService;
use gpio_controller::GpioController;
use sensors::Sensors;

// Include build-time configuration
include!(concat!(env!("OUT_DIR"), "/config.rs"));

// Shared state between BLE task and main loop
// Using simple static buffers with manual synchronization
// Note: Embassy executor is single-threaded, so this is safe
use heapless::Vec;

// Buffer for BLE data (BLE task -> Main loop)
static mut BLE_RX_BUFFER: Option<heapless::Vec<u8, 256>> = None;
// Buffer for sensor data (Main loop -> BLE task)  
static mut BLE_TX_BUFFER: Option<heapless::Vec<u8, 256>> = None;

// ============================================================================
// BLE VARIANT - Main function for Bluetooth Low Energy transport
// ============================================================================
#[cfg(feature = "transport-ble")]
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize micro:bit board using microbit-bsp
    let board = Microbit::default();
    
    // Get display from board (microbit-bsp manages peripherals)
    // The display field is a LedMatrix
    let mut display = board.display;
    
    // Startup sequence: Show FEAGI letters (BEFORE BLE init to ensure it always runs)
    use embassy_time::{Duration, Timer};
    use microbit_bsp::display::Frame;
    
    // Show "F"
    {
        let mut frame = Frame::<5, 5>::empty();
        let pattern = [
            [1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0],
            [1, 1, 1, 1, 0],
            [1, 0, 0, 0, 0],
            [1, 0, 0, 0, 0],
        ];
        for y in 0..5 {
            for x in 0..5 {
                if pattern[y][x] > 0 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(500)).await;
    }
    
    // Show "E"
    {
        let mut frame = Frame::<5, 5>::empty();
        let pattern = [
            [1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0],
            [1, 1, 1, 1, 0],
            [1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1],
        ];
        for y in 0..5 {
            for x in 0..5 {
                if pattern[y][x] > 0 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(500)).await;
    }

    // Show "A"
    {
        let mut frame = Frame::<5, 5>::empty();
        let pattern = [
            [0, 1, 1, 1, 0],
            [1, 0, 0, 0, 1],
            [1, 1, 1, 1, 1],
            [1, 0, 0, 0, 1],
            [1, 0, 0, 0, 1],
        ];
        for y in 0..5 {
            for x in 0..5 {
                if pattern[y][x] > 0 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(500)).await;
    }

    // Show "G"
    {
        let mut frame = Frame::<5, 5>::empty();
        let pattern = [
            [0, 1, 1, 1, 0],
            [1, 0, 0, 0, 0],
            [1, 0, 1, 1, 1],
            [1, 0, 0, 0, 1],
            [0, 1, 1, 1, 0],
        ];
        for y in 0..5 {
            for x in 0..5 {
                if pattern[y][x] > 0 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(500)).await;
    }

    // Show "I"
    {
        let mut frame = Frame::<5, 5>::empty();
        let pattern = [
            [1, 1, 1, 1, 1],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0],
            [1, 1, 1, 1, 1],
        ];
        for y in 0..5 {
            for x in 0..5 {
                if pattern[y][x] > 0 {
                    frame.set(x, y);
                }
            }
        }
        display.display(frame, Duration::from_millis(500)).await;
    }

    // Clear display
    let clear_frame = Frame::<5, 5>::empty();
    display.display(clear_frame, Duration::from_millis(30)).await;
    
    // Initialize BLE using microbit-bsp's built-in TrouBLE support
    // When trouble feature is enabled, board has a 'ble' field
    let (sdc, mpsl) = board
        .ble
        .init(board.timer0, board.rng)
        .expect("BLE Stack failed to initialize");
    
    // Spawn MPSL task to run the Multiprotocol Service Layer
    _spawner.must_spawn(mpsl_task(mpsl));
    
    // Initialize BLE stack with Softdevice Controller
    let mut ble_stack = ble_stack::BleStack::new(BLUETOOTH_NAME, sdc).await
        .expect("Failed to initialize BLE stack");
    
    // Start BLE advertising
    ble_stack.start_advertising(BLUETOOTH_NAME).await
        .expect("Failed to start BLE advertising");
    
    // Spawn BLE task to handle events
    _spawner.must_spawn(ble_task(ble_stack));
    
    // Create a simple display buffer for LED matrix
    let mut display_buffer = [[0u8; 5]; 5];
    let mut sensors = Sensors::new();
    let mut gpio = GpioController::new();
    let mut bluetooth = BluetoothService::new(BLUETOOTH_NAME);
    
    // Main control loop (async)
    let mut loop_count: u32 = 0;
    loop {
        // Read sensors
        let sensor_data = sensors.read_all();
        
        // Process BLE data if available
        unsafe {
            if let Some(ref ble_data) = BLE_RX_BUFFER.take() {
                bluetooth.process_received_data(ble_data);
            }
        }
        
        // Check for Bluetooth commands
        if let Some(cmd) = bluetooth.receive_command() {
            match cmd {
                bluetooth::Command::SetGpio { pin, value } => {
                    gpio.set_digital(pin, value);
                }
                bluetooth::Command::SetPwm { pin, duty } => {
                    gpio.set_pwm(pin, duty);
                }
                bluetooth::Command::SetLedMatrix { data } => {
                    if OUTPUT_LED_MATRIX_ENABLED {
                        // Update display buffer from data
                        for (i, &brightness) in data.iter().enumerate() {
                            let y = i / 5;
                            let x = i % 5;
                            display_buffer[y][x] = brightness;
                        }
                    }
                }
                bluetooth::Command::NeuronFiring { coordinates } => {
                    if OUTPUT_LED_MATRIX_ENABLED {
                        // Clear buffer first
                        display_buffer = [[0; 5]; 5];
                        // Set LEDs for each fired neuron
                        for &(x, y) in coordinates.iter() {
                            if x < 5 && y < 5 {
                                display_buffer[y as usize][x as usize] = 255;
                            }
                        }
                    }
                }
                bluetooth::Command::GetCapabilities => {
                    let caps = bluetooth.get_capabilities_data("{\"sensors\":{\"accel\":true,\"mag\":true,\"temp\":true,\"buttons\":true},\"gpio\":{\"digital\":8,\"analog\":3,\"pwm\":8},\"display\":{\"matrix\":true}}");
                    unsafe {
                        BLE_TX_BUFFER = Some(caps);
                    }
                }
            }
        }
        
        // Check for neuron firing data
        if let Some(neuron_coords) = bluetooth.receive_neuron_data() {
            if OUTPUT_LED_MATRIX_ENABLED {
                // Clear buffer first
                display_buffer = [[0; 5]; 5];
                // Set LEDs for each fired neuron
                for &(x, y) in neuron_coords.iter() {
                    if x < 5 && y < 5 {
                        display_buffer[y as usize][x as usize] = 255;
                    }
                }
            }
        }
        
        // Update LED display
        if OUTPUT_LED_MATRIX_ENABLED {
            let mut frame = Frame::<5, 5>::empty();
            for y in 0..5 {
                for x in 0..5 {
                    if display_buffer[y][x] > 127 {
                        frame.set(x, y);
                    }
                }
            }
            display.display(frame, Duration::from_millis(30)).await;
        }
        
        // Async delay (10ms)
        Timer::after(Duration::from_millis(10)).await;
        loop_count = loop_count.wrapping_add(1);
    }
}

// ============================================================================
// USB VARIANT - Main function for USB CDC Serial transport
// ============================================================================
#[cfg(feature = "transport-usb")]
#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    use embassy_nrf::{bind_interrupts, usb, peripherals};
    use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
    use embassy_usb::{Builder, Config};
    use embassy_time::{Duration, Timer};
    use crate::protocol::{FeagiProtocol, Command};
    use crate::usb_vbus::AlwaysOnVbus;
    
    // Initialize embassy-nrf FIRST for USB (can't use microbit-bsp at same time)
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::Internal;
    nrf_config.lfclk_source = embassy_nrf::config::LfclkSource::InternalRC;
    nrf_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P7;
    nrf_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P7;
    let p = embassy_nrf::init(nrf_config);
    
    // USB interrupt bindings
    bind_interrupts!(struct Irqs {
        USBD => usb::InterruptHandler<peripherals::USBD>;
    });
    
    // Create USB driver with always-on VBUS detect
    static VBUS_DETECT: AlwaysOnVbus = AlwaysOnVbus::new();
    let driver = usb::Driver::new(p.USBD, Irqs, &VBUS_DETECT);
    
    // Static storage for USB descriptors and state
    static CONFIG_DESC: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    static BOS_DESC: static_cell::StaticCell<[u8; 256]> = static_cell::StaticCell::new();
    static CONTROL_BUF: static_cell::StaticCell<[u8; 128]> = static_cell::StaticCell::new();
    static STATE: static_cell::StaticCell<State> = static_cell::StaticCell::new();
    
    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 128]);
    let state = STATE.init(State::new());
    
    // USB device configuration
    let mut config = Config::new(0x16c0, 0x27dd); // Generic VID/PID
    config.manufacturer = Some("Neuraville");
    config.product = Some("FEAGI-microbit");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    
    // Build USB device
    let mut builder = Builder::new(driver, config, config_desc, bos_desc, &mut [], control_buf);
    
    // Create CDC ACM class
    let cdc_class = CdcAcmClass::new(&mut builder, state, 64);
    
    // Store CDC class in static for access from other tasks
    static CDC: static_cell::StaticCell<embassy_sync::mutex::Mutex<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, Option<CdcAcmClass<'static, embassy_nrf::usb::Driver<'static, embassy_nrf::peripherals::USBD, &'static AlwaysOnVbus>>>>> = static_cell::StaticCell::new();
    let cdc = CDC.init(embassy_sync::mutex::Mutex::new(Some(cdc_class)));
    
    // Build and spawn USB device task
    let usb_device = builder.build();
    spawner.must_spawn(usb_device_task(usb_device));
    
    // Initialize FEAGI protocol
    let mut protocol = FeagiProtocol::new();
    
    // Wait for USB connection (CDC ACM DTR signal)
    loop {
        let mut cdc_lock = cdc.lock().await;
        if let Some(ref mut cdc_instance) = *cdc_lock {
            cdc_instance.wait_connection().await;
            break;
        }
    }
    
    // NOTE: LED display temporarily disabled in USB mode
    // Will implement raw GPIO control in future update
    
    // Main loop: read from USB, process commands (no display yet)
    loop {
        // Read from USB CDC
        let mut buf = [0u8; 64];
        let mut cdc_lock = cdc.lock().await;
        if let Some(ref mut cdc_instance) = *cdc_lock {
            match cdc_instance.read_packet(&mut buf).await {
                Ok(len) if len > 0 => {
                    drop(cdc_lock);
                    protocol.process_received_data(&buf[..len]);
                }
                _ => {
                    drop(cdc_lock);
                }
            }
        } else {
            drop(cdc_lock);
        }
        
        // Process commands from protocol (data is received but not displayed)
        while let Some(cmd) = protocol.receive_command() {
            match cmd {
                Command::NeuronFiring { coordinates: _ } => {
                    // TODO: Display via raw GPIO
                }
                Command::SetLedMatrix { data: _ } => {
                    // TODO: Display via raw GPIO
                }
                Command::SetGpio { pin: _, value: _ } => {
                    // TODO: GPIO control
                }
                Command::SetPwm { pin: _, duty: _ } => {
                    // TODO: PWM control
                }
                Command::GetCapabilities => {
                    // TODO: Send capabilities JSON
                }
            }
        }
        
        // Small delay
        Timer::after(Duration::from_millis(10)).await;
    }
}

// USB device task (runs USB stack)
#[cfg(feature = "transport-usb")]
#[embassy_executor::task]
async fn usb_device_task(
    mut usb_device: embassy_usb::UsbDevice<
        'static,
        embassy_nrf::usb::Driver<
            'static,
            embassy_nrf::peripherals::USBD,
            &'static crate::usb_vbus::AlwaysOnVbus,
        >,
    >,
) -> ! {
    usb_device.run().await
}

// ============================================================================
// BLE TASKS - Only compiled when transport-ble is enabled
// ============================================================================

// MPSL task to run the Multiprotocol Service Layer
#[cfg(feature = "transport-ble")]
#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

// BLE task to handle BLE events
#[cfg(feature = "transport-ble")]
#[embassy_executor::task]
async fn ble_task(mut ble_stack: ble_stack::BleStack<'static>) {
    loop {
        // Process BLE events
        ble_stack.process_events().await;
        
        // Check for received data and put it in RX buffer
        if let Some(data) = ble_stack.receive_data().await {
            unsafe {
                BLE_RX_BUFFER = Some(data);
            }
        }
        
        // Check for data to send and send it via BLE
        unsafe {
            if let Some(data) = BLE_TX_BUFFER.take() {
                if let Err(_) = ble_stack.send_notify(&data).await {
                    // If send fails, put data back (or drop it)
                }
            }
        }
        
        // Small delay to prevent busy loop
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
    }
}
