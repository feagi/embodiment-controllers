#![no_std]
#![no_main]

use panic_halt as _;
use microbit_bsp::Microbit;
// BLE will be implemented using TrouBLE via microbit-bsp (pure Rust, MIT license)

mod ble_stack;
mod bluetooth;
mod gpio_controller;
mod led_display;
mod sensors;

use bluetooth::BluetoothService;
use gpio_controller::GpioController;
use led_display::LedDisplay;
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

// Use embassy-executor main macro
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize micro:bit board using microbit-bsp
    let board = Microbit::default();
    
    // Get display from board (microbit-bsp manages peripherals)
    let display = board.display;
    
    let mut display_wrapper = LedDisplay::new(display);
    let mut sensors = Sensors::new();
    let mut gpio = GpioController::new();
    let mut bluetooth = BluetoothService::new(BLUETOOTH_NAME);
    
    // Startup sequence: Show FEAGI letters
    use embassy_time::{Duration, Timer};
    
    display_wrapper.show_letter_f();
    display_wrapper.show().await;
    Timer::after(Duration::from_millis(500)).await;
    
    display_wrapper.show_letter_e();
    display_wrapper.show().await;
    Timer::after(Duration::from_millis(500)).await;
    
    display_wrapper.show_letter_a();
    display_wrapper.show().await;
    Timer::after(Duration::from_millis(500)).await;
    
    display_wrapper.show_letter_g();
    display_wrapper.show().await;
    Timer::after(Duration::from_millis(500)).await;
    
    display_wrapper.show_letter_i();
    display_wrapper.show().await;
    Timer::after(Duration::from_millis(500)).await;
    
    display_wrapper.clear();
    
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
                        display_wrapper.set_matrix(&data);
                    }
                }
                bluetooth::Command::NeuronFiring { coordinates } => {
                    if OUTPUT_LED_MATRIX_ENABLED {
                        display_wrapper.update_from_neurons(&coordinates);
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
                display_wrapper.update_from_neurons(&neuron_coords);
            }
        }
        
        // Update LED display
        if OUTPUT_LED_MATRIX_ENABLED {
            display_wrapper.show().await;
        }
        
        // Async delay (10ms)
        Timer::after(Duration::from_millis(10)).await;
        loop_count = loop_count.wrapping_add(1);
    }
}

// BLE implementation TODO:
// - Implement using TrouBLE or embassy-nrf BLE (pure Rust, MIT license)
// - nrf-softdevice removed due to license issues
// - Once BLE is implemented, integrate with main control loop
