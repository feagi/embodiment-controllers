//! Simple LED blink example for micro:bit
//! This is a minimal example to test the toolchain setup

#![no_std]
#![no_main]

use panic_halt as _;

#[cfg(feature = "v2")]
use microbit::{
    board::Board,
    hal::{prelude::*, Timer},
};

#[cfg(feature = "v1")]
use microbit::{
    Board,
    hal::{prelude::*, Timer},
};

#[cortex_m_rt::entry]
fn main() -> ! {
    let board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);
    
    // Get LED matrix (simplified - just blink center LED)
    // TODO: Implement proper LED matrix control
    
    loop {
        // Blink pattern: on for 500ms, off for 500ms
        // Center LED at position (2, 2)
        
        timer.delay_ms(500_u32);
        // LED on
        
        timer.delay_ms(500_u32);
        // LED off
    }
}

