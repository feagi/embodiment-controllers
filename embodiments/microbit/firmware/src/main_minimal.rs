#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use microbit::{board::Board, display::blocking::Display, hal::Timer};

#[entry]
fn main() -> ! {
    if let Some(board) = Board::take() {
        let mut timer = Timer::new(board.TIMER0);
        let mut display = Display::new(board.display_pins);
        
        let heart = [
            [0, 1, 0, 1, 0],
            [1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1],
            [0, 1, 1, 1, 0],
            [0, 0, 1, 0, 0],
        ];
        
        loop {
            display.show(&mut timer, heart, 1000);
        }
    }
    
    loop {}
}


