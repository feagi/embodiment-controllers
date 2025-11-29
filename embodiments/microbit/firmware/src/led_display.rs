//! LED matrix display control for micro:bit 5×5 LED grid

use embassy_time::Duration;
use heapless::Vec;

// Use type inference - the display type will be inferred from board.display
// We'll use a generic type parameter and let Rust infer it
pub struct LedDisplay<'a, D> {
    display: &'a mut D,
    buffer: [[u8; 5]; 5],
}

impl<'a, D> LedDisplay<'a, D> {
    pub fn new(display: &'a mut D) -> Self {
        Self {
            display,
            buffer: [[0; 5]; 5],
        }
    }
    
    pub fn clear(&mut self) {
        self.buffer = [[0; 5]; 5];
        // microbit-bsp display doesn't have clear(), we'll just clear the buffer
    }
    
    pub fn set_pixel(&mut self, x: usize, y: usize, on: bool) {
        if x < 5 && y < 5 {
            self.buffer[y][x] = if on { 255 } else { 0 };
        }
    }
    
    pub fn set_brightness(&mut self, x: usize, y: usize, brightness: u8) {
        if x < 5 && y < 5 {
            self.buffer[y][x] = brightness;
        }
    }
    
    pub fn set_matrix(&mut self, data: &[u8; 25]) {
        // data is 25 bytes in row-major order
        for (i, &brightness) in data.iter().enumerate() {
            let y = i / 5;
            let x = i % 5;
            self.buffer[y][x] = brightness;
        }
    }
    
    pub async fn show(&mut self)
    where
        D: DisplayTrait,
    {
        // Convert buffer to LED matrix format (on/off, no brightness for now)
        let mut image = [[0u8; 5]; 5];
        for y in 0..5 {
            for x in 0..5 {
                image[y][x] = if self.buffer[y][x] > 127 { 1 } else { 0 };
            }
        }
        
        // Show image using microbit-bsp display API (async)
        // Based on microbit-bsp examples, we use Frame and Bitmap
        use microbit_bsp::display::{Frame, Bitmap};
        
        // Create bitmap and set pixels
        let mut bitmap = Bitmap::new(5, 5);
        for y in 0..5 {
            for x in 0..5 {
                if image[y][x] > 0 {
                    bitmap.set(x, y);
                }
            }
        }
        
        // Frame::new takes an array of bitmaps
        let frame = Frame::new([bitmap]);
        DisplayTrait::display(self.display, &frame, Duration::from_millis(30)).await;
    }
    
    pub fn show_heart(&mut self) {
        self.buffer = [
            [0, 255, 0, 255, 0],
            [255, 255, 255, 255, 255],
            [255, 255, 255, 255, 255],
            [0, 255, 255, 255, 0],
            [0, 0, 255, 0, 0],
        ];
    }
    
    pub fn show_letter_f(&mut self) {
        // Letter "F"
        self.buffer = [
            [255, 255, 255, 255, 255],
            [255, 0, 0, 0, 0],
            [255, 255, 255, 255, 0],
            [255, 0, 0, 0, 0],
            [255, 0, 0, 0, 0],
        ];
    }
    
    pub fn show_letter_e(&mut self) {
        // Letter "E"
        self.buffer = [
            [255, 255, 255, 255, 255],
            [255, 0, 0, 0, 0],
            [255, 255, 255, 255, 0],
            [255, 0, 0, 0, 0],
            [255, 255, 255, 255, 255],
        ];
    }
    
    pub fn show_letter_a(&mut self) {
        // Letter "A"
        self.buffer = [
            [0, 255, 255, 255, 0],
            [255, 0, 0, 0, 255],
            [255, 255, 255, 255, 255],
            [255, 0, 0, 0, 255],
            [255, 0, 0, 0, 255],
        ];
    }
    
    pub fn show_letter_g(&mut self) {
        // Letter "G"
        self.buffer = [
            [0, 255, 255, 255, 0],
            [255, 0, 0, 0, 0],
            [255, 0, 255, 255, 255],
            [255, 0, 0, 0, 255],
            [0, 255, 255, 255, 0],
        ];
    }
    
    pub fn show_letter_i(&mut self) {
        // Letter "I"
        self.buffer = [
            [255, 255, 255, 255, 255],
            [0, 0, 255, 0, 0],
            [0, 0, 255, 0, 0],
            [0, 0, 255, 0, 0],
            [255, 255, 255, 255, 255],
        ];
    }
    
    pub fn show_arrow_up(&mut self) {
        self.buffer = [
            [0, 0, 255, 0, 0],
            [0, 255, 255, 255, 0],
            [255, 0, 255, 0, 255],
            [0, 0, 255, 0, 0],
            [0, 0, 255, 0, 0],
        ];
    }
    
    pub fn show_checkmark(&mut self) {
        self.buffer = [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 255],
            [0, 0, 0, 255, 0],
            [255, 0, 255, 0, 0],
            [0, 255, 0, 0, 0],
        ];
    }
    
    /// Update LEDs based on neuron firing coordinates from cortical area
    /// 
    /// **FEAGI Cortical Area Standard:**
    /// - Type: `omis` (Miscellaneous Motor)
    /// - Name: "LED Matrix" or "Display Matrix"
    /// - Dimensions: 5×5×1
    /// 
    /// Coordinates are (x, y) where x and y are 0-4 (5x5 matrix)
    /// This maps directly: cortical area neuron at (x, y) -> LED at (x, y)
    pub fn update_from_neurons(&mut self, coordinates: &Vec<(u8, u8), 25>) {
        // Clear display first
        self.clear();
        
        // Set LEDs for each fired neuron
        for &(x, y) in coordinates.iter() {
            if x < 5 && y < 5 {
                self.set_pixel(x as usize, y as usize, true);
            }
        }
    }
}

// Trait to abstract the display API
trait DisplayTrait {
    async fn display(&mut self, frame: &microbit_bsp::display::Frame, duration: embassy_time::Duration);
}

// Implement for LedMatrix - we'll use a blanket impl with type inference
// The actual LedMatrix type from board.display will be inferred
impl<W, H, const N: usize> DisplayTrait for microbit_bsp::display::LedMatrix<W, H, N> 
where
    W: microbit_bsp::display::Width,
    H: microbit_bsp::display::Height,
{
    async fn display(&mut self, frame: &microbit_bsp::display::Frame, duration: embassy_time::Duration) {
        // LedMatrix::display takes &self, frame, and duration
        self.display(frame, duration).await;
    }
}
