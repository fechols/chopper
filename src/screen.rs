//! Host model of the VIC-II multicolor bitmap the C64 displays: 25 bands of
//! 40 cells, each with 8 bitmap bytes, a screen byte and a colour nibble,
//! over one background colour. Previews decode exactly these bytes, so they
//! show the per-cell colour limits the real machine has.

use crate::render::{BAND_PIXELS, BANDS, PALETTE_RGB, Renderer, VIEW_BANDS, WIDTH, pack_band};

pub struct Screen {
    pub background: u8,
    pub bitmap: [u8; 8000],
    pub screen: [u8; 1000],
    pub colors: [u8; 1000],
}

impl Default for Screen {
    fn default() -> Self {
        Self::new()
    }
}

impl Screen {
    pub fn new() -> Self {
        Self { background: 0, bitmap: [0; 8000], screen: [0; 1000], colors: [0; 1000] }
    }

    /// Pack one 160x8 pixel band into band `band`, keeping slot colours from
    /// what the band held before (as the target does between frames).
    pub fn pack(&mut self, band: usize, pixels: &[u8; BAND_PIXELS]) {
        let cells = band * 40..band * 40 + 40;
        let previous_screen: [u8; 40] = self.screen[cells.clone()].try_into().unwrap();
        let previous_color: [u8; 40] = self.colors[cells.clone()].try_into().unwrap();
        let mut bitmap = [0u8; 320];
        let mut screen = [0u8; 40];
        let mut colors = [0u8; 40];
        pack_band(pixels, &previous_screen, &previous_color, self.background, &mut bitmap, &mut screen, &mut colors);
        self.bitmap[band * 320..band * 320 + 320].copy_from_slice(&bitmap);
        self.screen[cells.clone()].copy_from_slice(&screen);
        self.colors[cells].copy_from_slice(&colors);
    }

    /// Rasterize the sorted renderer into the view bands. The frame's clear
    /// colour becomes the background, as on the target.
    pub fn draw_view(&mut self, renderer: &Renderer, mut overlay: impl FnMut(u8, &mut [u8; BAND_PIXELS])) {
        self.background = renderer.clear_color();
        let mut scratch = std::boxed::Box::new(crate::render::RasterScratch::new());
        let mut pass = renderer.raster_pass(&mut scratch);
        let mut pixels = [0u8; BAND_PIXELS];
        while let Some(band) = pass.next_band(&mut pixels) {
            overlay(band, &mut pixels);
            self.pack(band as usize, &pixels);
        }
    }

    /// Draw the bands below the view (the cockpit) from a band painter.
    pub fn draw_panel(&mut self, mut paint: impl FnMut(u8, &mut [u8; BAND_PIXELS])) {
        let mut pixels = [0u8; BAND_PIXELS];
        for band in VIEW_BANDS..BANDS {
            pixels.fill(self.background);
            paint(band as u8, &mut pixels);
            self.pack(band, &pixels);
        }
    }

    /// Palette index of logical pixel (x, y), decoded from the cell bytes.
    pub fn pixel(&self, x: usize, y: usize) -> u8 {
        let cell = (y / 8) * 40 + x / 4;
        let byte = self.bitmap[cell * 8 + (y & 7)];
        match (byte >> (6 - 2 * (x & 3))) & 3 {
            0 => self.background,
            1 => self.screen[cell] >> 4,
            2 => self.screen[cell] & 15,
            _ => self.colors[cell] & 15,
        }
    }

    /// 160x200 RGB pixels (each to be shown double width).
    pub fn rgb(&self) -> std::vec::Vec<[u8; 3]> {
        let mut out = std::vec::Vec::with_capacity(WIDTH * 200);
        for y in 0..200 {
            for x in 0..WIDTH {
                out.push(PALETTE_RGB[self.pixel(x, y) as usize]);
            }
        }
        out
    }

    pub fn save_png(&self, path: &std::path::Path) -> Result<(), std::string::String> {
        crate::png::save_wide(path, WIDTH, &self.rgb())
    }
}
