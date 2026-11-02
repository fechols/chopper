//! Allocation-free, flat-shaded triangle renderer for the VIC-II multicolor
//! bitmap, ported from H.A.V.O.C. (`sueme64/src/render.rs`, host reference
//! paths). The arithmetic is kept identical so H.A.V.O.C.'s native kernels can
//! be adapted later; the H.A.V.O.C.-specific scenery staging, shadows, space
//! dust and victory fade are gone.
//!
//! Camera coordinates are integer world units, +X right, +Y up, +Z forward.
//! A face whose projected corners run clockwise on screen is front-facing.
//! Only the top `VIEW_BANDS` bands hold the 3D view; the cockpit owns the rest.
//!
//! Band pixels carry the VIC colour in bits 0-3. Bit 7 marks HUD ink that wins
//! cell quantization. Bit 6 remembers coverage by terrain-protected geometry
//! (`TERRAIN_CLEAR_GEOMETRY`): later `EXTERIOR_TERRAIN` spans cannot erase it,
//! so an object standing on a slope is never cut by the slope's own triangles.

use crate::projection::project;
use crate::{EXTERIOR_TERRAIN, GeometrySink, TERRAIN_CLEAR_GEOMETRY, Vec3};

pub const WIDTH: usize = 160;
pub const HEIGHT: usize = 200;
pub const BAND_HEIGHT: usize = 8;
pub const BANDS: usize = HEIGHT / BAND_HEIGHT;
/// Bands rasterized by the 3D view; the cockpit panel owns the rest.
pub const VIEW_BANDS: usize = 17;
pub const VIEW_HEIGHT: usize = VIEW_BANDS * BAND_HEIGHT;
pub const BAND_PIXELS: usize = WIDTH * BAND_HEIGHT;
pub const TRIANGLE_CAPACITY: usize = 256;
/// Near clipping plane. Vertices are visible when |x| < z and |y| < 0.51 z.
pub const NEAR: i16 = 8;
pub const HUD_INK: u8 = 0x80;
pub(crate) const TERRAIN_CLEAR_INK: u8 = 0x40;
const KEPT_FLAGS: u8 = TERRAIN_CLEAR_GEOMETRY | EXTERIOR_TERRAIN;

/// An append-only geometry transaction, valid before sorting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Checkpoint {
    pub triangle_count: u16,
    pub dropped: u16,
}

/// One projected command: 16 bytes, as on the target.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct Triangle {
    x: [i16; 3],
    y: [i16; 3],
    depth: i16,
    color: u8,
    reserved: u8,
}
const _: () = assert!(core::mem::size_of::<Triangle>() == 16);

impl Triangle {
    const EMPTY: Self = Self { x: [0; 3], y: [0; 3], depth: 0, color: 0, reserved: 0 };
}

/// The frame's command list. `finish` sorts far to near (stable) into `order`.
pub struct Renderer {
    pub triangle_count: u16,
    pub dropped: u16,
    clear_color: u8,
    sorted: bool,
    triangles: [Triangle; TRIANGLE_CAPACITY],
    order: [u8; TRIANGLE_CAPACITY],
}

impl Default for Renderer {
    fn default() -> Self {
        Self::new()
    }
}

impl Renderer {
    pub const fn new() -> Self {
        Self {
            triangle_count: 0,
            dropped: 0,
            clear_color: 0,
            sorted: false,
            triangles: [Triangle::EMPTY; TRIANGLE_CAPACITY],
            order: [0; TRIANGLE_CAPACITY],
        }
    }

    pub fn begin(&mut self, clear_color: u8) {
        self.triangle_count = 0;
        self.dropped = 0;
        self.clear_color = clear_color & 15;
        self.sorted = false;
    }

    pub fn clear_color(&self) -> u8 {
        self.clear_color
    }

    pub fn checkpoint(&self) -> Checkpoint {
        Checkpoint { triangle_count: self.triangle_count, dropped: self.dropped }
    }

    pub fn rollback(&mut self, checkpoint: Checkpoint) {
        assert!(!self.sorted, "cannot roll back sorted commands");
        assert!(checkpoint.triangle_count <= self.triangle_count);
        self.triangle_count = checkpoint.triangle_count;
        self.dropped = checkpoint.dropped;
    }

    pub fn remaining_commands(&self) -> u16 {
        TRIANGLE_CAPACITY as u16 - self.triangle_count
    }

    /// Stable far-to-near sort: ties keep their emission order. Two 8-bit LSD
    /// radix passes over the key `!(depth ^ 0x8000)`, writing an index order
    /// like the native kernel (commands stay where they were emitted).
    pub fn finish(&mut self) {
        let count = self.triangle_count as usize;
        let key = |t: &Triangle| !(t.depth as u16 ^ 0x8000);
        let mut first = [0u8; TRIANGLE_CAPACITY];
        for (i, slot) in first.iter_mut().enumerate().take(count) {
            *slot = i as u8;
        }
        let mut second = [0u8; TRIANGLE_CAPACITY];
        for shift in [0u32, 8] {
            let mut counts = [0usize; 257];
            for &index in &first[..count] {
                counts[((key(&self.triangles[index as usize]) >> shift) & 255) as usize + 1] += 1;
            }
            for i in 0..256 {
                counts[i + 1] += counts[i];
            }
            for &index in &first[..count] {
                let digit = ((key(&self.triangles[index as usize]) >> shift) & 255) as usize;
                second[counts[digit]] = index;
                counts[digit] += 1;
            }
            core::mem::swap(&mut first, &mut second);
        }
        self.order = first;
        self.sorted = true;
    }

    /// Prepare every sorted command's edges for the band-by-band raster pass.
    /// The 7.6 KB of edge state lives in caller-owned `scratch` (a static on
    /// the C64, where the software stack is small).
    pub fn raster_pass<'a>(&'a self, scratch: &'a mut RasterScratch) -> RasterPass<'a> {
        assert!(self.sorted || self.triangle_count == 0, "finish() before rasterizing");
        scratch.masks = [[0; 32]; VIEW_BANDS];
        for position in 0..self.triangle_count as usize {
            let source = if self.sorted { self.order[position] as usize } else { position };
            let (head, tail) = prepare_triangle(&self.triangles[source]);
            scratch.heads[position] = head;
            scratch.tails[position] = tail;
            if tail.top < tail.bottom {
                for band in tail.top / 8..=(tail.bottom - 1) / 8 {
                    scratch.masks[band as usize][position / 8] |= 1 << (position & 7);
                }
            }
        }
        RasterPass { renderer: self, next_band: 0, scratch }
    }

    /// Near-plane classification, Sutherland–Hodgman clipping against
    /// z = NEAR and projection of the pieces.
    fn triangle_reference(&mut self, a: Vec3, b: Vec3, c: Vec3, color: u8, flags: u8) {
        if a.z >= NEAR && b.z >= NEAR && c.z >= NEAR {
            self.project_triangle(a, b, c, color, flags);
            return;
        }
        if a.z < NEAR && b.z < NEAR && c.z < NEAR {
            return;
        }
        let input = [a, b, c];
        let mut polygon = [Vec3::ZERO; 4];
        let mut len = 0;
        let mut previous = c;
        for current in input {
            if (previous.z >= NEAR) != (current.z >= NEAR) {
                polygon[len] = near_intersection(previous, current);
                len += 1;
            }
            if current.z >= NEAR {
                polygon[len] = current;
                len += 1;
            }
            previous = current;
        }
        if len < 3 {
            return;
        }
        self.project_triangle(polygon[0], polygon[1], polygon[2], color, flags);
        if len == 4 {
            self.project_triangle(polygon[0], polygon[2], polygon[3], color, flags);
        }
    }

    /// A convex screen-space polygon painted behind everything else (the
    /// ground below the horizon). Vertices must run clockwise on screen.
    pub fn backdrop(&mut self, points: &[(i16, i16)], color: u8) {
        for k in 1..points.len().saturating_sub(1) {
            if self.triangle_count as usize == TRIANGLE_CAPACITY {
                self.dropped = self.dropped.saturating_add(1);
                return;
            }
            let [a, b, c] = [points[0], points[k], points[k + 1]];
            let area = (b.0 as i32 - a.0 as i32) * (c.1 as i32 - a.1 as i32) - (b.1 as i32 - a.1 as i32) * (c.0 as i32 - a.0 as i32);
            if area <= 0 {
                continue;
            }
            self.triangles[self.triangle_count as usize] =
                Triangle { x: [a.0, b.0, c.0], y: [a.1, b.1, c.1], depth: i16::MAX, color: color & 15, reserved: 0 };
            self.triangle_count += 1;
        }
    }

    fn project_triangle(&mut self, a: Vec3, b: Vec3, c: Vec3, color: u8, flags: u8) {
        let mut triangle = Triangle::EMPTY;
        let (ax, ay) = project(a);
        let (bx, by) = project(b);
        let (cx, cy) = project(c);
        triangle.x = [ax, bx, cx];
        triangle.y = [ay, by, cy];
        let area = (triangle.x[1] as i32 - triangle.x[0] as i32)
            * (triangle.y[2] as i32 - triangle.y[0] as i32)
            - (triangle.y[1] as i32 - triangle.y[0] as i32) * (triangle.x[2] as i32 - triangle.x[0] as i32);
        if area <= 0 {
            return;
        }
        if triangle.x.iter().all(|&x| x <= 0)
            || triangle.x.iter().all(|&x| x >= WIDTH as i16)
            || triangle.y.iter().all(|&y| y <= 0)
            || triangle.y.iter().all(|&y| y >= VIEW_HEIGHT as i16)
        {
            return;
        }
        if self.triangle_count as usize == TRIANGLE_CAPACITY {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        // Clipping leaves positive i16 depths, so their average fits in i16.
        triangle.depth = crate::math::div_u32_u16(a.z as u32 + b.z as u32 + c.z as u32, 3) as i16;
        // Terrain sorts as if slightly farther away, so objects resting on a
        // tile are painted after it.
        if flags & EXTERIOR_TERRAIN != 0 {
            triangle.depth = triangle.depth.saturating_add(TERRAIN_DEPTH_BIAS);
        }
        triangle.color = color & 15;
        triangle.reserved = flags & KEPT_FLAGS;
        self.triangles[self.triangle_count as usize] = triangle;
        self.triangle_count += 1;
    }
}

/// Depth added to exterior-terrain commands before sorting.
pub const TERRAIN_DEPTH_BIAS: i16 = 32;

impl GeometrySink for Renderer {
    fn checkpoint(&self) -> Option<Checkpoint> {
        Some(Renderer::checkpoint(self))
    }
    fn rollback(&mut self, checkpoint: Checkpoint) {
        Renderer::rollback(self, checkpoint);
    }
    fn triangle(&mut self, a: Vec3, b: Vec3, c: Vec3, color: u8, flags: u8) {
        assert!(!self.sorted, "begin a new frame before geometry");
        self.triangle_reference(a, b, c, color, flags);
    }
}

fn near_intersection(a: Vec3, b: Vec3) -> Vec3 {
    // Both owners of a shared edge must round the same intersection, even
    // when their winding traverses that edge in opposite directions.
    let (a, b) = if a.z < b.z { (a, b) } else { (b, a) };
    let t = crate::math::div_i32((NEAR as i32 - a.z as i32) << 15, b.z as i32 - a.z as i32);
    Vec3 {
        x: (a.x as i32 + (((b.x as i32 - a.x as i32) * t) >> 15)) as i16,
        y: (a.y as i32 + (((b.y as i32 - a.y as i32) * t) >> 15)) as i16,
        z: NEAR,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RasterHead {
    long_x: i32,
    long_step: i32,
    short_x: i32,
    short_step: i32,
}
impl RasterHead {
    const EMPTY: Self = Self { long_x: 0, long_step: 0, short_x: 0, short_step: 0 };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RasterTail {
    lower_x: i32,
    lower_step: i32,
    top: u8,
    middle: u8,
    bottom: u8,
    color_flags: u8,
}
impl RasterTail {
    const EMPTY: Self = Self { lower_x: 0, lower_step: 0, top: 0, middle: 0, bottom: 0, color_flags: 0 };
}

/// Edge state for one raster pass: persistent heads, tails and per-band masks.
pub struct RasterScratch {
    heads: [RasterHead; TRIANGLE_CAPACITY],
    tails: [RasterTail; TRIANGLE_CAPACITY],
    masks: [[u8; 32]; VIEW_BANDS],
}

impl Default for RasterScratch {
    fn default() -> Self {
        Self::new()
    }
}

impl RasterScratch {
    pub const fn new() -> Self {
        Self {
            heads: [RasterHead::EMPTY; TRIANGLE_CAPACITY],
            tails: [RasterTail::EMPTY; TRIANGLE_CAPACITY],
            masks: [[0; 32]; VIEW_BANDS],
        }
    }
}

/// Sequential band raster over sorted, prepared commands. Edges step
/// continuously from band to band and are never recomputed.
pub struct RasterPass<'a> {
    renderer: &'a Renderer,
    next_band: u8,
    scratch: &'a mut RasterScratch,
}

impl RasterPass<'_> {
    /// Paint the next view band into `pixels` (cleared to the frame's clear
    /// colour first). Returns the band index, or `None` after the last one.
    pub fn next_band(&mut self, pixels: &mut [u8; BAND_PIXELS]) -> Option<u8> {
        let band = self.next_band;
        if band as usize >= VIEW_BANDS {
            return None;
        }
        self.next_band += 1;
        pixels.fill(self.renderer.clear_color);
        let mut coverage_seen = false;
        for byte in 0..32usize {
            let mut mask = self.scratch.masks[band as usize][byte];
            let mut bit = 0;
            while mask != 0 {
                if mask & 1 != 0 {
                    let index = byte * 8 + bit;
                    let tail = self.scratch.tails[index];
                    raster_rows(&mut self.scratch.heads[index], &tail, band * 8, |row, first, end, flags| {
                        paint_reference_span(pixels, row, first, end, flags, &mut coverage_seen);
                    });
                }
                mask >>= 1;
                bit += 1;
            }
        }
        Some(band)
    }
}

// Projected coordinates are within +/-8191. dx*65536 and dx*t*2 fit i32.
fn prepare_edge(ax: i16, ay: i16, bx: i16, by: i16) -> (i32, i32) {
    if ay == by || by <= 0 || ay >= VIEW_HEIGHT as i16 {
        return (0, 0);
    }
    let dx = bx as i32 - ax as i32;
    let dy = by as i32 - ay as i32;
    let start = ay.max(0) as i32;
    let numerator = ((start - ay as i32) * 2 + 1) << 14;
    let t = crate::math::div_u32_u16(numerator as u32, dy as u16) as i32;
    ((ax as i32 * 65536) + dx * t * 2, crate::math::div_i32(dx * 65536, dy))
}

#[derive(Clone, Copy)]
struct Point2 {
    x: i16,
    y: i16,
}

fn sort_pair(a: &mut Point2, b: &mut Point2) -> u8 {
    if a.y > b.y || (a.y == b.y && a.x > b.x) {
        core::mem::swap(a, b);
        16
    } else {
        0
    }
}

fn prepare_triangle(triangle: &Triangle) -> (RasterHead, RasterTail) {
    let mut head = RasterHead::EMPTY;
    let mut tail = RasterTail::EMPTY;
    let mut a = Point2 { x: triangle.x[0], y: triangle.y[0] };
    let mut b = Point2 { x: triangle.x[1], y: triangle.y[1] };
    let mut c = Point2 { x: triangle.x[2], y: triangle.y[2] };
    let mut orientation = 16;
    orientation ^= sort_pair(&mut a, &mut b);
    orientation ^= sort_pair(&mut b, &mut c);
    orientation ^= sort_pair(&mut a, &mut b);
    let h = VIEW_HEIGHT as i16;
    tail.top = a.y.clamp(0, h) as u8;
    tail.middle = b.y.clamp(0, h) as u8;
    tail.bottom = c.y.clamp(0, h) as u8;
    tail.color_flags = triangle.color | orientation | (triangle.reserved & KEPT_FLAGS);
    (head.long_x, head.long_step) = prepare_edge(a.x, a.y, c.x, c.y);
    (head.short_x, head.short_step) = prepare_edge(a.x, a.y, b.x, b.y);
    (tail.lower_x, tail.lower_step) = prepare_edge(b.x, b.y, c.x, c.y);
    (head, tail)
}

/// Walk a triangle's rows within one band, reporting every nonempty span as
/// (band row, first, end, color_flags) in top-to-bottom order.
fn raster_rows(head: &mut RasterHead, tail: &RasterTail, band_y: u8, mut paint: impl FnMut(usize, usize, usize, u8)) {
    for y in tail.top.max(band_y)..tail.bottom.min(band_y + 8) {
        if y == tail.middle {
            head.short_x = tail.lower_x;
            head.short_step = tail.lower_step;
        }
        let (left, right) = if tail.color_flags & 16 != 0 {
            (head.long_x, head.short_x)
        } else {
            (head.short_x, head.long_x)
        };
        let first = ((left + 32767) >> 16).clamp(0, WIDTH as i32) as usize;
        let end = ((right + 32767) >> 16).clamp(0, WIDTH as i32) as usize;
        if first < end {
            paint((y - band_y) as usize, first, end, tail.color_flags);
        }
        head.long_x += head.long_step;
        head.short_x += head.short_step;
    }
}

fn paint_reference_span(
    pixels: &mut [u8; BAND_PIXELS],
    row: usize,
    first: usize,
    end: usize,
    color_flags: u8,
    coverage_seen: &mut bool,
) {
    let offset = row * WIDTH;
    let span = &mut pixels[offset + first..offset + end];
    let color = color_flags & 15;
    if color_flags & TERRAIN_CLEAR_GEOMETRY != 0 {
        *coverage_seen = true;
        span.fill(color | TERRAIN_CLEAR_INK);
    } else if !*coverage_seen {
        span.fill(color);
    } else if color_flags & EXTERIOR_TERRAIN != 0 {
        for pixel in span {
            if *pixel & TERRAIN_CLEAR_INK == 0 {
                *pixel = color;
            }
        }
    } else {
        for pixel in span {
            *pixel = (*pixel & TERRAIN_CLEAR_INK) | color;
        }
    }
}

/// Approximate VIC-II palette for host previews and quantization distances.
pub const PALETTE_RGB: [[u8; 3]; 16] = [
    [0, 0, 0],
    [255, 255, 255],
    [136, 0, 0],
    [170, 255, 238],
    [204, 68, 204],
    [0, 204, 85],
    [0, 0, 170],
    [238, 238, 119],
    [221, 136, 85],
    [102, 68, 0],
    [255, 119, 119],
    [51, 51, 51],
    [119, 119, 119],
    [170, 255, 102],
    [0, 136, 255],
    [187, 187, 187],
];

const fn color_distances() -> [[u16; 16]; 16] {
    let mut result = [[0; 16]; 16];
    let mut a = 0;
    while a < 16 {
        let mut b = 0;
        while b < 16 {
            let r = PALETTE_RGB[a][0] as i32 - PALETTE_RGB[b][0] as i32;
            let g = PALETTE_RGB[a][1] as i32 - PALETTE_RGB[b][1] as i32;
            let blue = PALETTE_RGB[a][2] as i32 - PALETTE_RGB[b][2] as i32;
            result[a][b] = ((r * r + 2 * g * g + blue * blue) / 4) as u16;
            b += 1;
        }
        a += 1;
    }
    result
}

static COLOR_DISTANCE: [[u16; 16]; 16] = color_distances();

/// Convert one 160x8 band into forty VIC-II multicolor cells. The global
/// background (`$D021`) shows `background`; each cell supplies screen
/// high/low nibbles and one color RAM nibble for its other colours.
pub fn pack_band(
    pixels: &[u8; BAND_PIXELS],
    previous_screen: &[u8; 40],
    previous_color: &[u8; 40],
    background: u8,
    bitmap: &mut [u8; 320],
    screen: &mut [u8; 40],
    colors: &mut [u8; 40],
) {
    for cell in 0..40 {
        let (cell_bitmap, cell_screen, cell_color) = pack_cell(
            pixels,
            cell,
            [previous_screen[cell] >> 4, previous_screen[cell] & 15, previous_color[cell] & 15],
            background,
        );
        bitmap[cell * 8..cell * 8 + 8].copy_from_slice(&cell_bitmap);
        screen[cell] = cell_screen;
        colors[cell] = cell_color;
    }
}

/// Quantize one 4x8 cell to the background plus at most three colours,
/// preferring HUD ink and the cell's previous slot colours.
pub fn pack_cell(pixels: &[u8; BAND_PIXELS], cell: usize, old: [u8; 3], background: u8) -> ([u8; 8], u8, u8) {
    let background = background & 15;
    let mut bitmap = [0u8; 8];
    let mut counts = [0u8; 16];
    let mut hud = 0u16;
    let mut present = 0u16;
    for row in 0..8 {
        for column in 0..4 {
            let pixel = pixels[row * WIDTH + cell * 4 + column];
            counts[(pixel & 15) as usize] += 1;
            present |= 1 << (pixel & 15);
            if pixel & HUD_INK != 0 {
                hud |= 1 << (pixel & 15);
            }
        }
    }
    if present & present.wrapping_sub(1) == 0 {
        let color = pixels[cell * 4] & 15;
        let mut slot = 0;
        if color != background {
            slot = 1;
            for i in 0..3 {
                if old[i] == color {
                    slot = i + 1;
                    break;
                }
            }
        }
        bitmap.fill(slot as u8 * 0x55);
        let mut palette = [background; 4];
        palette[slot] = color;
        return (bitmap, (palette[1] << 4) | palette[2], palette[3]);
    }
    let mut selected = [background; 3];
    let mut selected_bits = 0u16;
    for chosen in &mut selected {
        let mut best = background;
        let mut best_score = 0;
        for color in 0..16 {
            if color == background as usize || counts[color] == 0 || selected_bits & (1 << color) != 0 {
                continue;
            }
            let score = counts[color] as u16 * 4
                + if old.contains(&(color as u8)) { 1 } else { 0 }
                + if hud & (1 << color) != 0 { 256 } else { 0 };
            if score > best_score {
                best = color as u8;
                best_score = score;
            }
        }
        *chosen = best;
        if best != background {
            selected_bits |= 1 << best;
        }
    }
    let mut palette = [background; 4];
    let mut assigned = 0u16;
    for slot in 0..3 {
        if selected_bits & (1 << old[slot]) != 0 && assigned & (1 << old[slot]) == 0 {
            palette[slot + 1] = old[slot];
            assigned |= 1 << old[slot];
        }
    }
    for chosen in selected {
        if chosen == background || assigned & (1 << chosen) != 0 {
            continue;
        }
        for slot in &mut palette[1..] {
            if *slot == background {
                *slot = chosen;
                assigned |= 1 << chosen;
                break;
            }
        }
    }
    let mut map = [0u8; 16];
    for color in 0..16 {
        if color == background as usize || counts[color] == 0 {
            continue;
        }
        let mut best_distance = COLOR_DISTANCE[color][background as usize];
        for slot in 1..4 {
            let distance = COLOR_DISTANCE[color][palette[slot] as usize];
            if distance < best_distance {
                map[color] = slot as u8;
                best_distance = distance;
            }
        }
    }
    for (row, byte) in bitmap.iter_mut().enumerate() {
        let mut packed = 0;
        for column in 0..4 {
            packed = (packed << 2) | map[(pixels[row * WIDTH + cell * 4 + column] & 15) as usize];
        }
        *byte = packed;
    }
    (bitmap, (palette[1] << 4) | palette[2], palette[3])
}

/// A band-local drawing surface: eight rows of 160 pixels. Colours carry the
/// VIC colour in bits 0-3 and `HUD_INK` in bit 7.
pub trait Canvas {
    fn span(&mut self, row: usize, first: usize, end: usize, color: u8);
    fn plot(&mut self, row: usize, x: usize, color: u8);
}

impl Canvas for [u8; BAND_PIXELS] {
    fn span(&mut self, row: usize, first: usize, end: usize, color: u8) {
        self[row * WIDTH + first..row * WIDTH + end].fill(color);
    }
    fn plot(&mut self, row: usize, x: usize, color: u8) {
        self[row * WIDTH + x] = color;
    }
}

/// Draw the compact 3x5 font (one-pixel gap) at screen position (x, y),
/// clipped to `band`, with integer `scale`.
#[allow(clippy::too_many_arguments)]
pub fn draw_text(band: u8, canvas: &mut impl Canvas, x: i16, y: i16, text: &[u8], color: u8, scale: u8) {
    let scale = scale.clamp(1, 4) as i16;
    let band_y = band as i16 * 8;
    if band as usize >= BANDS || y >= band_y + 8 || y.saturating_add(5 * scale) <= band_y {
        return;
    }
    let ink = (color & 15) | HUD_INK;
    let mut cursor = x;
    for &character in text {
        let glyph = glyph(character);
        let mut glyph_y = y;
        for &bits in glyph {
            for dy in 0..scale {
                let screen_y = glyph_y + dy;
                if screen_y < band_y || screen_y >= band_y + 8 {
                    continue;
                }
                let mut glyph_x = cursor;
                for mask in [4, 2, 1] {
                    if bits & mask != 0 {
                        let first = glyph_x.clamp(0, WIDTH as i16) as usize;
                        let end = glyph_x.saturating_add(scale).clamp(0, WIDTH as i16) as usize;
                        if first < end {
                            canvas.span((screen_y - band_y) as usize, first, end, ink);
                        }
                    }
                    glyph_x = glyph_x.saturating_add(scale);
                }
            }
            glyph_y += scale;
        }
        cursor = cursor.saturating_add(4 * scale);
        if cursor >= WIDTH as i16 {
            break;
        }
    }
}

/// A horizontal gauge bar: `value/maximum` of `width` in `color`, the rest dark gray.
#[allow(clippy::too_many_arguments)]
pub fn draw_bar(band: u8, canvas: &mut impl Canvas, x: i16, y: i16, width: u8, height: u8, value: u16, maximum: u16, color: u8) {
    let band_y = band as i16 * 8;
    if band as usize >= BANDS || y >= band_y + 8 || y.saturating_add(height as i16) <= band_y {
        return;
    }
    let fill = if maximum == 0 { 0 } else { (width as u32 * value.min(maximum) as u32 / maximum as u32) as i16 };
    let left = x.clamp(0, WIDTH as i16) as usize;
    let right = x.saturating_add(width as i16).clamp(0, WIDTH as i16) as usize;
    if left >= right {
        return;
    }
    let split = x.saturating_add(fill).clamp(left as i16, right as i16) as usize;
    for row in y..y.saturating_add(height as i16) {
        if row < band_y || row >= band_y + 8 {
            continue;
        }
        let band_row = (row - band_y) as usize;
        if left < split {
            canvas.span(band_row, left, split, (color & 15) | HUD_INK);
        }
        if split < right {
            canvas.span(band_row, split, right, 11 | HUD_INK);
        }
    }
}

static FONT: [[u8; 5]; 43] = [
    [2, 5, 7, 5, 5],
    [6, 5, 6, 5, 6],
    [3, 4, 4, 4, 3],
    [6, 5, 5, 5, 6],
    [7, 4, 6, 4, 7],
    [7, 4, 6, 4, 4],
    [3, 4, 5, 5, 3],
    [5, 5, 7, 5, 5],
    [7, 2, 2, 2, 7],
    [1, 1, 1, 5, 2],
    [5, 5, 6, 5, 5],
    [4, 4, 4, 4, 7],
    [5, 7, 7, 5, 5],
    [5, 7, 7, 7, 5],
    [2, 5, 5, 5, 2],
    [6, 5, 6, 4, 4],
    [2, 5, 5, 3, 1],
    [6, 5, 6, 5, 5],
    [3, 4, 2, 1, 6],
    [7, 2, 2, 2, 2],
    [5, 5, 5, 5, 7],
    [5, 5, 5, 5, 2],
    [5, 5, 7, 7, 5],
    [5, 5, 2, 5, 5],
    [5, 5, 2, 2, 2],
    [7, 1, 2, 4, 7],
    [7, 5, 5, 5, 7],
    [2, 6, 2, 2, 7],
    [6, 1, 2, 4, 7],
    [6, 1, 2, 1, 6],
    [5, 5, 7, 1, 1],
    [7, 4, 6, 1, 6],
    [3, 4, 6, 5, 2],
    [7, 1, 2, 2, 2],
    [2, 5, 2, 5, 2],
    [2, 5, 3, 1, 6],
    [0, 2, 0, 2, 0],
    [0, 0, 7, 0, 0],
    [1, 1, 2, 4, 4],
    [2, 2, 2, 0, 2],
    [0, 0, 0, 0, 2],
    [0, 2, 7, 2, 0],
    [0; 5],
];

pub(crate) fn glyph(c: u8) -> &'static [u8; 5] {
    let index = match c {
        b'A'..=b'Z' => c - b'A',
        b'a'..=b'z' => c - b'a',
        b'0'..=b'9' => c - b'0' + 26,
        b':' => 36,
        b'-' => 37,
        b'/' => 38,
        b'!' => 39,
        b'.' => 40,
        b'+' => 41,
        _ => 42,
    };
    &FONT[index as usize]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(renderer: &mut Renderer, z: i16, color: u8) {
        renderer.triangle(
            Vec3 { x: -z / 2, y: -z / 4, z },
            Vec3 { x: 0, y: z / 4, z },
            Vec3 { x: z / 2, y: -z / 4, z },
            color,
            0,
        );
    }

    #[test]
    fn sort_is_far_to_near_and_stable() {
        let mut r = Renderer::new();
        r.begin(0);
        for (z, color) in [(100, 1), (400, 2), (100, 3), (900, 4), (400, 5)] {
            tri(&mut r, z, color);
        }
        assert_eq!(r.triangle_count, 5);
        r.finish();
        let colors: std::vec::Vec<u8> = r.order[..5].iter().map(|&i| r.triangles[i as usize].color).collect();
        assert_eq!(colors, [4, 2, 5, 1, 3]);
    }

    #[test]
    fn nearer_triangle_paints_over_farther() {
        let mut r = Renderer::new();
        r.begin(6);
        tri(&mut r, 100, 2);
        tri(&mut r, 400, 5);
        r.finish();
        let mut scratch = RasterScratch::new();
        let mut pass = r.raster_pass(&mut scratch);
        let mut pixels = [0u8; BAND_PIXELS];
        while let Some(band) = pass.next_band(&mut pixels) {
            if band == (crate::projection::CENTER_Y / 8) as u8 {
                assert_eq!(pixels[(crate::projection::CENTER_Y as usize % 8) * WIDTH + 80] & 15, 2);
            }
        }
    }

    #[test]
    fn back_faces_and_overflow() {
        let mut r = Renderer::new();
        r.begin(0);
        // Counter-clockwise on screen: culled.
        r.triangle(Vec3 { x: 0, y: 0, z: 100 }, Vec3 { x: 50, y: 0, z: 100 }, Vec3 { x: 0, y: 50, z: 100 }, 1, 0);
        assert_eq!(r.triangle_count, 0);
        for _ in 0..300 {
            tri(&mut r, 200, 1);
        }
        assert_eq!(r.triangle_count as usize, TRIANGLE_CAPACITY);
        assert_eq!(r.dropped, 44);
    }
}
