//! Fixed-point arithmetic for `geom`, shaped for the 6502: one 16×16 → 32 unsigned multiply
//! (`asm/fixed.S`, quarter squares, ~230 cycles; the host multiplies natively and gets the same bits) and
//! division by a reciprocal table on an 11-bit normalised mantissa (no divide instruction anywhere).

/// Quarter squares `f(n) = n²/4` for `n` = 0..511, low and high bytes (`asm/fixed.S`); the C64's old
/// screen RAM. `a·b = f(a + b) - g(b - a + 255)` with `g(n) = f(|n - 255|)` for `n` = 0..510 (after the
/// geometry scratch), so both lookups are `(pointer),y` with `y = b`.
pub const SQ_LO: u16 = 0x0400;
pub const SQ_HI: u16 = 0x0600;
pub const SQ_NEG_LO: u16 = 0x5000;
pub const SQ_NEG_HI: u16 = 0x5200;
/// Reciprocals `(2^26 − 1) / m` for `m` = 1024..2047, u16 little-endian (2 KiB), after the scene block.
pub const RECIP: u16 = 0x9000;
pub const MANTISSA: u16 = 1024;

#[cfg(target_arch = "mos")]
unsafe extern "C" {
    fn m7_umul16(a: u16, b: u16) -> u32;
    fn m7_smul16(a: i16, b: i16) -> i32;
    fn m7_umul_init();
}

/// `a * b`, exactly.
#[inline(always)]
pub fn umul(a: u16, b: u16) -> u32 {
    #[cfg(target_arch = "mos")]
    unsafe {
        m7_umul16(a, b)
    }
    #[cfg(not(target_arch = "mos"))]
    {
        a as u32 * b as u32
    }
}

/// `a * b`, exactly (`asm/fixed.S` `m7_smul16` on the C64: one call site's worth of code, a tail call).
#[inline(never)]
pub fn mul(a: i16, b: i16) -> i32 {
    #[cfg(target_arch = "mos")]
    unsafe {
        m7_smul16(a, b)
    }
    #[cfg(not(target_arch = "mos"))]
    {
        a as i32 * b as i32
    }
}

/// `a * b >> 14`: a Q14 product in 16 bits (resident on the C64: one copy for the overlays' callers).
#[inline(never)]
pub fn mul14(a: i16, b: i16) -> i16 {
    (mul(a, b) >> 14) as i16
}

/// `a * b`, exactly; `a` must not be `i16::MIN`.
#[inline(never)]
pub fn mulsu(a: i16, b: u16) -> i32 {
    let p = umul(a.unsigned_abs(), b) as i32;
    if a < 0 { -p } else { p }
}

/// `d * v`, exactly, for `0 <= d` (a 16 × 32 product whose result fits in i32).
pub fn mul_wide(d: i16, v: i32) -> i32 {
    let hi = mul(d, (v >> 16) as i16);
    let lo = mulsu(d, v as u16);
    (hi << 16).wrapping_add(lo)
}

const fn recip_value(m: u16) -> u16 {
    (0x3FF_FFFF / m as u32) as u16
}

/// The reciprocal of a mantissa (`MANTISSA..2 * MANTISSA`): `(2^26 − 1) / m`.
#[inline]
pub fn recip(m: u16) -> u16 {
    #[cfg(target_arch = "mos")]
    unsafe {
        let p = (RECIP + (m - MANTISSA) * 2) as *const u8;
        u16::from_le_bytes([core::ptr::read_volatile(p), core::ptr::read_volatile(p.add(1))])
    }
    #[cfg(not(target_arch = "mos"))]
    {
        recip_value(m)
    }
}

/// `v = m · 2^e` (truncating) with `m` in `MANTISSA..2 * MANTISSA`; `v > 0`.
#[inline(never)]
pub fn norm(v: u16) -> (u16, i8) {
    let (mut m, mut e) = (v, 0i8);
    while m < MANTISSA {
        m <<= 1;
        e -= 1;
    }
    while m >= 2 * MANTISSA {
        m >>= 1;
        e += 1;
    }
    (m, e)
}

/// Arithmetic shift right by `s` (0..31).
#[inline]
pub fn sar(v: i32, s: u8) -> i32 {
    v >> s
}

/// Fill the C64's tables (at boot).
#[cfg(target_arch = "mos")]
pub fn init_tables() {
    unsafe {
        for n in 0..512u16 {
            let q = ((n as u32 * n as u32) >> 2) as u16;
            core::ptr::write_volatile((SQ_LO + n) as *mut u8, q as u8);
            core::ptr::write_volatile((SQ_HI + n) as *mut u8, (q >> 8) as u8);
            let d = n.abs_diff(255) as u32;
            let g = ((d * d) >> 2) as u16;
            core::ptr::write_volatile((SQ_NEG_LO + n) as *mut u8, g as u8);
            core::ptr::write_volatile((SQ_NEG_HI + n) as *mut u8, (g >> 8) as u8);
        }
        for m in MANTISSA..2 * MANTISSA {
            let r = recip_value(m);
            let p = (RECIP + (m - MANTISSA) * 2) as *mut u8;
            core::ptr::write_volatile(p, r as u8);
            core::ptr::write_volatile(p.add(1), (r >> 8) as u8);
        }
        m7_umul_init();
    }
}
