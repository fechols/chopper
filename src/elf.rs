//! Just enough of an ELF32 (little-endian) reader for `chopper bake --elf`: a section's bytes by name and
//! a symbol's bytes, to copy the C64 build's REU overlays and its build stamp into the image.

fn u16_at(b: &[u8], at: usize) -> usize {
    u16::from_le_bytes([b[at], b[at + 1]]) as usize
}

fn u32_at(b: &[u8], at: usize) -> usize {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]) as usize
}

/// A section header: name offset, type, flags (2: allocated), address, file offset, size, link.
struct Section {
    name: usize,
    kind: usize,
    flags: usize,
    addr: usize,
    offset: usize,
    size: usize,
    link: usize,
}

fn sections(elf: &[u8]) -> Result<Vec<Section>, String> {
    if elf.len() < 52 || &elf[..4] != b"\x7fELF" || elf[4] != 1 || elf[5] != 1 {
        return Err("not a little-endian ELF32 file".into());
    }
    let (shoff, shentsize, shnum) = (u32_at(elf, 0x20), u16_at(elf, 0x2E), u16_at(elf, 0x30));
    Ok((0..shnum)
        .map(|i| {
            let h = shoff + i * shentsize;
            Section { name: u32_at(elf, h), kind: u32_at(elf, h + 4), flags: u32_at(elf, h + 8), addr: u32_at(elf, h + 12), offset: u32_at(elf, h + 16), size: u32_at(elf, h + 20), link: u32_at(elf, h + 24) }
        })
        .collect())
}

fn cstr(b: &[u8], at: usize) -> &[u8] {
    let end = b[at..].iter().position(|&c| c == 0).map_or(b.len(), |n| at + n);
    &b[at..end]
}

/// The bytes of section `name`.
pub fn section<'a>(elf: &'a [u8], name: &str) -> Result<&'a [u8], String> {
    let secs = sections(elf)?;
    let names = &secs[u16_at(elf, 0x32)];
    for s in &secs {
        if cstr(elf, names.offset + s.name) == name.as_bytes() {
            return Ok(&elf[s.offset..s.offset + s.size]);
        }
    }
    Err(format!("no section {name}"))
}

/// The `len` bytes at symbol `name` (from the section holding its address).
pub fn symbol<'a>(elf: &'a [u8], name: &str, len: usize) -> Result<&'a [u8], String> {
    let secs = sections(elf)?;
    let symtab = secs.iter().find(|s| s.kind == 2).ok_or("no symbol table")?;
    let strtab = &secs[symtab.link];
    for k in 0..symtab.size / 16 {
        let e = symtab.offset + k * 16;
        if cstr(elf, strtab.offset + u32_at(elf, e)) != name.as_bytes() {
            continue;
        }
        let addr = u32_at(elf, e + 4);
        let s = secs.iter().find(|s| s.kind == 1 && s.flags & 2 != 0 && s.addr <= addr && addr + len <= s.addr + s.size).ok_or(format!("{name}: in no loaded section"))?;
        let at = s.offset + addr - s.addr;
        return Ok(&elf[at..at + len]);
    }
    Err(format!("no symbol {name}"))
}
