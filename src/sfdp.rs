//! JEDEC SFDP (Serial Flash Discoverable Parameters) and BFPT (Basic Flash Parameter Table) parsing.
//!
//! Use with [`read_sfdp`](crate::Is25lp128f::read_sfdp) to discover device geometry and capabilities.

/// SFDP header (9 bytes at offset 0). JEDEC standard.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct SfdpHeader {
    /// Minor revision (byte 4).
    pub minor: u8,
    /// Major revision (byte 5).
    pub major: u8,
    /// Number of parameter headers (1-based; byte 6).
    pub nph: u8,
}

impl SfdpHeader {
    /// Signature "SFDP" (0x53, 0x46, 0x44, 0x50).
    pub const SIGNATURE: [u8; 4] = [0x53, 0x46, 0x44, 0x50];

    /// Parse from raw SFDP data (at least 9 bytes). Returns `None` if signature is invalid.
    #[must_use]
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 9 {
            return None;
        }
        if data[0..4] != Self::SIGNATURE {
            return None;
        }
        Some(Self {
            minor: data[4],
            major: data[5],
            nph: data[6],
        })
    }
}

/// JEDEC Basic Flash Parameter Table (BFPT) ID in parameter header (LSB first).
pub const BFPT_PARAMETER_ID: u16 = 0xFF00;

/// Minimal BFPT-derived info (from first DWORDs of the table).
#[derive(Debug, Clone, Copy, Eq, PartialEq, Default)]
pub struct BfptInfo {
    /// Minimum erase size in bytes (e.g. 4096). From BFPT DWORD 1 bits 1:0 and table.
    pub sector_erase_size: Option<u32>,
    /// Page size in bytes (e.g. 256). From BFPT.
    pub page_size: Option<u32>,
    /// Memory density in bits (e.g. 128 * 1024 * 1024 for 128 Mbit). From BFPT DWORD 4–5.
    pub density_bits: Option<u64>,
}

/// Parse the first parameter header (8 bytes) to get table pointer and length.
#[must_use]
fn parse_param_header(data: &[u8], offset: usize) -> Option<(u16, u8)> {
    if data.len() < offset + 8 {
        return None;
    }
    let id = u16::from(data[offset]) | (u16::from(data[offset + 1]) << 8);
    if id != BFPT_PARAMETER_ID {
        return None;
    }
    let length = data[offset + 4];
    let pointer = u16::from(data[offset + 5]) | (u16::from(data[offset + 6]) << 8);
    Some((pointer, length))
}

/// Parse BFPT from raw SFDP data. Data should include the SFDP header and parameter headers,
/// and the BFPT table (typically read via `read_sfdp(0, buf)` with buf large enough, e.g. 256 bytes).
/// Returns `None` if header is invalid or BFPT table is not found.
#[must_use]
pub fn parse_bfpt(data: &[u8]) -> Option<BfptInfo> {
    let header = SfdpHeader::parse(data)?;
    let param_base = 8_usize;
    let (pointer, length) = parse_param_header(data, param_base)?;
    let table_start = (u32::from(pointer) * 4) as usize;
    let table_len = (u32::from(length) * 4) as usize;
    if data.len() < table_start + table_len || table_len < 8 {
        return None;
        // BFPT table: DWORD 0 = "SFBP" 0x50425753, DWORD 1 = revision and erase/page info
    }
    let d1 = u32::from(data[table_start + 4])
        | (u32::from(data[table_start + 5]) << 8)
        | (u32::from(data[table_start + 6]) << 16)
        | (u32::from(data[table_start + 7]) << 24);
    // DWORD 1: bits 1:0 = sector erase size (01b = 4 KB), bit 2 = write granularity, etc.
    let sector_erase_bits = d1 & 0x03;
    let sector_erase_size = match sector_erase_bits {
        0 => Some(256),
        1 => Some(4096),
        2 => Some(8192),
        3 => Some(65536),
        _ => None,
    };
    // Page size often in later DWORD; BFPT DWORD 2 byte 4 = page size exponent (8 = 256).
    let page_size = if table_len >= 12 {
        let exp = data[table_start + 8];
        Some(1u32 << exp)
    } else {
        Some(256)
    };
    // Density: BFPT DWORD 4 bits 7:0 = (size - 1) in bits; JEDEC uses (N+1) for density in bits.
    let density_bits = if table_len >= 20 {
        let d4 = u32::from(data[table_start + 16])
            | (u32::from(data[table_start + 17]) << 8)
            | (u32::from(data[table_start + 18]) << 16)
            | (u32::from(data[table_start + 19]) << 24);
        Some((d4 & 0x7F) as u64 + 1)
    } else {
        None
    };
    Some(BfptInfo {
        sector_erase_size,
        page_size,
        density_bits,
    })
}
