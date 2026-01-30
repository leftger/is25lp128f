//! Async no_std driver for ISSI IS25LP128F 128 Mbit SPI NOR flash.
//!
//! ## Implemented instructions (Table 6.3 and companion reads)
//!
//! | Opcode | Mnemonic | Method |
//! |--------|----------|--------|
//! | 01h | WRSR | `write_status`, `write_status_volatile` |
//! | 02h | PP | `write_page` |
//! | 03h | READ | `read` |
//! | 04h | WRDI | `disable_write_latch` |
//! | 05h | RDSR | `read_status` |
//! | 06h | WREN | `enable_write_latch` |
//! | 0Bh | Fast Read | `read_fast`, `read_fast_with_dummy` |
//! | 3Bh | Fast Read Dual Output | `read_fast_dual_output` |
//! | 6Bh | Fast Read Quad Output | `read_fast_quad_output` |
//! | BBh | Fast Read Dual I/O | `read_fast_dual_io` |
//! | EBh | Fast Read Quad I/O | `read_fast_quad_io` |
//! | 0Fh | Read Read Reg | `read_read_register` (opcode per datasheet) |
//! | 12h | 4PP | `write_page_4byte` |
//! | 14h | RDABR | `read_autoboot_register` |
//! | 15h | WRABR | `write_autoboot_register` |
//! | 16h | RDBR | `read_bank_register` |
//! | 17h | WRBRV (no WREN) | `write_bank_register_volatile_no_wren` |
//! | 18h | WRBRNV | `write_bank_register_nv` |
//! | 21h | 4SER | `erase_4byte` (4K) |
//! | 29h | EX4B | `exit_4byte_address_mode` |
//! | 32h | PPQ | `write_page_quad` |
//! | 34h | 4PPQ | `write_page_4byte_quad` |
//! | 42h | WRFR | `write_function_register` |
//! | 48h | RDFR | `read_function_register` |
//! | 4Bh | RDUID | `read_unique_id` |
//! | 4Fh | Read Extended Read Reg | `read_extended_read_register` (opcode per datasheet) |
//! | 50h | VSRWE | (used by `write_status_volatile`) |
//! | 52h | BER32 | `erase` (32K) |
//! | 5Ah | Read SFDP | `read_sfdp` |
//! | 5Ch | 4BER32 | `erase_4byte` (32K) |
//! | 62h | IRP | `program_information_row` |
//! | 63h | SRPV | `set_read_parameters_volatile` |
//! | 64h | IRER | `erase_information_row` |
//! | 65h | SRPNV | `set_read_parameters_nv` |
//! | 66h/99h | Reset | `software_reset` |
//! | 75h | Suspend | `suspend` |
//! | 7Ah | Resume | `resume` |
//! | 7Eh | GBLK | `gang_lock` |
//! | 83h | SERPV | `set_extended_read_parameters_volatile` |
//! | 85h | SERPNV | `set_extended_read_parameters_nv` |
//! | 91h | SFRZ | `set_freeze` |
//! | 98h | GBUN | `gang_unlock` |
//! | 9Fh | RDID | `read_jedec_id` |
//! | A6h | WRPLB | `write_ppb_lock` |
//! | A7h | RDPLB | `read_ppb_lock` |
//! | ABh | Release DP | `exit_deep_power_down` |
//! | B7h | EN4B | `enter_4byte_address_mode` |
//! | B9h | Enter DP | `enter_deep_power_down` |
//! | C5h | WRBRV | `write_bank_register_volatile` |
//! | C7h | CER | `full_erase` |
//! | D7h | SER | `erase` (4K) |
//! | D8h | BER64 | `erase` (64K) |
//! | DCh | 4BER64 | `erase_4byte` (64K) |
//! | E0h | 4RDDYB | `read_dyb_4byte` |
//! | E1h | 4WRDYB | `write_dyb_4byte` |
//! | E2h | 4RDPPB | `read_ppb_4byte` |
//! | E3h | 4PGPPB | `program_ppb_4byte` |
//! | E4h | ERPPB | `erase_ppb` |
//! | E7h | RDPWD | `read_password` |
//! | E8h | PGPWD | `program_password` |
//! | E9h | UNPWD | `unlock_password` |
//! | FAh | RDDYB | `read_dyb` |
//! | FBh | WRDYB | `write_dyb` |
//! | FCh | RDPPB | `read_ppb` |
//! | FDh | PGPPB | `program_ppb` |
//! | 2Bh | RDASP | `read_asp` |
//! | 2Fh | PGASP | `program_asp` |
//! | 13h | Read 4-byte | `read_4byte` |

#![allow(unused)]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod prelude;

#[cfg(feature = "sfdp")]
pub mod sfdp;

mod generated {
    include!(concat!(env!("OUT_DIR"), "/is25lp128f_device.rs"));
}
pub use generated::field_sets::StatusReg as Status;

use modular_bitfield::prelude::*;

/// Flash page size in bytes; Datasheet p. 77
pub const PAGE_SIZE: u32 = 256;

/// The total chip size in bytes; Datasheet pp. 2.3
pub const CHIP_SIZE: u32 = (128 * 1024 * 1024 / 8/* 128 Mbits */);

// The sector size in bytes; Datasheet p. 14
pub const SECTOR_SIZE: u32 = 4 * 1024;

/// Memory map specified starting address; Datasheet p. 14
pub const START_ADDRESS: u16 = 0x00;

/// JEDEC manufacturer and device identification (RDID 9Fh response).
///
/// IS25LP128F returns 3 bytes: manufacturer ID (e.g. 0x9D for ISSI), memory type (e.g. 0x60),
/// capacity (e.g. 0x18 for 128 Mbit). Compatible with JEDEC continuation codes in longer responses.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct JedecId {
    /// Manufacturer ID (first byte after any 0x7F continuations).
    pub mfr_code: u8,
    /// Memory type (second byte).
    pub memory_type: u8,
    /// Capacity / device ID (third byte; e.g. 0x18 = 128 Mbit).
    pub capacity: u8,
}

impl JedecId {
    /// Build from the 3-byte JEDEC ID (after the RDID opcode).
    pub fn from_bytes(bytes: [u8; 3]) -> Self {
        Self {
            mfr_code: bytes[0],
            memory_type: bytes[1],
            capacity: bytes[2],
        }
    }

    /// Build from a slice (e.g. from extended RDID response with 0x7F continuations).
    /// Uses the last 3 bytes as the main manufacturer + device ID (spi-memory style).
    pub fn from_jedec_id(buf: &[u8]) -> Self {
        let start = buf.len().saturating_sub(3);
        let slice = &buf[start..];
        Self::from_bytes([slice[0], slice[1], slice[2]])
    }
}

/// Max internal write page cycle time is 0.8ms; Datasheet p. 171
/// Add additional time for robustness
pub const WRITE_MAX_TIME_MSEC: u64 = 2;

#[bitfield]
#[derive(Default, Debug, Clone, Copy, Eq, PartialEq)]
struct ReadWriteCmd {
    #[skip(getters)]
    pub write_or_read: B3,
    pub address_msb: B1,
    #[skip(getters)]
    pub zeroes: B4,
}

#[derive(Debug, PartialEq)]
pub enum Error {
    SpiConfigError,
    SpiWriteError,
    SpiReadError,
    SpiTimeoutError(u32),
    AddressOutOfBounds(i32),
    BufferSizeInvalid(usize),
    /// Status register contained unexpected flags (e.g. BUSY or WEL after init).
    /// Can indicate a faulty chip, bad connection, or driver used while an operation was in progress.
    UnexpectedStatus,
    /// SPI device (e.g. shared bus) operation failed.
    #[cfg(feature = "spi-device")]
    SpiDevice,
}

/// Writable status register bits (WRSR 01h). WIP and WEL are read-only; BP0–BP3, QE, SRWD are writable.
/// Datasheet Table 6.1–6.2.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Default)]
pub struct StatusWrite {
    /// Block protection bits (BP0–BP3). See datasheet Table 6.4 for protected area.
    pub block_protect: u8,
    /// Quad Enable (bit 6).
    pub quad_enable: bool,
    /// Status Register Write Disable (bit 7); with WP# low, SR becomes read-only.
    pub srwd: bool,
}

/// Block protection level (BP3–BP0) per datasheet Table 6.4. Standard table; TBS selects top vs bottom.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum BlockProtectionLevel {
    /// No blocks protected (BP = 0000).
    None,
    /// 1 block (BP = 0001).
    Top1Block,
    /// 2 blocks (BP = 0010).
    Top2Blocks,
    /// 4 blocks (BP = 0011).
    Top4Blocks,
    /// 8 blocks (BP = 0100).
    Top8Blocks,
    /// 16 blocks (BP = 0101).
    Top16Blocks,
    /// 32 blocks (BP = 0110).
    Top32Blocks,
    /// 64 blocks (BP = 0111).
    Top64Blocks,
    /// 128 blocks (BP = 1000).
    Top128Blocks,
    /// All 256 blocks (BP = 1001).
    All,
}

impl BlockProtectionLevel {
    /// Raw BP3–BP0 value (0..=9) for Status Register.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Top1Block => 1,
            Self::Top2Blocks => 2,
            Self::Top4Blocks => 3,
            Self::Top8Blocks => 4,
            Self::Top16Blocks => 5,
            Self::Top32Blocks => 6,
            Self::Top64Blocks => 7,
            Self::Top128Blocks => 8,
            Self::All => 9,
        }
    }

    /// From raw BP bits (0..=15); values 10–15 map to All per optional table.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        match bits & 0x0F {
            0 => Self::None,
            1 => Self::Top1Block,
            2 => Self::Top2Blocks,
            3 => Self::Top4Blocks,
            4 => Self::Top8Blocks,
            5 => Self::Top16Blocks,
            6 => Self::Top32Blocks,
            7 => Self::Top64Blocks,
            8 => Self::Top128Blocks,
            _ => Self::All,
        }
    }
}

impl StatusWrite {
    /// Encode to the byte written by WRSR (bits 7:0 = SRWD, QE, BP3–BP0, 0, 0).
    #[must_use]
    pub fn to_byte(self) -> u8 {
        let bp = self.block_protect & 0x0F;
        let qe = if self.quad_enable { 1 << 6 } else { 0 };
        let srwd = if self.srwd { 1 << 7 } else { 0 };
        srwd | qe | (bp << 2)
    }

    /// Decode from a status byte (e.g. from RDSR); only writable bits are meaningful.
    #[must_use]
    pub fn from_status_byte(b: u8) -> Self {
        Self {
            block_protect: (b >> 2) & 0x0F,
            quad_enable: (b & (1 << 6)) != 0,
            srwd: (b & (1 << 7)) != 0,
        }
    }

    /// Set block protection from a level (BP3–BP0 per Table 6.4). Other fields unchanged.
    #[must_use]
    pub fn with_block_protection(mut self, level: BlockProtectionLevel) -> Self {
        self.block_protect = level.to_bits();
        self
    }
}

/// Addressing mode for legacy commands. 3-byte is sufficient for 128 Mbit.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum AddressMode {
    /// 24-bit address (default).
    ThreeByte,
    /// 32-bit address (EN4B B7h); use with 4-byte read/write/erase methods.
    FourByte,
}

/// Output driver strength (ODS) for read modes. Extended Read Register bits EB7, EB6, EB5.
/// Datasheet Table 6.14. SERPNV/SERPV set only these bits; EB4:0 are read-only and not affected.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum DriverStrength {
    /// 12.5% (ODS = 001)
    P12_5,
    /// 25% (ODS = 010)
    P25,
    /// 37.5% (ODS = 011)
    P37_5,
    /// 75% (ODS = 101)
    P75,
    /// 100% (ODS = 110)
    P100,
    /// 50% (ODS = 111). Default.
    P50,
}

impl Default for DriverStrength {
    fn default() -> Self {
        Self::P50
    }
}

impl DriverStrength {
    /// Raw ODS bits (0..=7) for EB7:5. Reserved 000 and 100 are not represented.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        match self {
            Self::P12_5 => 0b001,
            Self::P25 => 0b010,
            Self::P37_5 => 0b011,
            Self::P75 => 0b101,
            Self::P100 => 0b110,
            Self::P50 => 0b111,
        }
    }

    /// Byte value to write via SERPNV/SERPV (ODS in bits 5–7; lower bits ignored by device).
    #[must_use]
    pub const fn to_register_byte(self) -> u8 {
        self.to_bits() << 5
    }

    /// Decode from Extended Read Register byte (EB7:5). Reserved/unknown values map to P50.
    #[must_use]
    pub fn from_register_byte(b: u8) -> Self {
        match (b >> 5) & 0x07 {
            0b001 => Self::P12_5,
            0b010 => Self::P25,
            0b011 => Self::P37_5,
            0b101 => Self::P75,
            0b110 => Self::P100,
            _ => Self::P50,
        }
    }
}

/// Extended Read Register contents (opcode 4Fh). Table 6.12–6.13.
///
/// EB7, EB6, EB5 (ODS) are writable via SERPNV/SERPV. EB4 is reserved. EB3:0 are read-only
/// and indicate WIP and operation errors; they are not affected by SERPNV or SERPV.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct ExtendedReadStatus {
    /// EB0: Write In Progress (same as Status Register WIP).
    pub wip: bool,
    /// EB1: Protection error (erase/program attempted on protected area or locked IR).
    pub protection_error: bool,
    /// EB2: Program error (program failed or attempted on protected/locked area).
    pub program_error: bool,
    /// EB3: Erase error (erase failed or attempted on protected/locked area).
    pub erase_error: bool,
    /// EB7:5: Output driver strength (writable; current value as read back).
    pub driver_strength: DriverStrength,
}

impl ExtendedReadStatus {
    /// Build from the raw Extended Read Register byte.
    #[must_use]
    pub fn from_byte(b: u8) -> Self {
        Self {
            wip: (b & 1) != 0,
            protection_error: (b & (1 << 1)) != 0,
            program_error: (b & (1 << 2)) != 0,
            erase_error: (b & (1 << 3)) != 0,
            driver_strength: DriverStrength::from_register_byte(b),
        }
    }

    /// True if any operation error bit is set (P_ERR, E_ERR, or PROT_E).
    #[must_use]
    pub const fn has_error(self) -> bool {
        self.protection_error || self.program_error || self.erase_error
    }
}

/// Operation error bits from the Extended Read Register (EB3:1). Read-only; set by failed erase/program or protection violations.
/// Use [`read_operation_errors`](Is25lp128f::read_operation_errors) to read after an operation.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Default)]
pub struct OperationErrors {
    /// EB2: Program error (program failed or attempted on protected/locked area).
    pub program_error: bool,
    /// EB3: Erase error (erase failed or attempted on protected/locked area).
    pub erase_error: bool,
    /// EB1: Protection error (erase/program attempted on protected sector/block or locked IR).
    pub protection_error: bool,
}

impl OperationErrors {
    /// True if any error bit is set.
    #[must_use]
    pub const fn has_error(self) -> bool {
        self.program_error || self.erase_error || self.protection_error
    }
}

impl From<ExtendedReadStatus> for OperationErrors {
    fn from(s: ExtendedReadStatus) -> Self {
        Self {
            program_error: s.program_error,
            erase_error: s.erase_error,
            protection_error: s.protection_error,
        }
    }
}

// Datasheet pp. 21–24, Table 6.3, and SFDP/ASP sections.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
enum Opcode {
    WriteEnable = 0x06,
    WriteDisable = 0x04,
    ReadStatus = 0x05,
    WriteStatus = 0x01,
    VolatileStatusWriteEnable = 0x50,
    NormalRead = 0x03,
    FastRead = 0x0B,
    FastRead4 = 0x0C,
    FastReadDualOutput = 0x3B,
    FastReadDualIo = 0xBB,
    FastReadQuadOutput = 0x6B,
    FastReadQuadIo = 0xEB,
    PageProgram = 0x02,
    SectorErase = 0xD7,
    BlockErase32 = 0x52,
    BlockErase64 = 0xD8,
    ChipErase = 0xC7,
    FlashID = 0x9F,
    ReadUniqueID = 0x4B,
    Suspend = 0x75,
    Resume = 0x7A,
    ResetEnable = 0x66,
    ResetDevice = 0x99,
    DeepPowerDownEnter = 0xB9,
    DeepPowerDownExit = 0xAB,
    ReadSfdp = 0x5A,
    Enter4Byte = 0xB7,
    Exit4Byte = 0x29,
    Read4Byte = 0x13,
    PageProgram4Byte = 0x12,
    SectorErase4Byte = 0x21,
    BlockErase32_4Byte = 0x5C,
    BlockErase64_4Byte = 0xDC,
    ReadFunctionReg = 0x48,
    WriteFunctionReg = 0x42,
    SetReadParamsNv = 0x65,
    SetReadParamsVolatile = 0x63,
    ReadReadReg = 0x0F,
    SetExtendedReadParamsNv = 0x85,
    SetExtendedReadParamsVolatile = 0x83,
    ReadExtendedReadReg = 0x4F,
    RdAsp = 0x2B,
    PgAsp = 0x2F,
    RdPwd = 0xE7,
    PgPwd = 0xE8,
    UnPwd = 0xE9,
    RdPpbLock = 0xA7,
    WrPpbLock = 0xA6,
    SetFreeze = 0x91,
    RdPpb = 0xFC,
    PgPpb = 0xFD,
    ErasePpb = 0xE4,
    RdDyb = 0xFA,
    WrDyb = 0xFB,
    GangLock = 0x7E,
    GangUnlock = 0x98,
    RdPpb4Byte = 0xE2,
    PgPpb4Byte = 0xE3,
    RdDyb4Byte = 0xE0,
    WrDyb4Byte = 0xE1,
    // Table 6.3: PPQ / 4PPQ, IRER, IRP, WRABR, WRBRNV, WRBRV
    PageProgramQuad = 0x32,           // PPQ 3-byte address
    PageProgramQuad4Byte = 0x34,      // 4PPQ 4-byte address (34h/3Eh)
    EraseInformationRow = 0x64,       // IRER
    ProgramInformationRow = 0x62,     // IRP
    ReadInformationRow = 0x4A,       // IRRD (read Information Row; opcode per datasheet)
    WriteAutoBootReg = 0x15,          // WRABR
    ReadAutoBootReg = 0x14,           // RDABR (read AutoBoot Register)
    ReadBankReg = 0x16,               // RDBR
    WriteBankRegNv = 0x18,            // WRBRNV
    WriteBankRegVolatile = 0xC5,      // WRBRV (requires WREN)
    WriteBankRegVolatileNoWren = 0x17, // WRBRV (Note 2: no WREN)
}

pub enum FlashEraseSize {
    /// Sector Erase, 70-300ms erase time; Datasheet p. 171
    /// Add additional time to max for robustness
    EraseSize4K,
    /// Block Erase, 100-500ms erase time; Datasheet p. 171
    /// Add additional time to max for robustness
    EraseSize32K,
    /// Block Erase 150-1000ms erase time; Datasheet p. 171
    /// Add additional time to max for robustness
    EraseSize64K,
}

impl FlashEraseSize {
    pub fn get_timeout(&self) -> u64 {
        match self {
            FlashEraseSize::EraseSize4K => 301,
            FlashEraseSize::EraseSize32K => 501,
            FlashEraseSize::EraseSize64K => 1001,
        }
    }
    fn validate_address(&self, address: i32) -> Result<(), Error> {
        match self {
            FlashEraseSize::EraseSize4K => {
                if address + (4 * 1024) > (CHIP_SIZE as i32) {
                    return Err(Error::AddressOutOfBounds(address));
                }
            }
            FlashEraseSize::EraseSize32K => {
                if address + (32 * 1024) > (CHIP_SIZE as i32) {
                    return Err(Error::AddressOutOfBounds(address));
                }
            }
            FlashEraseSize::EraseSize64K => {
                if address + (64 * 1024) > (CHIP_SIZE as i32) {
                    return Err(Error::AddressOutOfBounds(address));
                }
            }
        };
        Ok(())
    }
}

#[allow(async_fn_in_trait)]
pub trait Spi {
    async fn configure_spi(&mut self) -> Result<(), Error>;
    async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Error>;
    async fn read(
        &mut self,
        read_cmd_buf: &[u8],
        read_buf: &mut [u8],
    ) -> Result<(), Error>;
    async fn write(&mut self, data: &[u8]) -> Result<(), Error>;
}

#[allow(async_fn_in_trait)]
pub trait HardwareInterface {
    async fn wait_ms(&mut self, timeout_ms: u64);
}
/// ISSI IS25LP128F driver.
pub struct Is25lp128f<S: Spi, H: HardwareInterface> {
    spi: S,
    hardware_interface: H,
}
impl<S: Spi, H: HardwareInterface> Is25lp128f<S, H> {
    pub fn new(spi: S, hardware_interface: H) -> Is25lp128f<S, H> {
        Is25lp128f {
            spi,
            hardware_interface,
        }
    }

    /// Initialize the driver: wait for any in-progress operation and verify status.
    ///
    /// Call after `new()` to ensure the chip is idle (no BUSY, WEL cleared).
    /// Returns `Err(Error::UnexpectedStatus)` if the status register indicates an unexpected state.
    pub async fn init(&mut self) -> Result<(), Error> {
        self.wait_done().await?;
        let status = self.read_status().await?;
        if status.busy() || status.wel() {
            return Err(Error::UnexpectedStatus);
        }
        Ok(())
    }

    pub async fn read(
        &mut self,
        address: i32,
        read_buf: &mut [u8],
    ) -> Result<(), Error> {
        let mut read_cmd_buf = [0x00_u8; 4];
        prepare_read_command_buffer(
            address,
            read_buf.len(),
            &mut read_cmd_buf,
        )?;
        self.spi.read(&read_cmd_buf, read_buf).await?;
        Ok(())
    }

    pub async fn enable_write_latch(&mut self) -> Result<(), Error> {
        let mut write_latch_cmd_buf = [Opcode::WriteEnable as u8];
        self.spi.write(&write_latch_cmd_buf).await?;
        Ok(())
    }

    pub async fn write_page(
        &mut self,
        address: i32,
        data: &[u8],
    ) -> Result<(), Error> {
        let mut buf = [0; (PAGE_SIZE + 4) as usize];
        prepare_write_command_buffer(address, data, &mut buf)?;
        let buf_slice = &buf[..4 + data.len()];

        self.enable_write_latch().await?;
        self.spi.write(buf_slice).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Write arbitrary-length data at the given address (caller must have erased the region).
    ///
    /// Writes in page-sized chunks; handles partial pages at the end.
    pub async fn write_bytes(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        if offset > CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(offset as i32));
        }
        let mut addr = offset as i32;
        let mut remaining = data;
        while !remaining.is_empty() {
            let chunk_len = remaining.len().min(PAGE_SIZE as usize);
            let (chunk, rest) = remaining.split_at(chunk_len);
            self.write_page(addr, chunk).await?;
            addr += chunk_len as i32;
            remaining = rest;
        }
        Ok(())
    }

    /// Erase the entire chip. Takes a long time; prefer sector/block erase when possible.
    pub async fn full_erase(&mut self) -> Result<(), Error> {
        let mut erase_cmd_buf = [Opcode::ChipErase as u8];
        self.spi.write(&erase_cmd_buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    pub async fn erase(
        &mut self,
        erase_type: FlashEraseSize,
        address: i32,
    ) -> Result<(), Error> {
        let buf = prepare_erase_command_buffer(&erase_type, address)?;
        self.enable_write_latch().await?;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read the 8-bit status register.
    pub async fn read_status(&mut self) -> Result<Status, Error> {
        let mut buf: [u8; 2] = [Opcode::ReadStatus as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(Status::from([buf[1]]))
    }

    /// Wait until the device is not busy (no erase/write in progress).
    pub async fn wait_done(&mut self) -> Result<(), Error> {
        while self.read_status().await?.busy() {
            self.hardware_interface.wait_ms(1).await;
        }
        Ok(())
    }

    /// Wait until the device is not busy, or return `Err(SpiTimeoutError(timeout_ms))` after `timeout_ms` ms.
    pub async fn wait_done_timeout(&mut self, timeout_ms: u64) -> Result<(), Error> {
        let mut elapsed_ms: u64 = 0;
        while self.read_status().await?.busy() {
            if elapsed_ms >= timeout_ms {
                return Err(Error::SpiTimeoutError(
                    timeout_ms.min(u64::from(u32::MAX)) as u32,
                ));
            }
            self.hardware_interface.wait_ms(1).await;
            elapsed_ms = elapsed_ms.saturating_add(1);
        }
        Ok(())
    }

    /// Read the single-byte manufacturer ID (first byte of JEDEC ID).
    /// For full JEDEC identification use [`read_jedec_id`](Self::read_jedec_id).
    pub async fn read_flash_id(&mut self) -> Result<u8, Error> {
        let id = self.read_jedec_id().await?;
        Ok(id.mfr_code)
    }

    /// Read JEDEC manufacturer and device identification (RDID 9Fh).
    /// Returns the 3-byte ID (manufacturer, memory type, capacity).
    pub async fn read_jedec_id(&mut self) -> Result<JedecId, Error> {
        let mut buf = [Opcode::FlashID as u8, 0, 0, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(JedecId::from_bytes([buf[1], buf[2], buf[3]]))
    }

    pub async fn read_unique_id(&mut self) -> Result<[u8; 16], Error> {
        // RDUID command: 4Bh + 3 dummy address bytes + 8 dummy clocks + 16 data bytes
        let mut cmd_buf = [0u8; 5];
        cmd_buf[0] = Opcode::ReadUniqueID as u8;
        let mut data_buf = [0u8; 16];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf)
    }

    // --- 1. Write Status Register (WRSR) ---

    /// Write the status register (WRSR 01h). Writes BP0–BP3, QE, SRWD (non-volatile).
    /// Precedes with WREN. Datasheet Table 6.3.
    pub async fn write_status(&mut self, value: StatusWrite) -> Result<(), Error> {
        self.enable_write_latch().await?;
        let buf = [Opcode::WriteStatus as u8, value.to_byte()];
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Write the volatile status register. Uses VSRWE (50h) then WRSR (01h); does not set WEL.
    /// Datasheet: "Volatile Status Register Write Enable (50h) instruction is required..."
    pub async fn write_status_volatile(&mut self, value: StatusWrite) -> Result<(), Error> {
        let buf_vsrwe = [Opcode::VolatileStatusWriteEnable as u8];
        self.spi.write(&buf_vsrwe).await?;
        let buf_wrsr = [Opcode::WriteStatus as u8, value.to_byte()];
        self.spi.write(&buf_wrsr).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Clear the write enable latch (WRDI 04h).
    pub async fn disable_write_latch(&mut self) -> Result<(), Error> {
        let buf = [Opcode::WriteDisable as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    // --- 2. Fast Read (0Bh) ---

    /// Fast Read (0Bh): opcode + 3-byte address + 1 dummy byte (8 cycles), then data.
    pub async fn read_fast(&mut self, address: i32, read_buf: &mut [u8]) -> Result<(), Error> {
        self.read_fast_with_dummy(address, read_buf, 1).await
    }

    /// Fast Read (0Bh) with configurable dummy bytes. `dummy_byte_count` 1 = 8 cycles (default), 2 = 16 cycles; datasheet Table 6.11.
    pub async fn read_fast_with_dummy(
        &mut self,
        address: i32,
        read_buf: &mut [u8],
        dummy_byte_count: u8,
    ) -> Result<(), Error> {
        read_fast_style(
            &mut self.spi,
            Opcode::FastRead as u8,
            address,
            read_buf,
            dummy_byte_count,
        )
        .await
    }

    /// Fast Read Dual Output (3Bh): opcode + 3-byte address + 1 dummy byte (8 cycles). Datasheet Table 6.11.
    /// For actual dual-line data throughput the SPI transport must support Dual SPI.
    pub async fn read_fast_dual_output(&mut self, address: i32, read_buf: &mut [u8]) -> Result<(), Error> {
        read_fast_style(
            &mut self.spi,
            Opcode::FastReadDualOutput as u8,
            address,
            read_buf,
            1,
        )
        .await
    }

    /// Fast Read Dual I/O (BBh): opcode + 3-byte address + 1 dummy byte (4 cycles typical). Datasheet Table 6.11.
    /// For actual dual I/O throughput the SPI transport must support Dual SPI.
    pub async fn read_fast_dual_io(&mut self, address: i32, read_buf: &mut [u8]) -> Result<(), Error> {
        read_fast_style(
            &mut self.spi,
            Opcode::FastReadDualIo as u8,
            address,
            read_buf,
            1,
        )
        .await
    }

    /// Fast Read Quad Output (6Bh): opcode + 3-byte address + 1 dummy byte (8 cycles). Datasheet Table 6.11.
    /// For actual quad-line data throughput the SPI transport must support Quad SPI (QE=1).
    pub async fn read_fast_quad_output(&mut self, address: i32, read_buf: &mut [u8]) -> Result<(), Error> {
        read_fast_style(
            &mut self.spi,
            Opcode::FastReadQuadOutput as u8,
            address,
            read_buf,
            1,
        )
        .await
    }

    /// Fast Read Quad I/O (EBh): opcode + 3-byte address + 1 dummy byte (6 cycles typical). Datasheet Table 6.11.
    /// For actual quad I/O throughput the SPI transport must support Quad SPI (QE=1).
    pub async fn read_fast_quad_io(&mut self, address: i32, read_buf: &mut [u8]) -> Result<(), Error> {
        read_fast_style(
            &mut self.spi,
            Opcode::FastReadQuadIo as u8,
            address,
            read_buf,
            1,
        )
        .await
    }

    // --- 3. Suspend / Resume ---

    /// Suspend an in-progress program or erase (75h). Datasheet Table 5.3.
    pub async fn suspend(&mut self) -> Result<(), Error> {
        let buf = [Opcode::Suspend as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    /// Resume a suspended program or erase (7Ah).
    pub async fn resume(&mut self) -> Result<(), Error> {
        let buf = [Opcode::Resume as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    // --- 4. Software reset ---

    /// Software reset: Reset Enable (66h) then Reset Device (99h). JEDEC standard.
    pub async fn software_reset(&mut self) -> Result<(), Error> {
        self.spi.write(&[Opcode::ResetEnable as u8]).await?;
        self.spi.write(&[Opcode::ResetDevice as u8]).await?;
        Ok(())
    }

    // --- 5. Deep Power-Down ---

    /// Enter deep power-down (B9h). Consumption ~1 µA; exit with `exit_deep_power_down`.
    pub async fn enter_deep_power_down(&mut self) -> Result<(), Error> {
        let buf = [Opcode::DeepPowerDownEnter as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    /// Exit deep power-down (ABh). Allow wake-up delay (e.g. 3 µs @ 3V) before next command. Datasheet 65h–67h.
    pub async fn exit_deep_power_down(&mut self) -> Result<(), Error> {
        let buf = [Opcode::DeepPowerDownExit as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    // --- 6. SFDP ---

    /// Read Serial Flash Discoverable Parameters (5Ah). `offset` is byte offset into SFDP space (3-byte address).
    pub async fn read_sfdp(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        if offset > 0xFF_FFFF || buf.len() > PAGE_SIZE as usize {
            return Err(Error::AddressOutOfBounds(offset as i32));
        }
        if buf.is_empty() {
            return Err(Error::BufferSizeInvalid(0));
        }
        let mut cmd_buf = [0u8; 4];
        cmd_buf[0] = Opcode::ReadSfdp as u8;
        cmd_buf[1..4].copy_from_slice(&offset.to_be_bytes()[1..4]);
        self.spi.read(&cmd_buf, buf).await?;
        Ok(())
    }

    // --- 7. 4-byte addressing ---

    /// Enter 4-byte address mode (EN4B B7h). Use with `read_4byte`, `write_page_4byte`, `erase_4byte`.
    pub async fn enter_4byte_address_mode(&mut self) -> Result<(), Error> {
        let buf = [Opcode::Enter4Byte as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    /// Exit 4-byte address mode (EX4B 29h).
    pub async fn exit_4byte_address_mode(&mut self) -> Result<(), Error> {
        let buf = [Opcode::Exit4Byte as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    /// Read using 4-byte address (Read 13h). Call after `enter_4byte_address_mode`.
    pub async fn read_4byte(&mut self, address: u32, read_buf: &mut [u8]) -> Result<(), Error> {
        if address > CHIP_SIZE || read_buf.len() + (address as usize) > (CHIP_SIZE as usize) {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        if read_buf.is_empty() || read_buf.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(read_buf.len()));
        }
        let mut cmd_buf = [0u8; 5];
        cmd_buf[0] = Opcode::Read4Byte as u8;
        cmd_buf[1..5].copy_from_slice(&address.to_be_bytes());
        self.spi.read(&cmd_buf, read_buf).await?;
        Ok(())
    }

    /// Page program with 4-byte address (4PP 12h). Call after `enter_4byte_address_mode`.
    pub async fn write_page_4byte(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        if address > CHIP_SIZE || data.len() + (address as usize) > (CHIP_SIZE as usize) {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        if data.is_empty() || data.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(data.len()));
        }
        let mut buf = [0u8; 5 + PAGE_SIZE as usize];
        buf[0] = Opcode::PageProgram4Byte as u8;
        buf[1..5].copy_from_slice(&address.to_be_bytes());
        buf[5..5 + data.len()].copy_from_slice(data);
        self.enable_write_latch().await?;
        self.spi.write(&buf[..5 + data.len()]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Erase with 4-byte address (4SER 21h, 4BER32 5Ch, 4BER64 DCh). Call after `enter_4byte_address_mode`.
    pub async fn erase_4byte(
        &mut self,
        erase_type: FlashEraseSize,
        address: u32,
    ) -> Result<(), Error> {
        if address > CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        let (opcode, size) = match erase_type {
            FlashEraseSize::EraseSize4K => (Opcode::SectorErase4Byte, 4 * 1024u32),
            FlashEraseSize::EraseSize32K => (Opcode::BlockErase32_4Byte, 32 * 1024),
            FlashEraseSize::EraseSize64K => (Opcode::BlockErase64_4Byte, 64 * 1024),
        };
        if address + size > CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        let mut buf = [0u8; 5];
        buf[0] = opcode as u8;
        buf[1..5].copy_from_slice(&address.to_be_bytes());
        self.enable_write_latch().await?;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Quad Input Page Program (PPQ 32h) with 3-byte address. Table 6.3. Requires WREN.
    /// For actual quad data throughput the SPI transport must support QSPI; otherwise data is sent on a single line.
    pub async fn write_page_quad(&mut self, address: i32, data: &[u8]) -> Result<(), Error> {
        if address < 0 || data.len() + (address as usize) > (CHIP_SIZE as usize) {
            return Err(Error::AddressOutOfBounds(address));
        }
        if data.is_empty() || data.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(data.len()));
        }
        let mut buf = [0u8; 4 + PAGE_SIZE as usize];
        buf[0] = Opcode::PageProgramQuad as u8;
        buf[1..4].copy_from_slice(&address.to_be_bytes()[1..4]);
        buf[4..4 + data.len()].copy_from_slice(data);
        self.enable_write_latch().await?;
        self.spi.write(&buf[..4 + data.len()]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Quad Input Page Program with 4-byte address (4PPQ 34h). Table 6.3. Requires WREN.
    /// Call after `enter_4byte_address_mode`.
    pub async fn write_page_4byte_quad(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        if address > CHIP_SIZE || data.len() + (address as usize) > (CHIP_SIZE as usize) {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        if data.is_empty() || data.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(data.len()));
        }
        let mut buf = [0u8; 5 + PAGE_SIZE as usize];
        buf[0] = Opcode::PageProgramQuad4Byte as u8;
        buf[1..5].copy_from_slice(&address.to_be_bytes());
        buf[5..5 + data.len()].copy_from_slice(data);
        self.enable_write_latch().await?;
        self.spi.write(&buf[..5 + data.len()]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Erase Information Row (IRER 64h). Table 6.3. Requires WREN.
    /// `address` is the row address (3-byte); see datasheet for Information Row layout.
    pub async fn erase_information_row(&mut self, address: i32) -> Result<(), Error> {
        if address < 0 {
            return Err(Error::AddressOutOfBounds(address));
        }
        let mut buf = address.to_be_bytes();
        buf[0] = Opcode::EraseInformationRow as u8;
        self.enable_write_latch().await?;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Program Information Row (IRP 62h). Table 6.3. Requires WREN.
    /// `address` is the row/offset (3-byte); `data` up to page size. See datasheet for IR layout.
    pub async fn program_information_row(&mut self, address: i32, data: &[u8]) -> Result<(), Error> {
        if address < 0 {
            return Err(Error::AddressOutOfBounds(address));
        }
        if data.is_empty() || data.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(data.len()));
        }
        let mut buf = [0u8; 4 + PAGE_SIZE as usize];
        buf[0] = Opcode::ProgramInformationRow as u8;
        buf[1..4].copy_from_slice(&address.to_be_bytes()[1..4]);
        buf[4..4 + data.len()].copy_from_slice(data);
        self.enable_write_latch().await?;
        self.spi.write(&buf[..4 + data.len()]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read Information Row (IRRD; opcode 4Ah). `offset` is 3-byte row/offset; read into `buf`. Opcode per datasheet.
    pub async fn read_information_row(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        if offset > 0xFF_FFFF {
            return Err(Error::AddressOutOfBounds(offset as i32));
        }
        if buf.is_empty() || buf.len() > PAGE_SIZE as usize {
            return Err(Error::BufferSizeInvalid(buf.len()));
        }
        let mut cmd_buf = [0u8; 4];
        cmd_buf[0] = Opcode::ReadInformationRow as u8;
        cmd_buf[1..4].copy_from_slice(&offset.to_be_bytes()[1..4]);
        self.spi.read(&cmd_buf, buf).await?;
        Ok(())
    }

    // --- 8. Function / Read / Extended Read registers ---

    /// Read Function Register (RDFR 48h). Bits: IRL3–IRL0, ESUS, PSUS, TBS, Dedicated RESET# Disable.
    pub async fn read_function_register(&mut self) -> Result<u8, Error> {
        let mut buf = [Opcode::ReadFunctionReg as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }

    /// Write Function Register (WRFR 42h). Requires WREN. Datasheet Table 6.5–6.6.
    pub async fn write_function_register(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::WriteFunctionReg as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Set Read Parameters non-volatile (SRPNV 65h). P7–P0: HOLD#/RESET#, dummy cycles, wrap, burst length.
    pub async fn set_read_parameters_nv(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::SetReadParamsNv as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Set Read Parameters volatile (SRPV 63h).
    pub async fn set_read_parameters_volatile(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::SetReadParamsVolatile as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read Read Register (opcode 0Fh). P7–P0 per datasheet Table 6.7–6.8.
    pub async fn read_read_register(&mut self) -> Result<u8, Error> {
        let mut buf = [Opcode::ReadReadReg as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }

    /// Set Extended Read Parameters non-volatile (SERPNV 85h). Only ODS (EB7:5) is writable; EB4:0 are read-only.
    /// Datasheet Table 6.12–6.13.
    pub async fn set_extended_read_parameters_nv(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::SetExtendedReadParamsNv as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Set Extended Read Parameters volatile (SERPV 83h). Only ODS (EB7:5) is writable.
    pub async fn set_extended_read_parameters_volatile(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::SetExtendedReadParamsVolatile as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Set Extended Read Parameters non-volatile from driver strength only (SERPNV 85h).
    /// EB7:5 = ODS; EB4:0 are read-only and not affected by this command.
    pub async fn set_extended_read_parameters_nv_with_driver_strength(
        &mut self,
        driver_strength: DriverStrength,
    ) -> Result<(), Error> {
        self.set_extended_read_parameters_nv(driver_strength.to_register_byte())
            .await
    }

    /// Set Extended Read Parameters volatile from driver strength only (SERPV 83h).
    pub async fn set_extended_read_parameters_volatile_with_driver_strength(
        &mut self,
        driver_strength: DriverStrength,
    ) -> Result<(), Error> {
        self.set_extended_read_parameters_volatile(driver_strength.to_register_byte())
            .await
    }

    /// Read Extended Read Register (opcode 4Fh). Table 6.12–6.13: ODS (EB7:5) writable; EB4 reserved;
    /// EB3:0 read-only (WIP, PROT_E, P_ERR, E_ERR). These read-only bits are not affected by SERPNV/SERPV.
    pub async fn read_extended_read_register(&mut self) -> Result<ExtendedReadStatus, Error> {
        let mut buf = [Opcode::ReadExtendedReadReg as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(ExtendedReadStatus::from_byte(buf[1]))
    }

    /// Read operation error bits (P_ERR, E_ERR, PROT_E) from the Extended Read Register.
    /// Use after an erase/program to check for failure or protection violation.
    pub async fn read_operation_errors(&mut self) -> Result<OperationErrors, Error> {
        let status = self.read_extended_read_register().await?;
        Ok(OperationErrors::from(status))
    }

    /// Read AutoBoot Register (RDABR 14h). Table 6.3 companion read for WRABR.
    pub async fn read_autoboot_register(&mut self) -> Result<u8, Error> {
        let mut buf = [Opcode::ReadAutoBootReg as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }

    /// Write AutoBoot Register (WRABR 15h). Table 6.3. Requires WREN.
    pub async fn write_autoboot_register(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::WriteAutoBootReg as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read Bank Address Register (RDBR 16h). EXTADD (3- vs 4-byte addressing) in bit 7.
    pub async fn read_bank_register(&mut self) -> Result<u8, Error> {
        let mut buf = [Opcode::ReadBankReg as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }

    /// Write non-volatile Bank Address Register (WRBRNV 18h). Table 6.3. Requires WREN.
    pub async fn write_bank_register_nv(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::WriteBankRegNv as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Write volatile Bank Address Register (WRBRV C5h). Table 6.3. Requires WREN.
    pub async fn write_bank_register_volatile(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::WriteBankRegVolatile as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Write volatile Bank Address Register without WREN (WRBRV 17h). Table 6.3 Note 2.
    pub async fn write_bank_register_volatile_no_wren(&mut self, value: u8) -> Result<(), Error> {
        self.spi
            .write(&[Opcode::WriteBankRegVolatileNoWren as u8, value])
            .await?;
        Ok(())
    }

    // --- 9. Advanced Sector/Block Protection (ASP) ---

    /// Read Advanced Sector/Block Protection Register (RDASP 2Bh). 2 bytes.
    pub async fn read_asp(&mut self) -> Result<[u8; 2], Error> {
        let cmd_buf = [Opcode::RdAsp as u8, 0, 0];
        let mut data_buf = [0u8; 2];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf)
    }

    /// Program ASP (PGASP 2Fh). Requires WREN. Datasheet Table 6.19–6.20.
    pub async fn program_asp(&mut self, value: [u8; 2]) -> Result<(), Error> {
        self.enable_write_latch().await?;
        let buf = [Opcode::PgAsp as u8, value[0], value[1]];
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read Password (RDPWD E7h). 8 bytes. Only readable before password protection mode is selected.
    pub async fn read_password(&mut self) -> Result<[u8; 8], Error> {
        let cmd_buf = [Opcode::RdPwd as u8];
        let mut data_buf = [0u8; 8];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf)
    }

    /// Program Password (PGPWD E8h). Requires WREN. 8 bytes. OTP.
    pub async fn program_password(&mut self, password: [u8; 8]) -> Result<(), Error> {
        self.enable_write_latch().await?;
        let mut buf = [0u8; 9];
        buf[0] = Opcode::PgPwd as u8;
        buf[1..9].copy_from_slice(&password);
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Unlock Password (UNPWD E9h). 8 bytes. Required in password mode to clear PPB Lock.
    pub async fn unlock_password(&mut self, password: [u8; 8]) -> Result<(), Error> {
        let mut buf = [0u8; 9];
        buf[0] = Opcode::UnPwd as u8;
        buf[1..9].copy_from_slice(&password);
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read PPB Lock Bit (RDPLB A7h). Returns one byte (PPBLK, FREEZE, etc.). Datasheet Table 6.22.
    pub async fn read_ppb_lock(&mut self) -> Result<u8, Error> {
        let mut buf = [Opcode::RdPpbLock as u8, 0];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }

    /// Write PPB Lock Bit (WRPLB A6h). Clears PPB Lock to 0 (locks PPBs). Requires WREN.
    pub async fn write_ppb_lock(&mut self, value: u8) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::WrPpbLock as u8, value]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Set FREEZE bit (SFRZ 91h). Locks BP3–0, TBS, TBPARM, Information Rows until power cycle.
    pub async fn set_freeze(&mut self) -> Result<(), Error> {
        let buf = [Opcode::SetFreeze as u8];
        self.spi.write(&buf).await?;
        Ok(())
    }

    /// Read PPB for sector/block (RDPPB FCh). Address must be sector/block start (3-byte). Returns 0 = protected, 0xFF = not.
    pub async fn read_ppb(&mut self, address: i32) -> Result<u8, Error> {
        if address < 0 || (address as u32) >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address));
        }
        let mut cmd_buf = address.to_be_bytes();
        cmd_buf[0] = Opcode::RdPpb as u8;
        let mut data_buf = [0u8; 1];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf[0])
    }

    /// Program PPB for sector/block (PGPPB FDh). Programs PPB to 0 (protect). Requires WREN; PPB Lock must be 1.
    pub async fn program_ppb(&mut self, address: i32) -> Result<(), Error> {
        if address < 0 || (address as u32) >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address));
        }
        self.enable_write_latch().await?;
        let mut buf = address.to_be_bytes();
        buf[0] = Opcode::PgPpb as u8;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Erase all PPBs (ERPPB E4h). Requires WREN.
    pub async fn erase_ppb(&mut self) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::ErasePpb as u8]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read DYB for sector/block (RDDYB FAh). 0 = protected, 0xFF = not.
    pub async fn read_dyb(&mut self, address: i32) -> Result<u8, Error> {
        if address < 0 || (address as u32) >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address));
        }
        let mut cmd_buf = address.to_be_bytes();
        cmd_buf[0] = Opcode::RdDyb as u8;
        let mut data_buf = [0u8; 1];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf[0])
    }

    /// Write DYB for sector/block (WRDYB FBh). 0 = protect, 0xFF = unprotect. Requires WREN.
    pub async fn write_dyb(&mut self, address: i32, value: u8) -> Result<(), Error> {
        if address < 0 || (address as u32) >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address));
        }
        self.enable_write_latch().await?;
        let mut buf = [0u8; 5];
        buf[0] = Opcode::WrDyb as u8;
        buf[1..4].copy_from_slice(&address.to_be_bytes()[1..4]);
        buf[4] = value;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Gang lock (GBLK 7Eh). Locks all sectors/blocks per current PPB/DYB. Requires WREN.
    pub async fn gang_lock(&mut self) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::GangLock as u8]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Gang unlock (GBUN 98h). Requires WREN.
    pub async fn gang_unlock(&mut self) -> Result<(), Error> {
        self.enable_write_latch().await?;
        self.spi.write(&[Opcode::GangUnlock as u8]).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read PPB with 4-byte address (4RDPPB E2h). Use after `enter_4byte_address_mode`.
    pub async fn read_ppb_4byte(&mut self, address: u32) -> Result<u8, Error> {
        if address >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        let mut cmd_buf = [0u8; 5];
        cmd_buf[0] = Opcode::RdPpb4Byte as u8;
        cmd_buf[1..5].copy_from_slice(&address.to_be_bytes());
        let mut data_buf = [0u8; 1];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf[0])
    }

    /// Program PPB with 4-byte address (4PGPPB E3h).
    pub async fn program_ppb_4byte(&mut self, address: u32) -> Result<(), Error> {
        if address >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        self.enable_write_latch().await?;
        let mut buf = [0u8; 5];
        buf[0] = Opcode::PgPpb4Byte as u8;
        buf[1..5].copy_from_slice(&address.to_be_bytes());
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }

    /// Read DYB with 4-byte address (4RDDYB E0h).
    pub async fn read_dyb_4byte(&mut self, address: u32) -> Result<u8, Error> {
        if address >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        let mut cmd_buf = [0u8; 5];
        cmd_buf[0] = Opcode::RdDyb4Byte as u8;
        cmd_buf[1..5].copy_from_slice(&address.to_be_bytes());
        let mut data_buf = [0u8; 1];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf[0])
    }

    /// Write DYB with 4-byte address (4WRDYB E1h).
    pub async fn write_dyb_4byte(&mut self, address: u32, value: u8) -> Result<(), Error> {
        if address >= CHIP_SIZE {
            return Err(Error::AddressOutOfBounds(address as i32));
        }
        self.enable_write_latch().await?;
        let mut buf = [0u8; 6];
        buf[0] = Opcode::WrDyb4Byte as u8;
        buf[1..5].copy_from_slice(&address.to_be_bytes());
        buf[5] = value;
        self.spi.write(&buf).await?;
        self.wait_done().await?;
        Ok(())
    }
}

/// Shared helper for Fast Read–style commands (0Bh, 3Bh, BBh, 6Bh, EBh): opcode + 3-byte address + dummy bytes, then read.
async fn read_fast_style<S>(
    spi: &mut S,
    opcode: u8,
    address: i32,
    read_buf: &mut [u8],
    dummy_byte_count: u8,
) -> Result<(), Error>
where
    S: Spi,
{
    if dummy_byte_count == 0 || dummy_byte_count > 2 {
        return Err(Error::BufferSizeInvalid(dummy_byte_count as usize));
    }
    if address < 0 || read_buf.len() + (address as usize) > (CHIP_SIZE as usize) {
        return Err(Error::AddressOutOfBounds(address));
    }
    if read_buf.is_empty() || read_buf.len() > PAGE_SIZE as usize {
        return Err(Error::BufferSizeInvalid(read_buf.len()));
    }
    let cmd_len = 4 + dummy_byte_count as usize;
    let mut cmd_buf = [0u8; 6]; // opcode + 3 addr + up to 2 dummy
    cmd_buf[0] = opcode;
    cmd_buf[1..4].copy_from_slice(&address.to_be_bytes()[1..4]);
    spi.read(&cmd_buf[..cmd_len], read_buf).await?;
    Ok(())
}

fn prepare_read_command_buffer(
    address: i32,
    read_buf_len: usize,
    read_cmd_buf: &mut [u8],
) -> Result<(), Error> {
    if address < 0 || read_buf_len + (address as usize) > (CHIP_SIZE as usize) {
        return Err(Error::AddressOutOfBounds(address));
    }
    if read_buf_len < 1 || read_buf_len > PAGE_SIZE as usize {
        return Err(Error::BufferSizeInvalid(read_buf_len));
    }
    let read_cmd: ReadWriteCmd =
        ReadWriteCmd::default().with_write_or_read(0b011);
    read_cmd_buf[..4].copy_from_slice(&address.to_be_bytes());
    read_cmd_buf[0] = read_cmd.into_bytes()[0];
    Ok(())
}

fn prepare_write_command_buffer(
    address: i32,
    data: &[u8],
    buf: &mut [u8],
) -> Result<(), Error> {
    if address < 0 || buf.len() + (address as usize) > (CHIP_SIZE as usize) {
        return Err(Error::AddressOutOfBounds(address));
    }
    if data.len() < 1 || data.len() > PAGE_SIZE as usize {
        return Err(Error::BufferSizeInvalid(data.len()));
    }
    let write_cmd = ReadWriteCmd::default().with_write_or_read(0b010);

    buf[0..4].copy_from_slice(&address.to_be_bytes());
    buf[0] = write_cmd.into_bytes()[0];

    buf[4..4 + data.len()].copy_from_slice(data);

    Ok(())
}

fn prepare_erase_command_buffer(
    erase_type: &FlashEraseSize,
    address: i32,
) -> Result<[u8; 4], Error> {
    if address < 0 || erase_type.validate_address(address).is_err() {
        return Err(Error::AddressOutOfBounds(address));
    }
    let instruction: Opcode = match erase_type {
        FlashEraseSize::EraseSize4K => Opcode::SectorErase,
        FlashEraseSize::EraseSize32K => Opcode::BlockErase32,
        FlashEraseSize::EraseSize64K => Opcode::BlockErase64,
    };

    let mut buf = address.to_be_bytes();
    buf[0] = instruction as u8;
    Ok(buf)
}

#[cfg(feature = "spi-device")]
pub mod spi_device;

#[cfg(feature = "embedded-storage")]
mod storage_impl {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind};
    use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};

    impl NorFlashError for Error {
        fn kind(&self) -> NorFlashErrorKind {
            match self {
                Error::AddressOutOfBounds(_) | Error::BufferSizeInvalid(_) => {
                    NorFlashErrorKind::OutOfBounds
                }
                Error::UnexpectedStatus => NorFlashErrorKind::Other,
                Error::SpiTimeoutError(_) => NorFlashErrorKind::Other,
                _ => NorFlashErrorKind::Other,
            }
        }
    }

    impl<S: Spi, H: HardwareInterface> ErrorType for Is25lp128f<S, H> {
        type Error = Error;
    }

    #[allow(async_fn_in_trait)]
    impl<S: Spi, H: HardwareInterface> ReadNorFlash for Is25lp128f<S, H> {
        const READ_SIZE: usize = 1;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let address = i32::try_from(offset).map_err(|_| Error::AddressOutOfBounds(-1))?;
            let mut remaining = bytes;
            let mut addr = address;
            while !remaining.is_empty() {
                let chunk_len = remaining.len().min(PAGE_SIZE as usize);
                let (chunk, rest) = remaining.split_at_mut(chunk_len);
                Is25lp128f::read(self, addr, chunk).await?;
                addr += chunk_len as i32;
                remaining = rest;
            }
            Ok(())
        }

        fn capacity(&self) -> usize {
            CHIP_SIZE as usize
        }
    }

    #[allow(async_fn_in_trait)]
    impl<S: Spi, H: HardwareInterface> NorFlash for Is25lp128f<S, H> {
        const WRITE_SIZE: usize = PAGE_SIZE as usize;
        const ERASE_SIZE: usize = SECTOR_SIZE as usize;

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if from >= to || to > CHIP_SIZE {
                return Err(Error::AddressOutOfBounds(from as i32));
            }
            if from % SECTOR_SIZE != 0 || to % SECTOR_SIZE != 0 {
                return Err(Error::AddressOutOfBounds(from as i32));
            }
            let mut sector_start = from;
            while sector_start < to {
                Is25lp128f::erase(self, FlashEraseSize::EraseSize4K, sector_start as i32)
                    .await?;
                sector_start += SECTOR_SIZE;
            }
            Ok(())
        }

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if offset > CHIP_SIZE {
                return Err(Error::AddressOutOfBounds(offset as i32));
            }
            let mut remaining = bytes;
            let mut addr = offset as i32;
            while !remaining.is_empty() {
                let chunk_len = remaining.len().min(PAGE_SIZE as usize);
                let (chunk, rest) = remaining.split_at(chunk_len);
                Is25lp128f::write_page(self, addr, chunk).await?;
                addr += chunk_len as i32;
                remaining = rest;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pretty_assertions::assert_eq;
    use rstest::rstest;

    #[rstest]
    #[case(-1, 10, vec![0; 4], Err(Error::AddressOutOfBounds(-1)))]
    #[case(CHIP_SIZE as i32, 10, vec![0; 4], Err(Error::AddressOutOfBounds(CHIP_SIZE as i32)))]
    #[case(10, 0, vec![0; 4], Err(Error::BufferSizeInvalid(0)))]
    #[case(10, 1, vec![0; 4], Ok(()))]
    #[case(10, 4, vec![0; 4], Ok(()))]
    #[case(0, PAGE_SIZE as usize, vec![0; 4], Ok(()))]
    #[case((CHIP_SIZE - PAGE_SIZE - 4) as i32, PAGE_SIZE as usize, vec![0; 4], Ok(()))]
    fn test_prepare_read_command_buffer(
        #[case] address: i32,
        #[case] read_buf_len: usize,
        #[case] mut read_cmd_buf: Vec<u8>,
        #[case] expected: Result<(), Error>,
    ) {
        let result = prepare_read_command_buffer(
            address,
            read_buf_len,
            &mut read_cmd_buf,
        );
        assert_eq!(result, expected);

        if result.is_ok() {
            let address_bytes = address.to_be_bytes();
            let expected_opcode = ReadWriteCmd::default()
                .with_write_or_read(0b011)
                .into_bytes()[0];

            assert_eq!(read_cmd_buf[0], expected_opcode);
            assert_eq!(read_cmd_buf[1..4], address_bytes[1..4]);
        }
    }

    #[rstest]
    #[case(-1, &[0xAA, 0xBB, 0xCC, 0xDD], vec![0; PAGE_SIZE as usize + 4], Err(Error::AddressOutOfBounds(-1)))]
    #[case(CHIP_SIZE as i32, &[0xAA, 0xBB, 0xCC, 0xDD], vec![0; PAGE_SIZE as usize + 4], Err(Error::AddressOutOfBounds(CHIP_SIZE as i32)))]
    #[case(10, &[0; 0], vec![0; PAGE_SIZE as usize + 4], Err(Error::BufferSizeInvalid(0)))]
    #[case(10, &[0xAA, 0xBB, 0xCC, 0xDD], vec![0; PAGE_SIZE as usize + 4], Ok(()))]
    #[case(0, &[0xFF; 4], vec![0; PAGE_SIZE as usize + 4], Ok(()))]
    #[case((CHIP_SIZE - PAGE_SIZE - 4) as i32, &[0xFF; PAGE_SIZE as usize], vec![0; PAGE_SIZE as usize + 4], Ok(()))]
    fn test_prepare_write_command_buffer(
        #[case] address: i32,
        #[case] data: &[u8],
        #[case] mut buf: Vec<u8>,
        #[case] expected: Result<(), Error>,
    ) {
        let result = prepare_write_command_buffer(address, data, &mut buf);
        assert_eq!(result, expected);

        if result.is_ok() {
            let address_bytes = address.to_be_bytes();
            let expected_opcode = ReadWriteCmd::default()
                .with_write_or_read(0b010)
                .into_bytes()[0];
            assert_eq!(buf[0], expected_opcode);
            assert_eq!(buf[1..4], address_bytes[1..4]);
            assert_eq!(&buf[4..4 + data.len()], data);
        }
    }

    #[rstest]
    #[case(FlashEraseSize::EraseSize4K, -1, Err(Error::AddressOutOfBounds(-1)))]
    #[case(FlashEraseSize::EraseSize4K, CHIP_SIZE as i32, Err(Error::AddressOutOfBounds(CHIP_SIZE as i32)))]
    #[case(FlashEraseSize::EraseSize4K, 0x100, Ok([Opcode::SectorErase as u8, 0, 1, 0]))]
    #[case(FlashEraseSize::EraseSize32K, 0x200, Ok([Opcode::BlockErase32 as u8, 0, 2, 0]))]
    #[case(FlashEraseSize::EraseSize64K, 0x300, Ok([Opcode::BlockErase64 as u8, 0, 3, 0]))]
    #[case(FlashEraseSize::EraseSize64K, (CHIP_SIZE - 64 * 1024) as i32, Ok([Opcode::BlockErase64 as u8, 255, 0, 0]))]
    #[case(FlashEraseSize::EraseSize4K, (CHIP_SIZE - 4096 + 1) as i32, Err(Error::AddressOutOfBounds((CHIP_SIZE - 4096 + 1) as i32)))]
    #[case(FlashEraseSize::EraseSize4K, (CHIP_SIZE - 4096) as i32, Ok([Opcode::SectorErase as u8, 0xFF, 0xF0, 0x00]))]
    #[case(FlashEraseSize::EraseSize32K, (CHIP_SIZE - 32768 + 1) as i32, Err(Error::AddressOutOfBounds((CHIP_SIZE - 32768 + 1) as i32)))]
    #[case(FlashEraseSize::EraseSize32K, (CHIP_SIZE - 32768) as i32, Ok([Opcode::BlockErase32 as u8, 0xFF, 0x80, 0x00]))]
    #[case(FlashEraseSize::EraseSize64K, (CHIP_SIZE - 65536 + 1) as i32, Err(Error::AddressOutOfBounds((CHIP_SIZE - 65536 + 1) as i32)))]
    #[case(FlashEraseSize::EraseSize64K, (CHIP_SIZE - 65536) as i32, Ok([Opcode::BlockErase64 as u8, 0xFF, 0x00, 0x00]))]
    fn test_prepare_erase_command_buffer(
        #[case] erase_type: FlashEraseSize,
        #[case] address: i32,
        #[case] expected: Result<[u8; 4], Error>,
    ) {
        let result = prepare_erase_command_buffer(&erase_type, address);
        assert_eq!(result, expected);
    }
}
