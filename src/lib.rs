#![allow(unused)]
#![cfg_attr(not(feature = "std"), no_std)]

pub mod prelude;

use bitflags::bitflags;
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

// is25lp128 rev L0 pp. 21-22
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
enum Opcode {
    /// Set the write enable latch.
    WriteEnable = 0x06,
    /// Clear the write enable latch.
    WriteDisable = 0x04,
    /// Read the 8-bit status register.
    ReadStatus = 0x05,
    /// Write the 8-bit status register. Not all bits are writeable.
    WriteStatus = 0x01,
    /// Sector Erase.
    SectorErase = 0xD7,
    /// Block Erase (32KByte).
    BlockErase32 = 0x52,
    /// Block Erase (64KByte).
    BlockErase64 = 0xD8,
    /// Chip Erase
    ChipErase = 0xC7,
    /// Read Flash ID Register.
    FlashID = 0x9F,
    /// Read Unique ID Number.
    ReadUniqueID = 0x4B,
}

bitflags! {
    /// Status register bits.
    pub struct Status: u8 {
        /// Write in progress.
        const BUSY = 1 << 0;
        /// Status of the **W**rite **E**nable **L**atch.
        const WEL = 1 << 1;
        /// The 4 protection region bits.
        const PROT = 0b00111100;
        /// Quad Enable
        const QE = 1 << 6;
        /// Status Register Write Disable
        const SRWD = 1 << 7;
    }
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
        if status.contains(Status::BUSY | Status::WEL) {
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
        Ok(Status::from_bits_truncate(buf[1]))
    }

    /// Wait until the device is not busy (no erase/write in progress).
    pub async fn wait_done(&mut self) -> Result<(), Error> {
        while self.read_status().await?.contains(Status::BUSY) {
            self.hardware_interface.wait_ms(1).await;
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
        // Total command: [4Bh, addr1, addr2, addr3, dummy] followed by 16 data bytes
        let mut cmd_buf = [0u8; 5]; // 1 byte opcode + 3 dummy address + 1 dummy byte
        cmd_buf[0] = Opcode::ReadUniqueID as u8;
        // Address bytes and dummy byte are already 0

        let mut data_buf = [0u8; 16];
        self.spi.read(&cmd_buf, &mut data_buf).await?;
        Ok(data_buf)
    }
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
