//! Adapter from [`embedded_hal_async::spi::SpiDevice`] to this crate's [`Spi`] trait.
//!
//! Use this when your SPI peripheral is a **device** (with its own CS and possibly a shared bus)
//! rather than a raw bus. For example, with [embassy-embedded-hal] you can share one SPI bus
//! across multiple devices; each device gets a `SpiDevice` (bus + CS). Wrap that `SpiDevice` in
//! [`SpiDeviceAdapter`] to use it with [`Is25lp128f`].
//!
//! [embassy-embedded-hal]: https://crates.io/crates/embassy-embedded-hal
//!
//! # Example (embassy-embedded-hal shared SPI)
//!
//! ```ignore
//! use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
//! use embassy_sync::mutex::Mutex;
//! use is25lp128f::{spi_device::SpiDeviceAdapter, Is25lp128f, ...};
//!
//! static SPI_BUS: StaticCell<Mutex<NoopRawMutex, embassy_stm32::spi::Spi<...>>> = StaticCell::new();
//! let spi_bus = SPI_BUS.init(Mutex::new(spi));
//! let flash_cs = Output::new(p.PA12, Level::High, Speed::VeryHigh);
//! let flash_device = SpiDevice::new(spi_bus, flash_cs);
//! let adapter = SpiDeviceAdapter::new(flash_device);
//! let flash = Is25lp128f::new(adapter, EmbassyDelay);
//! ```

use crate::{Error, Spi};
use embedded_hal_async::spi::{Operation, SpiDevice};

/// Wraps an [`embedded_hal_async::spi::SpiDevice`] and implements this crate's [`Spi`] trait.
///
/// Use with [embassy-embedded-hal]'s shared `SpiDevice` when multiple devices share one SPI bus.
#[derive(Debug)]
pub struct SpiDeviceAdapter<D> {
    device: D,
}

impl<D> SpiDeviceAdapter<D> {
    /// Create an adapter from any async SPI device (e.g. shared bus + CS).
    pub fn new(device: D) -> Self {
        Self { device }
    }
}

impl<D> SpiDeviceAdapter<D>
where
    D: SpiDevice<u8>,
{
    fn map_err(_: D::Error) -> Error {
        Error::SpiDevice
    }
}

#[allow(async_fn_in_trait)]
impl<D> Spi for SpiDeviceAdapter<D>
where
    D: SpiDevice<u8>,
{
    async fn configure_spi(&mut self) -> Result<(), Error> {
        Ok(())
    }

    async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.device
            .transfer_in_place(buf)
            .await
            .map_err(SpiDeviceAdapter::<D>::map_err)
    }

    async fn read(
        &mut self,
        read_cmd_buf: &[u8],
        read_buf: &mut [u8],
    ) -> Result<(), Error> {
        let mut ops = [
            Operation::Write(read_cmd_buf),
            Operation::Read(read_buf),
        ];
        self.device
            .transaction(&mut ops)
            .await
            .map_err(SpiDeviceAdapter::<D>::map_err)
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.device
            .write(data)
            .await
            .map_err(SpiDeviceAdapter::<D>::map_err)
    }
}
