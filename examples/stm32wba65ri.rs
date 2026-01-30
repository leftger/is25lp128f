//! IS25LP128F example for STM32WBA65RI
//!
//! Demonstrates the driver with embassy-stm32 and the `embedded-storage` NorFlash trait.
//! Wire the IS25LP128F to SPI1 (adjust pins to match your board):
//! - SCK:  PB4 (or your board's SPI1 SCK)
//! - MISO: PA15 (or your board's SPI1 MISO)
//! - MOSI: PB3 (or your board's SPI1 MOSI)
//! - CS:   PA12 (or any GPIO for chip select)
//!
//! # Build & run
//!
//! ```bash
//! cargo build --example stm32wba65ri --no-default-features --features embedded-storage --target thumbv8m.main-none-eabihf --release
//! probe-rs run --chip STM32WBA65RI target/thumbv8m.main-none-eabihf/release/examples/stm32wba65ri
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::{
        AHB5Prescaler, AHBPrescaler, APBPrescaler, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
        VoltageScale,
    },
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
    Config,
};
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::{NorFlash, ReadNorFlash};
use is25lp128f::{HardwareInterface, Is25lp128f, Spi, SECTOR_SIZE};
use {defmt_rtt as _, panic_probe as _};

/// Adapter that implements the driver's Spi using embassy SPI with software CS.
struct EmbassySpiFlash<'a> {
    spi: Spi<'a, embassy_stm32::mode::Async, embassy_stm32::spi::Master>,
    cs: Output<'a>,
}

impl<'a> EmbassySpiFlash<'a> {
    fn new(
        spi: Spi<'a, embassy_stm32::mode::Async, embassy_stm32::spi::Master>,
        cs: Output<'a>,
    ) -> Self {
        Self { spi, cs }
    }

    fn cs_low(&mut self) {
        self.cs.set_low();
    }

    fn cs_high(&mut self) {
        self.cs.set_high();
    }
}

#[allow(async_fn_in_trait)]
impl<'a> Spi for EmbassySpiFlash<'a> {
    async fn configure_spi(&mut self) -> Result<(), is25lp128f::Error> {
        Ok(())
    }

    async fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), is25lp128f::Error> {
        self.cs_low();
        let r = self.spi.transfer_in_place(buf).await.map_err(|_| is25lp128f::Error::SpiConfigError);
        self.cs_high();
        r
    }

    async fn read(
        &mut self,
        read_cmd_buf: &[u8],
        read_buf: &mut [u8],
    ) -> Result<(), is25lp128f::Error> {
        self.cs_low();
        self.spi.write(read_cmd_buf).await.map_err(|_| is25lp128f::Error::SpiWriteError)?;
        self.spi.read(read_buf).await.map_err(|_| is25lp128f::Error::SpiReadError)?;
        self.cs_high();
        Ok(())
    }

    async fn write(&mut self, data: &[u8]) -> Result<(), is25lp128f::Error> {
        self.cs_low();
        let r = self.spi.write(data).await.map_err(|_| is25lp128f::Error::SpiWriteError);
        self.cs_high();
        r
    }
}

/// Delay implementation using embassy Timer.
struct EmbassyDelay;

#[allow(async_fn_in_trait)]
impl HardwareInterface for EmbassyDelay {
    async fn wait_ms(&mut self, timeout_ms: u64) {
        Timer::after(Duration::from_millis(timeout_ms)).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("IS25LP128F STM32WBA65RI example (embedded-storage NorFlash)");

    let mut config = Config::default();
    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL30,
        divr: Some(PllDiv::DIV5),
        divq: None,
        divp: Some(PllDiv::DIV30),
        frac: Some(0),
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.apb7_pre = APBPrescaler::DIV1;
    config.rcc.ahb5_pre = AHB5Prescaler::DIV4;
    config.rcc.voltage_scale = VoltageScale::RANGE1;
    config.rcc.sys = Sysclk::PLL1_R;

    let p = embassy_stm32::init(config);

    // SPI1: SCK=PB4, MISO=PA15, MOSI=PB3 (adjust for your board)
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(10_000_000);
    let spi = Spi::new(
        p.SPI1,
        p.PB4,
        p.PA15,
        p.PB3,
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
        spi_config,
    );

    let cs = Output::new(p.PA12, Level::High, Speed::VeryHigh);

    let spi_flash = EmbassySpiFlash::new(spi, cs);
    let mut flash = Is25lp128f::new(spi_flash, EmbassyDelay);

    // Use as embedded-storage NorFlash
    let capacity = flash.capacity();
    info!("Flash capacity: {} bytes", capacity);

    // Read JEDEC ID (optional sanity check)
    if let Ok(jedec) = flash.read_jedec_id().await {
        info!("JEDEC ID: mfr=0x{:02x} type=0x{:02x} cap=0x{:02x}", jedec.mfr_code, jedec.memory_type, jedec.capacity);
    }

    // Example: read first 16 bytes via NorFlash trait
    let mut buf = [0u8; 16];
    if embedded_storage_async::nor_flash::ReadNorFlash::read(&mut flash, 0, &mut buf)
        .await
        .is_ok()
    {
        info!("Read @0: {:?}", buf);
    }

    // Example: erase first sector and write a page (optional; comment out if you don't want to modify flash)
    const OFFSET: u32 = 0;
    let sector_end = SECTOR_SIZE;
    if let Err(e) = NorFlash::erase(&mut flash, OFFSET, sector_end).await {
        warn!("Erase failed: {:?}", e);
    } else {
        info!("Erased sector [0..{}]", sector_end);
        let write_data = [0xAAu8; 256];
        if NorFlash::write(&mut flash, OFFSET, &write_data).await.is_ok() {
            info!("Wrote 256 bytes at 0");
            let mut read_back = [0u8; 256];
            if ReadNorFlash::read(&mut flash, OFFSET, &mut read_back).await.is_ok() {
                info!("Read back: first 8 bytes {:?}", &read_back[..8]);
            }
        }
    }

    info!("Example done.");
}
